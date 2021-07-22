#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_halt;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v1_compat::OldOutputPin;
use rtic::app;
use rtic::cyccnt::U32Ext;
use stm32f1xx_hal::gpio::{gpioa::*, gpiob::*, gpioc::*, Input, Floating, Analog, Alternate, Output, PushPull, OpenDrain, State};
use stm32f1xx_hal::spi;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::usb::{UsbBus, Peripheral, UsbBusType};
use stm32f1xx_hal::adc::Adc;
use stm32f1xx_hal::pac::ADC1;
use mcp49xx::Mcp49xx;
use mcp49xx::interface::SpiInterface;
use mcp49xx::marker::Resolution12Bit;
use mcp49xx::marker::DualChannel;
use mcp49xx::marker::Unbuffered;

use usb_device::bus;
use usb_device::prelude::*;
use core::convert::Infallible;

use micromath::F32Ext;

const PERIOD: u32 = 100_000_000;

struct X3423DataPins {
	d0: PB4<Output<OpenDrain>>,
	d1: PB6<Output<OpenDrain>>,
	d2: PB7<Output<OpenDrain>>,
	d3: PB8<Output<OpenDrain>>,
	d4: PB9<Output<OpenDrain>>,
	d5: PB10<Output<OpenDrain>>,
	d6: PB11<Output<OpenDrain>>,
	d7: PB12<Output<OpenDrain>>,
}

pub struct X3423 {
	mfpos0: PA5<Analog>,
	mfpos1: PA6<Analog>,
	mfpos2: PA7<Analog>,
	mfpos3: PB0<Analog>,
	mfpos4: PB1<Analog>,
	reset: PB3<Output<OpenDrain>>,
	fd0: PA8<Output<OpenDrain>>,
	fd1: PA9<Output<OpenDrain>>,
	fd2: PA10<Output<OpenDrain>>,
	fd3: PA15<Output<OpenDrain>>,
	data: X3423DataPins,
}

trait OutputPinExt {
	type Error;
	fn set(&mut self, is_high: bool) -> Result<(), Self::Error>;
}

impl<T: OutputPin> OutputPinExt for T {
	type Error = T::Error;
	fn set(&mut self, is_high: bool) -> Result<(), T::Error> {
		if is_high {
			self.set_high()
		}
		else {
			self.set_low()
		}
	}
}

fn wait() {
	cortex_m::asm::delay(10);
}

impl X3423DataPins {
	fn write_data(&mut self, data: u8) -> Result<(), Infallible> {
		self.d0.set(data & 0x01 != 0)?;
		self.d1.set(data & 0x02 != 0)?;
		self.d2.set(data & 0x04 != 0)?;
		self.d3.set(data & 0x08 != 0)?;
		self.d4.set(data & 0x10 != 0)?;
		self.d5.set(data & 0x20 != 0)?;
		self.d6.set(data & 0x40 != 0)?;
		self.d7.set(data & 0x80 != 0)?;
		Ok(())
	}
	
	fn apply_data(&mut self, data: u8, fd_pin: &mut impl embedded_hal::digital::v2::OutputPin) {
		self.write_data(data).unwrap();
		wait();
		fd_pin.set_high();
		wait();
		fd_pin.set_low();
	}
}

impl X3423 {
	pub fn set_motor_enable(&mut self, state: u32) {
		self.data.apply_data((state & 0xFF) as u8, &mut self.fd0);
		self.data.apply_data(((state >> 8) & 0xFF) as u8, &mut self.fd1);
		self.data.apply_data(((state >> 16) & 0x01) as u8, &mut self.fd2);
	}

	fn set_other(&mut self, dasel: u8, dainh: u8, adsel: u8) {
		self.data.apply_data((dasel & 7) | ((dainh & 7) << 3) | ((adsel & 2) << 6), &mut self.fd3);
	}

	pub fn capture_analog_value(&mut self, index: u8, delay: &mut embedded_hal::blocking::delay::DelayUs<u32>) {
		assert!(index < 17);
		let dasel = index & 7;
		let dainh = !(1 << (index >> 3));
		let adsel = 0;
		self.set_other(dasel, dainh, adsel);
		delay.delay_us(1000);
		self.set_other(dasel, 7, adsel);
	}
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
	struct Resources {
		led: PC13<Output<PushPull>>,
		x3423: X3423,
		usb_dev: UsbDevice<'static, UsbBusType>,
		midi: usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>,
		adc: Adc<ADC1>,
		dac: Mcp49xx<SpiInterface<spi::Spi<stm32f1xx_hal::pac::SPI2, spi::Spi2NoRemap, (PB13<Alternate<PushPull>>, spi::NoMiso, PB15<Alternate<PushPull>>), u8>, OldOutputPin<PC15<Output<PushPull>>>>, Resolution12Bit, DualChannel, Unbuffered>,
		delay: stm32f1xx_hal::delay::Delay
	}

	#[init(spawn=[xmain])]
	fn init(cx: init::Context) -> init::LateResources {
		static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

		// Enable cycle counter
		let mut core = cx.core;
		core.DWT.enable_cycle_counter();

		let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

		// Setup clocks
		let mut flash = device.FLASH.constrain();
		let mut rcc = device.RCC.constrain();
		let mut afio = device.AFIO.constrain(&mut rcc.apb2);
		let clocks = rcc
			.cfgr
			.use_hse(8.mhz())
			.sysclk(72.mhz())
			.pclk1(36.mhz())
			.freeze(&mut flash.acr);

		let mut delay = Delay::new(core.SYST, clocks);

		// Setup LED
		let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
		let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
		let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
		let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
		let mut led = gpioc
			.pc13
			.into_push_pull_output_with_state(&mut gpioc.crh, State::Low);
		led.set_low().unwrap();


		// Configure USB
		// BluePill board has a pull-up resistor on the D+ line.
		// Pull the D+ pin down to send a RESET condition to the USB bus.
		let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
		usb_dp.set_low().ok();
		cortex_m::asm::delay(clocks.sysclk().0 / 100);
		
		let usb_dm = gpioa.pa11;
		let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

		let usb_pins = Peripheral {
			usb: device.USB,
			pin_dm: usb_dm,
			pin_dp: usb_dp
		};
		*USB_BUS = Some( UsbBus::new(usb_pins) );
		let usb_bus = USB_BUS.as_ref().unwrap();

		let midi = usbd_midi::midi_device::MidiClass::new(usb_bus, 1, 1).unwrap();
		let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
			.manufacturer("Windfisch")
			.product("Faderboard")
			.serial_number("000000")
			.device_class(usbd_midi::data::usb::constants::USB_CLASS_NONE)
			.build();

		// USB interrupt
		core.NVIC.enable(stm32f1xx_hal::pac::Interrupt::USB_LP_CAN_RX0);

		// Configure SPI
		let mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
		let sck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
		let dac_cs: OldOutputPin<_> = gpioc.pc15.into_push_pull_output_with_state(&mut gpioc.crh, State::High).into();

		let spi = spi::Spi::spi2(device.SPI2, (sck, spi::NoMiso, mosi), spi::Mode { phase : spi::Phase::CaptureOnFirstTransition, polarity : spi::Polarity::IdleLow }, 5.mhz(), clocks, &mut rcc.apb1);
		let mut dac = mcp49xx::Mcp49xx::new_mcp4822(spi, dac_cs);

		let mut adc = stm32f1xx_hal::adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);

		let mut resources = init::LateResources {
			led,
			x3423: X3423 {
				mfpos0: gpioa.pa5.into_analog(&mut gpioa.crl),
				mfpos1: gpioa.pa6.into_analog(&mut gpioa.crl),
				mfpos2: gpioa.pa7.into_analog(&mut gpioa.crl),
				mfpos3: gpiob.pb0.into_analog(&mut gpiob.crl),
				mfpos4: gpiob.pb1.into_analog(&mut gpiob.crl),
				reset: pb3.into_open_drain_output_with_state(&mut gpiob.crl, State::Low),
				fd0: gpioa.pa8.into_open_drain_output(&mut gpioa.crh),
				fd1: gpioa.pa9.into_open_drain_output(&mut gpioa.crh),
				fd2: gpioa.pa10.into_open_drain_output(&mut gpioa.crh),
				fd3: pa15.into_open_drain_output(&mut gpioa.crh),
				data: X3423DataPins {
					d0: pb4.into_open_drain_output(&mut gpiob.crl),
					d1: gpiob.pb6.into_open_drain_output(&mut gpiob.crl),
					d2: gpiob.pb7.into_open_drain_output(&mut gpiob.crl),
					d3: gpiob.pb8.into_open_drain_output(&mut gpiob.crh),
					d4: gpiob.pb9.into_open_drain_output(&mut gpiob.crh),
					d5: gpiob.pb10.into_open_drain_output(&mut gpiob.crh),
					d6: gpiob.pb11.into_open_drain_output(&mut gpiob.crh),
					d7: gpiob.pb12.into_open_drain_output(&mut gpiob.crh),
				}
			},
			usb_dev,
			midi,
			adc,
			dac,
			delay
		};

		cortex_m::asm::delay(5);

		cx.spawn.xmain();

		resources
	}

	
	#[task(resources = [x3423, adc, dac, delay], capacity=10, priority=1)]
	fn xmain(mut c : xmain::Context) {
		c.resources.x3423.reset.set_high();

		c.resources.x3423.set_motor_enable(0x1FFFF);
		for frame in 0..400 {
			for i in 0..17 {
				c.resources.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value(
					((f32::sin((frame as f32 /200. + i as f32 / 17.) * core::f32::consts::PI * 2.) / 2. + 0.5 ) * 2600. + 256.) as u16
				));
				c.resources.delay.delay_us(5_u16);
				c.resources.x3423.capture_analog_value(i, c.resources.delay);
			}
		}

		c.resources.x3423.set_motor_enable( 0xa222 );
		loop {
			let fnord: u16 = c.resources.adc.read(&mut c.resources.x3423.mfpos0).unwrap();
			c.resources.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value( ((fnord as u32) *3895 / 4096 + 200) as u16));
			c.resources.delay.delay_us(5_u16);
			c.resources.x3423.capture_analog_value(1, c.resources.delay);

			let fnord: u16 = c.resources.adc.read(&mut c.resources.x3423.mfpos1).unwrap();
			c.resources.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value( ((fnord as u32) *3895 / 4096 + 200) as u16));
			c.resources.delay.delay_us(5_u16);
			c.resources.x3423.capture_analog_value(5, c.resources.delay);

			let fnord: u16 = c.resources.adc.read(&mut c.resources.x3423.mfpos2).unwrap();
			c.resources.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value( ((fnord as u32) *3895 / 4096 + 200) as u16));
			c.resources.delay.delay_us(5_u16);
			c.resources.x3423.capture_analog_value(9, c.resources.delay);

			let fnord: u16 = c.resources.adc.read(&mut c.resources.x3423.mfpos3).unwrap();
			c.resources.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value( ((fnord as u32) *3895 / 4096 + 200) as u16));
			c.resources.delay.delay_us(5_u16);
			c.resources.x3423.capture_analog_value(13, c.resources.delay);

			let fnord: u16 = c.resources.adc.read(&mut c.resources.x3423.mfpos4).unwrap();
			c.resources.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value( ((fnord as u32) *3895 / 4096 + 200) as u16));
			c.resources.delay.delay_us(5_u16);
			c.resources.x3423.capture_analog_value(15, c.resources.delay);
		}
	
		
		//c.resources.delay.delay_ms(1000_u16);
		c.resources.x3423.set_motor_enable(0);
	}

	#[task(binds = USB_LP_CAN_RX0, spawn=[xmain], resources=[midi, usb_dev, led], priority=2)]
	fn periodic_usb_poll(mut c : periodic_usb_poll::Context) {
		c.resources.usb_dev.poll(&mut[c.resources.midi]);

		let mut message: [u8; 4] = [0; 4];
		while let Ok(len) = c.resources.midi.read(&mut message) {
			if len == 4 {
				c.resources.led.toggle();
			}
		}
	}
	
	extern "C" {
		fn EXTI0();
	}
};
