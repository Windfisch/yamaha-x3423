#![no_main]
#![no_std]

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v1_compat::OldOutputPin;
use rtic::app;
use stm32f1xx_hal::gpio::{gpioa::*, gpiob::*, gpioc::*, Input, Floating, Analog, Alternate, Output, PushPull, OpenDrain, State};
use stm32f1xx_hal::spi;
use stm32f1xx_hal::timer;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::usb::{UsbBus, Peripheral, UsbBusType};
use stm32f1xx_hal::adc::Adc;
use stm32f1xx_hal::pac::{ADC1, TIM2};
use mcp49xx::Mcp49xx;
use mcp49xx::interface::SpiInterface;
use mcp49xx::marker::Resolution12Bit;
use mcp49xx::marker::DualChannel;
use mcp49xx::marker::Unbuffered;
use stm32f1xx_hal::time::Hertz;

use usb_device::prelude::*;
use core::convert::Infallible;

use micromath::F32Ext;

const PERIOD: u32 = 100_000_000;


#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
	use core::mem::MaybeUninit;
	cortex_m::interrupt::disable();

	let led: stm32f1xx_hal::gpio::gpioc::PC13<Input<Floating>> = unsafe { MaybeUninit::uninit().assume_init() };
	let mut reg = unsafe { MaybeUninit::uninit().assume_init() };
	let mut led = led.into_push_pull_output(&mut reg);
	loop {
		let mut blink_thrice = |delay: u32| {
			for _ in 0..3 {
				led.set_low().ok();
				cortex_m::asm::delay(5000000*delay);
				led.set_high().ok();
				cortex_m::asm::delay(10000000);
			}
			cortex_m::asm::delay(10000000);
		};
		blink_thrice(1);
		blink_thrice(4);
		blink_thrice(1);
		cortex_m::asm::delay(10000000);
	}
}




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
		self.data.apply_data((dasel & 7) | ((dainh & 7) << 3) | ((adsel & 3) << 6), &mut self.fd3);
	}

	pub fn capture_analog_value(&mut self, index: u8, delay: &mut impl embedded_hal::blocking::delay::DelayUs<u32>) {
		assert!(index < 17);
		let dasel = index & 7;
		let dainh = !(1 << (index >> 3));
		let adsel = 0;
		self.set_other(dasel, dainh, adsel);
		delay.delay_us(20);
		self.set_other(dasel, 7, adsel);
	}

	pub fn select_analog_value(&mut self, index: u8, delay: &mut impl embedded_hal::blocking::delay::DelayUs<u32>) {
		assert!(index < 4);
		self.set_other(0, 7, index);
		delay.delay_us(20);
	}
}

enum FaderCalibrationPhase {
	NotCalibrating,
	Init,
	ApproachMin,
	ApproachMidFromBelow,
	ApproachMax,
	ApproachMidFromAbove,
	Approach75FromBelow,
	Approach25FromAbove
}

pub struct Fader {
	write_low: u16,
	write_high: u16,
	write_deadzone: u16,
	read_low: u16,
	read_high: u16,
	integrator: f32,

	target_value: Option<f32>,
	time_left: u32,
	last_value: f32,
	last_value_raw: u16,
	
	mid_from_above: u16,
	mid_from_below: u16,
	v25_from_above: u16,

	calibration_phase: FaderCalibrationPhase
}

impl Default for Fader {
	fn default() -> Fader { Fader::new() }
}

const WRITE_LOW_APPROX: u16 = 200;
const WRITE_HIGH_APPROX: u16 = 2900;
const WRITE_25_APPROX: u16 = (WRITE_LOW_APPROX * 3 + WRITE_HIGH_APPROX) / 4;
const WRITE_75_APPROX: u16 = (WRITE_LOW_APPROX + WRITE_HIGH_APPROX * 3) / 4;
const WRITE_MID_APPROX: u16 = (WRITE_LOW_APPROX + WRITE_HIGH_APPROX) / 2;

impl Fader {
	pub fn new() -> Fader {
		Fader {
			write_low: 200,
			write_high: 2900,
			write_deadzone: 0,
			read_low: 0,
			read_high: 4095,
			target_value: None,
			time_left: 0,
			last_value: 0.5,
			last_value_raw: 0,
			mid_from_above: 0,
			mid_from_below: 0,
			v25_from_above: 0,
			integrator: 0.,
			calibration_phase: FaderCalibrationPhase::Init
		}
	}
	pub fn update_value(&mut self, raw: u16) {
		self.last_value = self.cook_raw_read_value(raw);
		self.last_value_raw = raw;
	}

	fn cook_raw_read_value(&self, raw: u16) -> f32 {
		if raw < self.read_low { 0. }
		else if raw > self.read_high { 1. }
		else {(raw - self.read_low) as f32 / ((self.read_high - self.read_low) as f32)}
	}

	pub fn value(&self) -> f32 {
		if let Some(target) = self.target_value {
			target
		}
		else {
			self.last_value
		}
	}
	pub fn live_value(&self) -> f32 {
		self.last_value
	}
	pub fn set_target(&mut self, target: f32) {
		const TIMEOUT: u32 = 500;
		match self.calibration_phase {
			FaderCalibrationPhase::NotCalibrating => {
				self.target_value = Some(target.clamp(0., 1.));
				self.time_left = TIMEOUT;
			}
			_ => {}
		}
	}
	pub fn target(&self) -> Option<f32> {
		self.target_value
	}
	pub fn process(&mut self) -> Option<u16> {
		use FaderCalibrationPhase::*;
		match self.calibration_phase {
			NotCalibrating => self.process_normal(),
			_ => self.process_calibration()
		}
	}

	fn process_calibration(&mut self) -> Option<u16> {
		use FaderCalibrationPhase::*;
		if self.time_left > 0 {
			self.time_left -= 1;
		}
		match self.calibration_phase {
			Init => {
				self.calibration_phase = ApproachMin;
				self.time_left = 300;
				Some(0)
			}
			ApproachMin => {
				if self.time_left == 0 {
					self.read_low = self.last_value_raw;
					self.calibration_phase = ApproachMidFromBelow;
					self.time_left = 250;
				}
				Some(0)
			}
			ApproachMidFromBelow => {
				if self.time_left == 0 {
					self.mid_from_below = self.last_value_raw;
					self.calibration_phase = ApproachMax;
					self.time_left = 250;
				}
				Some(WRITE_MID_APPROX)
			}
			ApproachMax => {
				if self.time_left == 0 {
					self.read_high = self.last_value_raw;
					self.calibration_phase = ApproachMidFromAbove;
					self.time_left = 250;
				}
				Some(4095)
			}
			ApproachMidFromAbove => {
				if self.time_left == 0 {
					self.mid_from_above = self.last_value_raw;
					self.calibration_phase = Approach25FromAbove;
					self.time_left = 250;
				}
				Some(WRITE_MID_APPROX)
			}
			Approach25FromAbove => {
				if self.time_left == 0 {
					self.v25_from_above = self.last_value_raw;
					self.calibration_phase = Approach75FromBelow;
					self.time_left = 250;
				}
				Some(WRITE_25_APPROX)
			}
			Approach75FromBelow => {
				if self.time_left == 0 {
					let v75_from_below = self.last_value_raw;

					let read_deadzone = if self.mid_from_above >= self.mid_from_below { (self.mid_from_above - self.mid_from_below) / 2 } else { 0 };
					let v75_cooked = self.cook_raw_read_value(v75_from_below + read_deadzone);
					let v25_cooked = self.cook_raw_read_value(self.v25_from_above - read_deadzone);

					let gain = (WRITE_75_APPROX - WRITE_25_APPROX) as f32 / (v75_cooked - v25_cooked);
					assert!(0. < gain);
					self.write_low = WRITE_25_APPROX - ((v25_cooked * gain) as u16);
					self.write_high = self.write_low + (gain as u16);
					self.write_deadzone = ((self.cook_raw_read_value(self.mid_from_above) - self.cook_raw_read_value(self.mid_from_below)) * gain) as u16;


					self.calibration_phase = NotCalibrating;
				}
				Some(WRITE_75_APPROX)
			}
			_ => { None }
		}
		
	}

	fn process_normal(&mut self) -> Option<u16> {
		if let Some(target) = self.target_value {
			if self.last_value - target < 0.02 {
				self.time_left = self.time_left.min(200);
			}
			if self.time_left == 0 {
				self.target_value = None;
				return None;
			}
			else {
				self.time_left -= 1;
				let target_raw = (target * (self.write_high - self.write_low) as f32 + self.write_low as f32) as u16;

				let error = target - self.last_value;
				let limit = (1.0 - error.abs()/0.1).clamp(0.,1.);
				self.integrator = (self.integrator + 0.2*error).clamp(-limit, limit);

				if error > 0. {
					self.integrator = self.integrator.max(0.0);
				}
				if error < 0. {
					self.integrator = self.integrator.min(0.0);
				}

				return Some((target_raw as i32 + ((self.integrator * self.write_deadzone as f32) as i32)).clamp(0,4095) as u16);
			}
		}
		else {
			return None;
		}
	}
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
	struct Resources {
		mytimer: timer::CountDownTimer<TIM2>,
		led: PC13<Output<PushPull>>,
		x3423: X3423,
		faders: [Fader; 17],
		fader_steps: [u8; 17],
		usb_dev: UsbDevice<'static, UsbBusType>,
		midi: usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>,
		adc: Adc<ADC1>,
		dac: Mcp49xx<SpiInterface<spi::Spi<stm32f1xx_hal::pac::SPI2, spi::Spi2NoRemap, (PB13<Alternate<PushPull>>, spi::NoMiso, PB15<Alternate<PushPull>>), u8>, OldOutputPin<PC15<Output<PushPull>>>>, Resolution12Bit, DualChannel, Unbuffered>,
		delay: stm32f1xx_hal::delay::Delay,

		values: [f32; 17],
		target_values: [Option<f32>; 17],
	}

	#[init]
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
		//core.NVIC.enable(stm32f1xx_hal::pac::Interrupt::USB_LP_CAN_RX0);

		// Configure SPI
		let mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
		let sck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
		let dac_cs: OldOutputPin<_> = gpioc.pc15.into_push_pull_output_with_state(&mut gpioc.crh, State::High).into();

		let spi = spi::Spi::spi2(device.SPI2, (sck, spi::NoMiso, mosi), spi::Mode { phase : spi::Phase::CaptureOnFirstTransition, polarity : spi::Polarity::IdleLow }, 5.mhz(), clocks, &mut rcc.apb1);
		let mut dac = mcp49xx::Mcp49xx::new_mcp4822(spi, dac_cs);

		let mut adc = stm32f1xx_hal::adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);

		let mut mytimer =
			timer::Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1)
			.start_count_down(Hertz(1000));
		mytimer.listen(timer::Event::Update);

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
			delay,
			faders: Default::default(),
			values: [0.5; 17],
			target_values: [None; 17],
			mytimer,
			fader_steps: [0; 17],
		};

		cortex_m::asm::delay(5);
		resources.x3423.reset.set_high();

		resources
	}

	
	#[task(binds = TIM2, resources = [x3423, adc, dac, delay, faders, values, target_values, mytimer, led, midi, fader_steps], priority=1)]
	fn xmain(mut c : xmain::Context) {
		static mut blink: u64 = 0;
		static mut last_value: [u16; 17] = [0x42*128; 17];
		c.resources.mytimer.clear_update_interrupt_flag();
		
		*blink += 1;
		if (*blink) % 100 == 0 {
			c.resources.led.toggle();
		}

		for i in 0..4 {
			c.resources.x3423.select_analog_value(i as u8, c.resources.delay);
			c.resources.faders[0 + i].update_value( c.resources.adc.read(&mut c.resources.x3423.mfpos0).unwrap() );
			c.resources.faders[4 + i].update_value( c.resources.adc.read(&mut c.resources.x3423.mfpos1).unwrap() );
			c.resources.faders[8 + i].update_value( c.resources.adc.read(&mut c.resources.x3423.mfpos2).unwrap() );
			c.resources.faders[12 + i].update_value( c.resources.adc.read(&mut c.resources.x3423.mfpos3).unwrap() );
		}
		c.resources.faders[16].update_value( c.resources.adc.read(&mut c.resources.x3423.mfpos4).unwrap() );

		let mut fader_steps = c.resources.fader_steps.lock(|s| *s);
		for i in 0..17 { fader_steps[i] = i as u8; }
		fader_steps[3] = 3;
		for (fader, steps) in c.resources.faders.iter_mut().zip(fader_steps.iter()) {
			if *steps > 1 {
				let steps_f32 = *steps as f32;
				let quantized_value = (fader.live_value() * (steps_f32 - 1.)).round() / (steps_f32 - 1.);
				let diff = fader.live_value() - quantized_value;

				if diff.abs() >= 0.02 {
					fader.set_target(quantized_value);
				}
			}
		}




		let mut fader_values = [0.0; 17];
		let mut active_faders = 0;
		let mut n_active_faders = 0;

		let mut fnord : [Option<u16>; 17] = [None; 17];
		for i in 0..17 {
			fader_values[i] = c.resources.faders[i].value();

			fnord[i] = c.resources.faders[i].process();
			if let Some(target_value) = fnord[i] {
				active_faders |= 1<<i;
				n_active_faders += 1;

				if n_active_faders >= 4 {
					break;
				}
			}
		}
		c.resources.x3423.set_motor_enable(active_faders);
		for i in 0..17 {
			if let Some(target_value) = fnord[i] {
				c.resources.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value(target_value));
				c.resources.delay.delay_us(5_u16);
				c.resources.x3423.capture_analog_value(i as u8, c.resources.delay);
			}
		}



		c.resources.midi.lock(|midi| {
			for i in 0..17 {
				assert!(fader_values[i] >= 0.0 && fader_values[i] <= 1.0);
				let val = (fader_values[i] * 16383.0) as u16;
				if last_value[i] / 128 != val / 128 {
					if midi.send_bytes([0x0B, 0xB0, i as u8 + 1, (val / 128) as u8]).is_ok() {
						last_value[i] = val;
					}
				}
			}
		});



		let target_values = c.resources.target_values.lock(|target_values| {
			let tmp = *target_values;
			*target_values = [None; 17];
			tmp
		});
		
		for i in 0..17 {
			if let Some(val) = target_values[i] {
				c.resources.faders[i].set_target(val);
			}
		}
	}

	#[task(binds = USB_LP_CAN_RX0, resources=[midi, usb_dev, values, target_values, fader_steps], priority=2)]
	fn periodic_usb_poll(mut c : periodic_usb_poll::Context) {

		c.resources.usb_dev.poll(&mut[c.resources.midi]);

		let mut message: [u8; 4] = [0; 4];
		while let Ok(len) = c.resources.midi.read(&mut message) {
			if len == 4 {
				let cable = (message[0] & 0xF0) >> 4;
				let messagetype = message[0] & 0x0F;

				match messagetype {
					0xB => {
						let channel = message[1] & 0x0F;
						let cc = message[2];
						let value = message[3];
						if (1..=17).contains(&cc) {
							c.resources.target_values[(cc-1) as usize] = Some(value as f32 / 128.0);
						}
					}
					_ => {}
				}
			}
		}

		
	}
	
	extern "C" {
		fn EXTI0();
		fn EXTI1();
	}
};
