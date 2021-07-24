#![no_main]
#![no_std]

mod x3423;
mod fader;
use x3423::X3423;
use fader::Fader;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v1_compat::OldOutputPin;
use rtic::app;
use stm32f1xx_hal::gpio::{gpiob::*, gpioc::*, Input, Floating, Alternate, Output, PushPull, State};
use stm32f1xx_hal::spi;
use stm32f1xx_hal::timer;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::usb::{UsbBus, Peripheral, UsbBusType};
use stm32f1xx_hal::pac::TIM2;
use mcp49xx::Mcp49xx;
use mcp49xx::interface::SpiInterface;
use mcp49xx::marker::Resolution12Bit;
use mcp49xx::marker::DualChannel;
use mcp49xx::marker::Unbuffered;
use stm32f1xx_hal::time::Hertz;

use usb_device::prelude::*;

use micromath::F32Ext;

const MAX_ACTIVE_FADERS: u16 = 17;


#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
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




#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
	struct Resources {
		delay: stm32f1xx_hal::delay::Delay,
		mytimer: timer::CountDownTimer<TIM2>,

		led: PC13<Output<PushPull>>,

		usb_dev: UsbDevice<'static, UsbBusType>,
		midi: usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>,
		dac: Mcp49xx<SpiInterface<spi::Spi<stm32f1xx_hal::pac::SPI2, spi::Spi2NoRemap, (PB13<Alternate<PushPull>>, spi::NoMiso, PB15<Alternate<PushPull>>), u8>, OldOutputPin<PC15<Output<PushPull>>>>, Resolution12Bit, DualChannel, Unbuffered>,

		x3423: X3423,

		faders: [Fader; 17],
		fader_steps: [u8; 17],
		target_values: [Option<f32>; 17],
		fader_processes_user_input: [bool; 17],
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

		let delay = Delay::new(core.SYST, clocks);

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

		// Configure SPI
		let mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
		let sck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
		let dac_cs: OldOutputPin<_> = gpioc.pc15.into_push_pull_output_with_state(&mut gpioc.crh, State::High).into();

		let spi = spi::Spi::spi2(device.SPI2, (sck, spi::NoMiso, mosi), spi::Mode { phase : spi::Phase::CaptureOnFirstTransition, polarity : spi::Polarity::IdleLow }, 5.mhz(), clocks, &mut rcc.apb1);
		let dac = mcp49xx::Mcp49xx::new_mcp4822(spi, dac_cs);

		let adc = stm32f1xx_hal::adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);

		let mut mytimer =
			timer::Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1)
			.start_count_down(Hertz(1000));
		mytimer.listen(timer::Event::Update);

		let mut resources = init::LateResources {
			led,
			x3423: X3423::new(x3423::X3423Resources {
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
				data: x3423::X3423DataPins {
					d0: pb4.into_open_drain_output(&mut gpiob.crl),
					d1: gpiob.pb6.into_open_drain_output(&mut gpiob.crl),
					d2: gpiob.pb7.into_open_drain_output(&mut gpiob.crl),
					d3: gpiob.pb8.into_open_drain_output(&mut gpiob.crh),
					d4: gpiob.pb9.into_open_drain_output(&mut gpiob.crh),
					d5: gpiob.pb10.into_open_drain_output(&mut gpiob.crh),
					d6: gpiob.pb11.into_open_drain_output(&mut gpiob.crh),
					d7: gpiob.pb12.into_open_drain_output(&mut gpiob.crh),
				},
				adc,
			}),
			usb_dev,
			midi,
			dac,
			delay,
			faders: Default::default(),
			target_values: [None; 17],
			mytimer,
			fader_steps: [0; 17],
			fader_processes_user_input: [false; 17],
		};

		resources.x3423.reset();

		resources
	}

	
	#[task(binds = TIM2, resources = [x3423, dac, delay, faders, target_values, mytimer, led, midi, fader_steps, fader_processes_user_input], priority=1)]
	fn xmain(mut c : xmain::Context) {
		static mut BLINK: u64 = 0;
		static mut LAST_VALUE: [u16; 17] = [0x42*128; 17];

		let res = &mut c.resources;
		res.mytimer.clear_update_interrupt_flag();
		
		*BLINK += 1;
		if (*BLINK) % 100 == 0 {
			res.led.toggle().unwrap();
		}

		// read raw fader values
		let faders = &mut res.faders;
		res.x3423.read_values(|idx, value| { faders[idx].update_value(value) }, res.delay);

		// handle steppiness
		let mut fader_steps = res.fader_steps.lock(|s| *s);
		for i in 0..17 { fader_steps[i] = i as u8; } // FIXME DEBUG
		for ((fader, steps), user_flag) in res.faders.iter_mut().zip(fader_steps.iter()).zip(res.fader_processes_user_input.iter()) {
			if *steps > 1 && !*user_flag {
				let steps_f32 = *steps as f32;
				let quantized_value = (fader.live_value() * (steps_f32 - 1.)).round() / (steps_f32 - 1.);
				let diff = fader.live_value() - quantized_value;

				if diff.abs() >= 0.02 {
					fader.set_target(quantized_value);
				}
			}
		}

		// send current values via MIDI
		let midi = &mut res.midi;
		for (i, (fader, last)) in res.faders.iter_mut().zip(LAST_VALUE.iter_mut()).enumerate()
		{
			let val = (fader.value() * 16383.0) as u16;
			if *last / 128 != val / 128 {
				midi.lock(|midi| {
					if midi.send_bytes([0x0B, 0xB0, i as u8 + 1, (val / 128) as u8]).is_ok() {
						*last = val;
					}
				});
			}
		}


		// set fader values, handle movement and calibration
		let target_values = res.target_values.lock(|target_values| {
			let tmp = *target_values;
			*target_values = [None; 17];
			tmp
		});
		let mut active_faders = 0;
		let mut n_active_faders = 0;
		
		for (i, ((target, fader), user_flag)) in
			target_values.iter()
			.zip(res.faders.iter_mut())
			.zip(res.fader_processes_user_input.iter_mut())
			.enumerate()
		{
			if let Some(val) = *target {
				fader.set_target(val);
				*user_flag = true;
			}

			// set values received via MIDI and process fader movement / calibration
			if let Some(target_value) = fader.process() {
				if n_active_faders <= MAX_ACTIVE_FADERS
				{
					active_faders |= 1 << i;
					n_active_faders += 1;

					res.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value(target_value)).unwrap();
					res.delay.delay_us(5_u16);
					res.x3423.capture_analog_value(i as u8, res.delay);
				}
			}
			
			if fader.target().is_none() {
				*user_flag = false;
			}
		}
		res.x3423.set_motor_enable(active_faders);
	}

	#[task(binds = USB_LP_CAN_RX0, resources=[midi, usb_dev, target_values, fader_steps], priority=2)]
	fn periodic_usb_poll(c : periodic_usb_poll::Context) {

		c.resources.usb_dev.poll(&mut[c.resources.midi]);

		let mut message: [u8; 4] = [0; 4];
		while let Ok(len) = c.resources.midi.read(&mut message) {
			if len == 4 {
				//let cable = (message[0] & 0xF0) >> 4;
				let messagetype = message[0] & 0x0F;

				match messagetype {
					0xB => {
						//let channel = message[1] & 0x0F;
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
