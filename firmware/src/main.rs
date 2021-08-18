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

#[derive(Clone, Copy, PartialEq)]
pub enum Calib
{
	Pending,
	Running,
	Idle
}

pub enum FaderState {
	Idle,
	UserControlled,
	UserOverride,
	MidiControlledSlow,
	MidiControlledJump
}

pub struct Queue {
	history: [f32; 128],
	pointer: usize
}

impl Queue {
	pub const fn new() -> Queue { Queue { history: [0.0; 128], pointer: 0 } }
	pub fn push(&mut self, value: f32) {
		self.history[self.pointer] = value;
		self.pointer = (self.pointer + 1) % self.history.len();
	}
	pub fn delta(&self) -> f32 {
		self.history.iter().max_by(|a,b| a.partial_cmp(b).unwrap()).unwrap() - self.history.iter().min_by(|a,b| a.partial_cmp(b).unwrap()).unwrap()
	}

	pub fn trend(&self) -> f32 {
		let prev_pointer = (self.pointer + self.history.len() - 1) % self.history.len();
		return self.history[self.pointer] - self.history[prev_pointer];
	}
}

pub struct FaderStateMachine {
	state: FaderState,
	old_setpoint: f32,
	stable_timer: u32,
	measured_history: Queue,
	setpoint_history: Queue,
}

pub struct FaderResult {
	pub fader_move_target: Option<f32>,
	pub midi_send_value: Option<f32>
}

trait Signum {
	fn signum(self) -> Self;
}

impl Signum for f32 {
	fn signum(self) -> f32 {
		if self > 0.0 { return 1.0; }
		else if self < 0.0 { return -1.0; }
		else { return 0.0; }
	}
}

impl FaderStateMachine {
	pub const fn new() -> FaderStateMachine {
		FaderStateMachine {
			state: FaderState::Idle,
			old_setpoint: 0.5,
			stable_timer: 0,
			measured_history: Queue::new(),
			setpoint_history: Queue::new()
		}
	}
	pub fn process(&mut self, setpoint: f32, measured: f32) -> FaderResult {
		let JUMP_THRESHOLD = 1.0;
		let SET_DEADZONE = 0.01;
		let ESCAPE_LIMIT = 5.0;
		let INPUT_DEADZONE = 0.02;
		let CAPTURE_ZONE = 0.03;
		let STABILIZE_TIMEOUT = 500;

		self.measured_history.push(measured);
		self.setpoint_history.push(setpoint);

		if (setpoint - measured).abs() < SET_DEADZONE {
			self.stable_timer = self.stable_timer.saturating_add(1);
		}
		else {
			self.stable_timer = 0;
		}

		let setpoint_diff = (self.old_setpoint - setpoint).abs();
		if setpoint_diff >= SET_DEADZONE {
			self.old_setpoint = setpoint;
		}

		match self.state {
			FaderState::Idle => {
				if setpoint_diff >= JUMP_THRESHOLD {
					self.state = FaderState::MidiControlledJump;
				}
				else if setpoint_diff >= SET_DEADZONE {
					self.state = FaderState::MidiControlledSlow;
				}
			
				if (setpoint - measured).abs() >= ESCAPE_LIMIT {
					self.state = FaderState::UserOverride;
				}
				else if (setpoint - measured).abs() >= INPUT_DEADZONE {
					self.state = FaderState::UserControlled;
				}
				
				FaderResult {
					fader_move_target: None,
					midi_send_value: None
				}
			}
			FaderState::MidiControlledSlow => {
				if setpoint_diff >= JUMP_THRESHOLD {
					self.state = FaderState::MidiControlledJump;
				}
				if self.stable_timer >= STABILIZE_TIMEOUT {
					self.state = FaderState::Idle;
				}
				if (setpoint - measured).abs() >= ESCAPE_LIMIT {
					self.state = FaderState::UserOverride;
				}
				else if (setpoint - measured).abs() >= INPUT_DEADZONE {
					self.state = FaderState::UserControlled;
				}

				FaderResult {
					fader_move_target: Some(setpoint),
					midi_send_value: None
				}
			}
			FaderState::MidiControlledJump => {
				if self.stable_timer >= STABILIZE_TIMEOUT {
					self.state = FaderState::Idle;
				}

				FaderResult {
					fader_move_target: Some(setpoint),
					midi_send_value: None
				}
			}
			FaderState::UserControlled => {
				let expected_trend_signum = (measured - setpoint).signum();
				if self.measured_history.delta() < INPUT_DEADZONE && (self.setpoint_history.trend().signum() != expected_trend_signum || self.setpoint_history.delta() < SET_DEADZONE || (setpoint - measured).abs() < SET_DEADZONE) {
					self.state = FaderState::Idle;
				}
				if (setpoint - measured).abs() >= ESCAPE_LIMIT {
					self.state = FaderState::UserOverride;
				}

				let fader_move_target =
					if (setpoint - measured).abs() < CAPTURE_ZONE {
						None
					}
					else {
						let limit = setpoint + CAPTURE_ZONE * (measured - setpoint).signum();
						Some( limit.clamp(0.0, 1.0) )
					};

				FaderResult {
					fader_move_target,
					midi_send_value: Some(measured)
				}
			}
			FaderState::UserOverride => {
				let expected_trend_signum = (measured - setpoint).signum();
				if self.measured_history.delta() < INPUT_DEADZONE && (self.setpoint_history.trend().signum() != expected_trend_signum || self.setpoint_history.delta() < SET_DEADZONE || (setpoint - measured).abs() < SET_DEADZONE) {
					self.state = FaderState::Idle;
				}

				FaderResult {
					fader_move_target: None,
					midi_send_value: Some(measured)
				}
			}
		}
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
		calibration_request: Calib,

		flash: stm32f1xx_hal::flash::Parts,
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
			flash,
			calibration_request: Calib::Idle,
		};

		
		let writer = resources.flash.writer(stm32f1xx_hal::flash::SectorSize::Sz2K, stm32f1xx_hal::flash::FlashSize::Sz128K);
		for i in 0..17 {
			let params = writer.read(65536-2048 + 10*i, 10).unwrap();
			if params[0] != 0xFF && params[1] != 0xFF {
				resources.faders[i as usize].set_calibration_data(params);
			}
		}


		resources.x3423.reset();

		resources
	}

	
	#[task(binds = TIM2, resources = [x3423, dac, delay, faders, target_values, mytimer, led, midi, fader_steps, fader_processes_user_input, calibration_request, flash], priority=1)]
	fn xmain(mut c : xmain::Context) {
		static mut BLINK: u64 = 0;
		static mut LAST_VALUE: [u16; 17] = [0x42*128; 17];

		static mut BLAH: FaderStateMachine = FaderStateMachine::new();
		static mut BLUBB: FaderStateMachine = FaderStateMachine::new();
		static mut BLUBB_SOLL: f32 = 0.5;

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
		let fader_steps = res.fader_steps.lock(|s| *s);
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
		let mut target_values = res.target_values.lock(|target_values| {
			let tmp = *target_values;
			*target_values = [None; 17];
			tmp
		});


		let result = BLAH.process(res.faders[1].value(), res.faders[2].value());
		target_values[2] = result.fader_move_target;

		let result2 = BLUBB.process(*BLUBB_SOLL, res.faders[3].value());
		if let Some(val) = result2.midi_send_value {
			*BLUBB_SOLL += 0.0003 * (val - *BLUBB_SOLL).signum();
			*BLUBB_SOLL = BLUBB_SOLL.clamp(0.0,1.0);
		}
		target_values[3] = result2.fader_move_target;
		target_values[4] = Some(*BLUBB_SOLL);



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
			else {
				fader.clear_target(); // FIXME
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

		match res.calibration_request.lock(|c| *c)
		{
			Calib::Running => {
				if !res.faders.iter().any(|fader| fader.is_calibrating()) {
					// save to flash
					let mut data: [u8; 170] = [0; 170];
					for i in 0..17 {
						let calib = res.faders[i].get_calibration_data();
						for j in 0..10 {
							data[i*10 + j] = calib[j];
						}
					}

					let mut writer = res.flash.writer(stm32f1xx_hal::flash::SectorSize::Sz2K, stm32f1xx_hal::flash::FlashSize::Sz128K);
					writer.change_verification(false);
					writer.erase(65536-2048, 2*1024).unwrap();
					writer.write(65536-2048, &data).unwrap();

					res.calibration_request.lock(|c| *c = Calib::Idle);
				}
			}
			Calib::Pending => {
				for fader in res.faders.iter_mut() {
					fader.start_calibration();
				}
				res.calibration_request.lock(|c| *c = Calib::Running);
			}
			Calib::Idle => {}
		}
	}

	#[task(binds = USB_LP_CAN_RX0, resources=[midi, usb_dev, target_values, fader_steps, calibration_request], priority=2)]
	fn periodic_usb_poll(c : periodic_usb_poll::Context) {

		c.resources.usb_dev.poll(&mut[c.resources.midi]);

		let mut buffer = [0u8; 128];
		while let Ok(len) = c.resources.midi.read(&mut buffer) {
			for message in buffer[0..len].chunks(4) {
				if message.len() == 4 {
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

							if (71..87).contains(&cc) {
								c.resources.fader_steps[(cc-71) as usize] = value;
							}

							if cc == 100 {
								*c.resources.calibration_request = Calib::Pending;
							}
						}
						_ => {}
					}
				}
			}
		}

		
	}
	
	extern "C" {
		fn EXTI0();
		fn EXTI1();
	}
};
