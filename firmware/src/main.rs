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


const BASE_CURRENT: f32 = 0.3; // Ampere
const FADER_CURRENT: f32 = 0.25; // Ampere. Note: this is only for slow movement and stall. Large jumps draw more current!
const CURRENT_LIMIT: f32 = 3.33; // Ampere.
const MAX_ACTIVE_FADERS: u16 = ((CURRENT_LIMIT - BASE_CURRENT) / FADER_CURRENT) as u16;
const MAX_ACTIVE_FADERS_DURING_CALIBRATION: u16 = 3;


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

#[derive(Clone, Copy, PartialEq)]
pub enum FaderState {
	Idle,
	UserControlled,
	UserOverride,
	MidiControlledSlow,
	MidiControlledJump
}

#[derive(Clone, Copy)]
pub struct Queue {
	history: [u16; 64],
	pointer: usize,
	counter: u16
}

impl Queue {
	const SPARSITY: u16 = 24;
	pub const fn new() -> Queue { Queue { history: [0; 64], pointer: 0, counter: 0 } }

	fn u16_to_f32(value: u16) -> f32 {
		value as f32 / 65535.0
	}

	pub fn push(&mut self, value: f32) {
		if self.counter == 0 {
			self.history[self.pointer] = (value * 65535.0) as u16;
			self.pointer = (self.pointer + 1) % self.history.len();
		}
		self.counter = (self.counter + 1) % Self::SPARSITY;
	}
	pub fn delta(&self) -> f32 {
		Self::u16_to_f32(*self.history.iter().max().unwrap()) - Self::u16_to_f32(*self.history.iter().min().unwrap())
	}

	pub fn trend(&self) -> f32 {
		let most_recent_pointer = (self.pointer + self.history.len() - 1) % self.history.len();
		return Self::u16_to_f32(self.history[most_recent_pointer]) - Self::u16_to_f32(self.history[self.pointer]);
	}
}

#[derive(Copy,Clone)]
pub struct FaderStateMachine {
	state: FaderState,
	old_setpoint: f32,
	stable_timer: u32,
	measured_history: Queue,
	setpoint_history: Queue,
	state_inertia_counter: u32
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
			setpoint_history: Queue::new(),
			state_inertia_counter: 0
		}
	}
	pub fn process(&mut self, setpoint: f32, measured: f32) -> FaderResult {
		let JUMP_THRESHOLD = 3.0;
		let SET_DEADZONE = 0.01;
		let SETPOINT_DEADZONE = 0.001;
		let ESCAPE_LIMIT = 0.2;
		let INPUT_DEADZONE = 0.05;
		let CAPTURE_ZONE = 0.06;
		let STABILIZE_TIMEOUT = 500;
		let STATE_INERTIA = 300;

		self.measured_history.push(measured);
		self.setpoint_history.push(setpoint);

		if (setpoint - measured).abs() < CAPTURE_ZONE {
			self.stable_timer = self.stable_timer.saturating_add(1);
		}
		else {
			self.stable_timer = 0;
		}

		let setpoint_diff = (self.old_setpoint - setpoint).abs();
		if setpoint_diff > SETPOINT_DEADZONE {
			self.stable_timer = 0;
			self.old_setpoint = setpoint;
		}
		//let old_state = self.state;
		let result = match self.state {
			FaderState::Idle => {
				if setpoint_diff >= JUMP_THRESHOLD {
					self.state = FaderState::MidiControlledJump;
				}
				else if setpoint_diff > SETPOINT_DEADZONE {
					self.state = FaderState::MidiControlledSlow;
				}
			
				if (setpoint - measured).abs() >= ESCAPE_LIMIT {
					self.state = FaderState::UserOverride;
					self.measured_history.push(1.0);
					self.measured_history.push(0.0);
				}
				else if (setpoint - measured).abs() >= INPUT_DEADZONE {
					self.state = FaderState::UserControlled;
					self.measured_history.push(1.0);
					self.measured_history.push(0.0);
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
					self.measured_history.push(1.0);
					self.measured_history.push(0.0);
				}
				else if (setpoint - measured).abs() >= 3.*INPUT_DEADZONE {
					self.state = FaderState::UserControlled;
					self.measured_history.push(1.0);
					self.measured_history.push(0.0);
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
				if self.measured_history.delta() < INPUT_DEADZONE {
					if (setpoint - measured).abs() < SET_DEADZONE {
						self.state = FaderState::Idle;
					}
					else if self.setpoint_history.trend().signum() != expected_trend_signum || self.setpoint_history.delta() < SET_DEADZONE {
						self.state = FaderState::MidiControlledJump;
					}
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
				if self.measured_history.delta() < INPUT_DEADZONE {
					if (setpoint - measured).abs() < SET_DEADZONE {
						self.state = FaderState::Idle;
					}
					else if self.setpoint_history.trend().signum() != expected_trend_signum || self.setpoint_history.delta() < SET_DEADZONE {
						self.state = FaderState::MidiControlledJump;
					}
				}

				FaderResult {
					fader_move_target: None,
					midi_send_value: Some(measured)
				}
			}
		};
		/*if self.state != old_state {
			if self.state_inertia_counter >= STATE_INERTIA {
				self.state_inertia_counter = 0;
			}
			else {
				self.state = old_state;
				self.state_inertia_counter += 1;
			}
		}*/
		return result;
	}
}

#[derive(Copy,Clone)]
pub struct SlewTarget {
	target_value: f32,
	current_value: f32,
}

impl SlewTarget {
	pub fn new() -> SlewTarget { SlewTarget { target_value: 0.5, current_value: 0.0 } }
	pub fn set_target(&mut self, value: f32) {
		self.target_value = value;
	}
	pub fn slew_value(&mut self) -> f32 {
		if self.current_value > self.target_value {
			self.current_value = (self.current_value - 0.002).clamp(self.target_value, 1.0);
		}
		else if self.current_value < self.target_value {
			self.current_value = (self.current_value + 0.002).clamp(0.0, self.target_value);
		}
		return self.current_value;
	}
}

pub struct MidiSender {
	last_values: [f32; 17]
}
impl MidiSender {
	pub fn new() -> MidiSender { MidiSender { last_values: [-1.0; 17] } }
	pub fn send_midi_cc(&mut self, idx: usize, value: f32, midi: &mut usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>) {
		let cc_number = idx as u8 + 1;
		if (self.last_values[idx] - value).abs() > 0.005 {
			let val = (value * 16383.0) as u16;
			let lsb = (val % 128) as u8;
			let msb = (val / 128) as u8;
			
			if midi.send_bytes(&[0x0B, 0xB0, cc_number + 32, lsb, 0x0B, 0xB0, cc_number, msb]).is_ok() {
				self.last_values[idx] = value;
			}
		}
	}
}



#[derive(Clone, Copy)]
pub enum FaderConfig {
	Continuous,
	Steps(u8),
	Bidirectional,
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
		fader_config: [FaderConfig; 17],
		fader_midi_commands: [Option<f32>; 17],
		fader_processes_midi_command: [bool; 17],
		calibration_request: Calib,

		flash: stm32f1xx_hal::flash::Parts,

		midi_sender: MidiSender,
		fader_bidir: [FaderStateMachine; 17],
		fader_bidir_target: [SlewTarget; 17]
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
			fader_midi_commands: [None; 17],
			mytimer,
			fader_config: [FaderConfig::Bidirectional; 17],
			fader_processes_midi_command: [false; 17],
			flash,
			calibration_request: Calib::Idle,
			midi_sender: MidiSender::new(),
			fader_bidir: [FaderStateMachine::new(); 17],
			fader_bidir_target: [SlewTarget::new(); 17]
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

	
	#[task(binds = TIM2, resources = [x3423, dac, delay, faders, fader_midi_commands, mytimer, led, midi, fader_config, fader_processes_midi_command, calibration_request, flash, midi_sender, fader_bidir, fader_bidir_target], priority=1)]
	fn xmain(mut c : xmain::Context) {
		static mut BLINK: u64 = 0;

		let res = &mut c.resources;
		res.mytimer.clear_update_interrupt_flag();
		
		*BLINK += 1;
		if (*BLINK) % 100 == 0 {
			res.led.toggle().unwrap();
		}

		let calibration_state = res.calibration_request.lock(|c|*c);

		let fader_midi_commands = res.fader_midi_commands.lock(|fader_midi_commands| {
			let tmp = *fader_midi_commands;
			*fader_midi_commands = [None; 17];
			tmp
		});

		// read raw fader values
		let faders = &mut res.faders;
		res.x3423.read_values(|idx, value| { faders[idx].update_value(value) }, res.delay);

		// do all the fader processing
		if calibration_state == Calib::Idle {
			let faders = &mut res.faders;
			let fader_config = res.fader_config.lock(|s| *s);
			for i in 0..17 {
				match fader_config[i] {
					FaderConfig::Steps(steps) => {
						if !res.fader_processes_midi_command[i] {
							let steps_f32 = steps as f32;
							let quantized_value = (faders[i].live_value() * (steps_f32 - 1.)).round() / (steps_f32 - 1.);
							let diff = faders[i].live_value() - quantized_value;

							if diff.abs() >= 0.02 {
								faders[i].set_target(quantized_value);
							}
						}
						
						let midi_sender = &mut res.midi_sender;
						res.midi.lock(|midi| midi_sender.send_midi_cc(i, faders[i].value(), midi));

						if let Some(target) = fader_midi_commands[i] {
							faders[i].set_target(target);
							res.fader_processes_midi_command[i] = true;
						}
					}
					FaderConfig::Bidirectional => {
						if let Some(value) = fader_midi_commands[i] {
							res.fader_bidir_target[i].set_target(value);
						}

						let result = res.fader_bidir[i].process(res.fader_bidir_target[i].slew_value(), faders[i].live_value());

						if let Some(value) = result.midi_send_value {
							let midi_sender = &mut res.midi_sender;
							res.midi.lock(|midi| midi_sender.send_midi_cc(i, value, midi));
						}

						if let Some(target) = result.fader_move_target {
							faders[i].set_target(target);
						}
						else {
							faders[i].clear_target();
						}
					}
					FaderConfig::Continuous => {
						let midi_sender = &mut res.midi_sender;
						res.midi.lock(|midi| midi_sender.send_midi_cc(i, faders[i].value(), midi));
						
						if let Some(target) = fader_midi_commands[i] {
							faders[i].set_target(target);
						}
					}
				}
			}
		}


		// set fader values, handle movement and calibration
		let mut active_faders = 0;
		let mut n_active_faders = 0;
		let fader_limit = match res.calibration_request.lock(|c|*c) {
			Calib::Idle => MAX_ACTIVE_FADERS,
			_ => MAX_ACTIVE_FADERS_DURING_CALIBRATION
		};
		
		for (i, (fader, processes_midi)) in
			res.faders.iter_mut()
			.zip(res.fader_processes_midi_command.iter_mut())
			.enumerate()
		{
			// set values received via MIDI and process fader movement / calibration
			if n_active_faders < fader_limit
			{
				if let Some(target_value) = fader.process() {
					active_faders |= 1 << i;
					n_active_faders += 1;

					res.dac.send(mcp49xx::Command::default().double_gain().channel(mcp49xx::Channel::Ch0).value(target_value)).unwrap();
					res.delay.delay_us(5_u16);
					res.x3423.capture_analog_value(i as u8, res.delay);
				}
			}
			
			if fader.target().is_none() {
				*processes_midi = false;
			}
		}
		res.x3423.set_motor_enable(active_faders);


		// handle pending calibration request or write finished calibration to flash
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

	#[task(binds = USB_LP_CAN_RX0, resources=[midi, usb_dev, fader_midi_commands, fader_config, calibration_request], priority=2)]
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
								c.resources.fader_midi_commands[(cc-1) as usize] = Some(value as f32 / 128.0);
							}

							if (71..87).contains(&cc) {
								c.resources.fader_config[(cc-71) as usize] =
									match value {
										0 => FaderConfig::Continuous,
										127 => FaderConfig::Bidirectional,
										steps => FaderConfig::Steps(steps)
									};
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
