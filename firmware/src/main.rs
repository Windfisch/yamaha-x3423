#![no_main]
#![no_std]

use core::convert::Infallible;

mod x3423;
mod fader;
use x3423::X3423;
use fader::Fader;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v1_compat::OldOutputPin;
use rtic::app;
use stm32f1xx_hal::gpio::{gpioa::*, gpiob::*, gpioc::*, Input, Floating, Alternate, Output, PushPull, State};
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

use shared_bus_rtic::SharedBus;

use usb_device::prelude::*;

use micromath::F32Ext;

pub struct RefMutOutputPin<'a, T: OutputPin + ?Sized>(pub &'a mut T);

impl<'a, T: OutputPin + ?Sized> OutputPin for RefMutOutputPin<'a, T> {
	type Error = T::Error;
	fn set_low(&mut self) -> Result<(), Self::Error> {
		self.0.set_low()
	}
	fn set_high(&mut self) -> Result<(), Self::Error> {
		self.0.set_high()
	}
}

mod panic_mutex {
	use core::sync::atomic::{AtomicBool, Ordering};

	/// Ensures mutually exclusive access to the contained `T`. This mutex
	/// implementation does *not* attempt to block / serialize concurrent
	/// data access. Rather, if concurrent locking is attempted, it panics.
	pub struct PanicMutex<T> {
		inner: core::cell::UnsafeCell<T>,
		busy: AtomicBool,
	}

	/// RAII-guard for [`PanicMutex`]. When this object is dropped, the mutex
	/// is unlocked.
	///
	/// Access to the protected data is possible via its [`Deref`] and [`DerefMut`]
	/// implementations.
	pub struct PanicMutexGuard<'a, T: 'a> {
		parent: &'a PanicMutex<T>,
	}

	impl<T> PanicMutex<T> {
		/// Constructs a new, unlocked `PanicMutex`.
		pub fn new(inner: T) -> Self {
			PanicMutex {
				inner: core::cell::UnsafeCell::new(inner),
				busy: AtomicBool::from(false),
			}
		}

		/// Locks the mutex and returns a `PanicMutexGuard` through which the data
		/// can be accessed. The mutex stays locked for the guard's lifetime.
		///
		/// Panics if the lock is already held by another guard.
		pub fn lock(&self) -> PanicMutexGuard<'_, T> {
			PanicMutexGuard::new(self)
		}

		/// Locks the mutex, applies the `f` function to its data, unlocks and
		/// returns the result of `f`.
		///
		/// Panics if the lock is already held.
		pub fn with_lock<R, F: FnOnce(&mut T) -> R>(&self, f: F) -> R {
			use core::ops::DerefMut;
			f(self.lock().deref_mut())
		}
	}

	/// Creates a static variable of type PanicMutex<$T> initialized with the specified expression and returns a `&'static` reference to it.
	// FIXME this is unsafe iirc. use cortex_m::singleton instead
	#[macro_export]
	macro_rules! new {
		($expr:expr; $T:ty) => {
			{
				let thing = $expr;
				unsafe {
					static mut _MANAGER: core::mem::MaybeUninit<panic_mutex::PanicMutex<$T>> = core::mem::MaybeUninit::uninit();
					_MANAGER = core::mem::MaybeUninit::new(panic_mutex::PanicMutex::new(thing));
					&*_MANAGER.as_ptr()
				}
			}
		};
	}
	pub use new;

	impl<'a, T: 'a> PanicMutexGuard<'a, T> {
		// SAFETY: This is the only constructor and thus ensures that only one guard
		// can exist for a given PanicMutex.
		fn new(parent: &'a PanicMutex<T>) -> Self {
			if
				atomic::compare_exchange(&parent.busy, false, true, Ordering::SeqCst, Ordering::SeqCst)
					.is_err() {
				panic!("PanicMutex conflict");
			}

			Self {
				parent
			}
		}
	}

	impl<'a, T: 'a> core::ops::Deref for PanicMutexGuard<'a, T> {
		type Target = T;

		fn deref<'b>(&'b self) -> &'b T {
			// SAFETY: Only one guard can exist. Its only constructor (new) ensures this.
			unsafe { & *self.parent.inner.get() }
		}
	}

	impl<'a, T: 'a> core::ops::DerefMut for PanicMutexGuard<'a, T> {
		fn deref_mut<'b>(&'b mut self) -> &'b mut T {
			// SAFETY: Only one guard can exist. Its only constructor (new) ensures this.
			unsafe { &mut *self.parent.inner.get() }
		}
	}

	impl<'a, T: 'a> core::ops::Drop for PanicMutexGuard<'a, T> {
		fn drop(&mut self) {
			self.parent.busy.store(false, Ordering::SeqCst);
		}
	}

	unsafe impl<T: Send> Sync for PanicMutex<T> {}
	unsafe impl<T: Send> Send for PanicMutex<T> {}

	#[cfg(not(any( target_has_atomic = "8", feature = "cortex-m")))]
	core::compile_error!("Your architecture does not support atomic operations.");
	
	#[cfg(all( not(target_has_atomic = "8"), feature = "cortex-m"))]
	mod atomic {
		use core::sync::atomic::{AtomicBool, Ordering};

		#[inline(always)]
		pub fn compare_exchange(
			atomic: &AtomicBool,
			current: bool,
			new: bool,
			_success: Ordering,
			_failure: Ordering,
		) -> Result<bool, bool> {
			cortex_m::interrupt::free(|_cs| {
				let prev = atomic.load(Ordering::Acquire);
				if prev == current {
					atomic.store(new, Ordering::Release);
					Ok(prev)
				} else {
					Err(false)
				}
			})
		}
	}

	#[cfg(target_has_atomic = "8")]
	mod atomic {
		use core::sync::atomic::{AtomicBool, Ordering};

		#[inline(always)]
		pub fn compare_exchange(
			atomic: &AtomicBool,
			current: bool,
			new: bool,
			success: Ordering,
			failure: Ordering,
		) -> Result<bool, bool> {
			atomic.compare_exchange(current, new, success, failure)
		}
	}
}

mod shared_spi {
	use super::panic_mutex::PanicMutex;
	use embedded_hal::blocking::spi::Write;

	#[derive(Clone, Copy)]
	pub struct SharedSpi<BUS: 'static>(pub &'static PanicMutex<BUS>);

	impl<BUS: Write<u8>> Write<u8> for SharedSpi<BUS> {
		type Error = BUS::Error;
		fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
			self.0.lock().write(data)
		}
	}
}

mod shared_gpio {
	use super::panic_mutex::PanicMutex;
	use embedded_hal::digital::v2::OutputPin;

	#[derive(Copy)]
	pub struct SharedGpio<GPIO: 'static>(pub &'static PanicMutex<GPIO>);

	impl<GPIO: 'static> Clone for SharedGpio<GPIO> {
		fn clone(&self) -> Self {
			Self(self.0)
		}
	}

	impl<GPIO: OutputPin> OutputPin for SharedGpio<GPIO> {
		type Error = GPIO::Error;
		fn set_low(&mut self) -> Result<(), Self::Error> {
			self.0.lock().set_low()
		}
		fn set_high(&mut self) -> Result<(), Self::Error> {
			self.0.lock().set_high()
		}
	}
}

mod shift_register {
	use super::panic_mutex::PanicMutex;
	use embedded_hal::digital::v2::OutputPin;

	pub struct ShiftRegister<LATCH: OutputPin, CLOCK: OutputPin, DATA: OutputPin> {
		latch: LATCH,
		clock: CLOCK,
		data: DATA,
		value: u8
	}

	impl<LATCH: OutputPin, CLOCK: OutputPin, DATA: OutputPin> ShiftRegister<LATCH, CLOCK, DATA> {
		pub fn new(latch: LATCH, clock: CLOCK, data: DATA, value: u8) -> Self {
			let mut result = Self { latch, clock, data, value };
			result.send();
			result
		}

		pub fn split(this: &PanicMutex<Self>) -> [Pin<LATCH, CLOCK, DATA>; 8] {
			[
				Pin::new(this, 0),
				Pin::new(this, 1),
				Pin::new(this, 2),
				Pin::new(this, 3),
				Pin::new(this, 4),
				Pin::new(this, 5),
				Pin::new(this, 6),
				Pin::new(this, 7),
			]
		}

		fn set_pin(&mut self, pin: u8, value: bool) {
			if value {
				self.value = self.value | (1 << pin);
			}
			else {
				self.value = self.value & !(1 << pin);
			}
			self.send();
		}

		fn send(&mut self) {
			self.latch.set_low();
			for i in 0..8 {
				self.send_bit(self.value & (1<<(7-i)) != 0);
			}
			self.latch.set_high();
			Self::delay();
		}

		fn send_bit(&mut self, bit: bool) {
			if bit { self.data.set_high(); } else { self.data.set_low(); }
			self.clock.set_low();
			Self::delay();
			self.clock.set_high();
			Self::delay();
		}

		fn delay() {
			cortex_m::asm::delay(3);
		}
	}

	pub struct Pin<'a, LATCH: OutputPin, CLOCK: OutputPin, DATA: OutputPin> {
		shift_register: &'a PanicMutex<ShiftRegister<LATCH, CLOCK, DATA>>,
		index: u8,
	}

	impl<'a, LATCH: OutputPin, CLOCK: OutputPin, DATA: OutputPin> Pin<'a, LATCH, CLOCK, DATA> {
		pub fn new(shift_register: &'a PanicMutex<ShiftRegister<LATCH, CLOCK, DATA>>, index: u8) -> Self {
			Self { shift_register, index }
		}
	}

	impl<'a, LATCH: OutputPin, CLOCK: OutputPin, DATA: OutputPin> OutputPin for Pin<'a, LATCH, CLOCK, DATA> {
		type Error = core::convert::Infallible;
		fn set_low(&mut self) -> Result<(), Self::Error> {
			self.shift_register.lock().set_pin(self.index, false);
			Ok(())
		}
		fn set_high(&mut self) -> Result<(), Self::Error> {
			self.shift_register.lock().set_pin(self.index, true);
			Ok(())
		}
	}
}

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



type ShiftRegisterType = shift_register::ShiftRegister<PB14<Output<PushPull>>, PA0<Output<PushPull>>, PA1<Output<PushPull>>>;
type ShiftRegisterPin = shift_register::Pin<'static, PB14<Output<PushPull>>, PA0<Output<PushPull>>, PA1<Output<PushPull>>>;



#[derive(Clone, Copy, PartialEq)]
pub enum Calib
{
	Pending,
	Running,
	Idle
}

#[derive(Clone, Copy)]
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
	jump_count: u32,
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
			jump_count: 0,
			measured_history: Queue::new(),
			setpoint_history: Queue::new()
		}
	}
	pub fn process(&mut self, setpoint: f32, measured: f32) -> FaderResult {
		let JUMP_THRESHOLD = 3.0;
		let SET_DEADZONE = 0.01;
		let SETPOINT_DEADZONE = 0.001;
		let ESCAPE_LIMIT = 0.2;
		let INPUT_DEADZONE = 0.05;
		let INPUT_DEADZONE_WHEN_MIDI_CONTROLLED = 0.15;
		let CAPTURE_ZONE = 0.06;
		let STABILIZE_TIMEOUT = 500;
		let JUMP_TIMEOUT = 2000;

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

		match self.state {
			FaderState::Idle => {
				if setpoint_diff >= JUMP_THRESHOLD {
					self.state = FaderState::MidiControlledJump;
					self.jump_count = 0;
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
					self.jump_count = 0;
				}
				if self.stable_timer >= STABILIZE_TIMEOUT {
					self.state = FaderState::Idle;
				}
				if (setpoint - measured).abs() >= ESCAPE_LIMIT {
					self.state = FaderState::UserOverride;
					self.measured_history.push(1.0);
					self.measured_history.push(0.0);
				}
				else if (setpoint - measured).abs() >= INPUT_DEADZONE_WHEN_MIDI_CONTROLLED {
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
				if self.jump_count >= JUMP_TIMEOUT {
					self.state = FaderState::Idle;
				}
				self.jump_count += 1;

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
						self.jump_count = 0;
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
						self.jump_count = 0;
					}
				}

				FaderResult {
					fader_move_target: None,
					midi_send_value: Some(measured)
				}
			}
		}
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

type SpiType = spi::Spi<stm32f1xx_hal::pac::SPI2, spi::Spi2NoRemap, (PB13<Alternate<PushPull>>, spi::NoMiso, PB15<Alternate<PushPull>>), u8>;

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
	struct Resources {
		delay: stm32f1xx_hal::delay::Delay,
		mytimer: timer::CountDownTimer<TIM2>,

		led: PC13<Output<PushPull>>,

		usb_dev: UsbDevice<'static, UsbBusType>,
		midi: usbd_midi::midi_device::MidiClass<'static, UsbBus<Peripheral>>,
		dac: Mcp49xx<SpiInterface<SharedBus<SpiType>, OldOutputPin<PC15<Output<PushPull>>>>, Resolution12Bit, DualChannel, Unbuffered>,

		x3423: X3423,

		displays: [
			ssd1306::Ssd1306<
				ssd1306::prelude::SPIInterface<
					SharedBus<SpiType>,
					shared_gpio::SharedGpio<PB5<Output<PushPull>>>,
					RefMutOutputPin<'static, dyn embedded_hal::digital::v2::OutputPin<Error = Infallible> + Send>
				>,
				ssd1306::size::DisplaySize128x64,
				ssd1306::mode::BufferedGraphicsMode<ssd1306::size::DisplaySize128x64>
			>; 2
		],

		faders: [Fader; 17],
		fader_config: [FaderConfig; 17],
		fader_midi_commands: [Option<f32>; 17],
		fader_processes_midi_command: [bool; 17],
		calibration_request: Calib,

		flash: stm32f1xx_hal::flash::Parts,

		midi_sender: MidiSender,
		fader_bidir: [FaderStateMachine; 17],
		fader_bidir_target: [SlewTarget; 17],
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
		let spi: SpiType = spi::Spi::spi2(device.SPI2, (sck, spi::NoMiso, mosi), spi::Mode { phase : spi::Phase::CaptureOnFirstTransition, polarity : spi::Polarity::IdleLow }, 10.mhz(), clocks, &mut rcc.apb1);

		let shared_spi = shared_bus_rtic::new!(spi, SpiType);

		
		// Configure display
		let chip_selects = {
			// Configure shift register
			let shiftreg_data = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
			let shiftreg_clock = gpioa.pa0.into_push_pull_output(&mut gpioa.crl); // on black pill, this is pb10
			let shiftreg_latch = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
			let shift_register = shift_register::ShiftRegister::new(shiftreg_latch, shiftreg_clock, shiftreg_data, 0xFF);
			let cs_0to7 = cortex_m::singleton!(: [ShiftRegisterPin; 8] = {
				shift_register::ShiftRegister::split(panic_mutex::new!(shift_register; ShiftRegisterType))
			}).unwrap();
		
			// Configure the 9th chip select pin
			let pc14 = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
			let cs8 = cortex_m::singleton!(: PC14<Output<PushPull>> = pc14).unwrap();
			cs8.set_high();

			cs_0to7.iter_mut()
				.map(|x| x as &mut (dyn OutputPin<Error = Infallible> + Send))
				.chain(core::iter::once(cs8 as _))
		};


			let display_dc = shared_gpio::SharedGpio(panic_mutex::new!(gpiob.pb5.into_push_pull_output(&mut gpiob.crl); PB5<Output<PushPull>>));

			let mut displays = chip_selects.map(|cs|
				ssd1306::Ssd1306::new(
					ssd1306::prelude::SPIInterface::new(shared_spi.acquire(), display_dc.clone(), RefMutOutputPin(cs)),
					ssd1306::size::DisplaySize128x64,
					ssd1306::rotation::DisplayRotation::Rotate0
				)
			);

			displays.next().unwrap();
			displays.next().unwrap();
			let mut displays: [_; 2] = [
				displays.next().unwrap(),
				displays.next().unwrap(),
				//displays.next().unwrap(),
				/*displays.next().unwrap(),
				displays.next().unwrap(),
				displays.next().unwrap(),
				displays.next().unwrap(),
				displays.next().unwrap(),
				displays.next().unwrap(),*/
			];

			for display in displays.iter_mut() {
				use ssd1306::mode::DisplayConfig;
				display.init().unwrap();
			}
		//}

		let dac_cs: OldOutputPin<_> = gpioc.pc15.into_push_pull_output_with_state(&mut gpioc.crh, State::High).into();
		let dac = mcp49xx::Mcp49xx::new_mcp4822(shared_spi.acquire(), dac_cs);

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
			fader_bidir_target: [SlewTarget::new(); 17],
			displays
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

	
	#[task(binds = TIM2, resources = [x3423, dac, displays, delay, faders, fader_midi_commands, mytimer, led, midi, fader_config, fader_processes_midi_command, calibration_request, flash, midi_sender, fader_bidir, fader_bidir_target], priority=1)]
	fn xmain(mut c : xmain::Context) {
		static mut BLINK: u64 = 0;
		static mut FOO: i32 = 0;


		use embedded_graphics::{
			pixelcolor::BinaryColor,
			mono_font::{ascii::FONT_6X10, MonoTextStyle},
			prelude::*,
			primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle},
			text::Text,
		};

		let res = &mut c.resources;
		res.mytimer.clear_update_interrupt_flag();
		
		*BLINK += 1;
		if *BLINK % 100 == 0 {
			*FOO = (*FOO + 1) % 20;
			res.led.toggle().unwrap();

			let display = &mut res.displays[0];

			display.clear(BinaryColor::Off);

			/*	
			let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
			Text::new("Hello Rust!", Point::new(20+*FOO, 30), style).draw(display).ok();

			for i in 0..15 {
				display.set_pixel(i, i, true);
			}

			display.set_draw_area((10,10),(128,64));

			display.flush().ok();
			*/
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

/*
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
		*/
	}

	#[task(binds = USB_LP_CAN_RX0, resources=[midi, usb_dev, fader_midi_commands, fader_config, calibration_request], priority=2)]
	fn periodic_usb_poll(c : periodic_usb_poll::Context) {
		static mut LSB: [u8; 17] = [0; 17];

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
								c.resources.fader_midi_commands[(cc-1) as usize] = Some( (((value as u16) << 7) + LSB[(cc-1) as usize] as u16) as f32 / 16384.0);
							}
							if (33..=49).contains(&cc) {
								LSB[cc as usize - 33] = value;
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
