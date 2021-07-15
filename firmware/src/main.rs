#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_halt;
use embedded_hal::digital::v2::OutputPin;
use rtic::app;
use rtic::cyccnt::U32Ext;
use stm32f1xx_hal::gpio::{gpioa::*, gpiob::*, gpioc::*, Input, Floating, Analog, Output, PushPull, OpenDrain, State};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::delay::Delay;
use core::convert::Infallible;

const PERIOD: u32 = 100_000_000;

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
	d0: PB4<Output<OpenDrain>>,
	d1: PB6<Output<OpenDrain>>,
	d2: PB7<Output<OpenDrain>>,
	d3: PB8<Output<OpenDrain>>,
	d4: PB9<Output<OpenDrain>>,
	d5: PB10<Output<OpenDrain>>,
	d6: PB11<Output<OpenDrain>>,
	d7: PB12<Output<OpenDrain>>,
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
	cortex_m::asm::delay(200);
}


impl X3423 {
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

	pub fn set_motor_enable(&mut self, state: u32) {
		self.write_data((state & 0xFF) as u8).unwrap();
		wait();
		self.fd0.set_high();
		wait();
		self.fd0.set_low();
		
		self.write_data(((state >> 8) & 0xFF) as u8).unwrap();
		wait();
		self.fd1.set_high();
		wait();
		self.fd1.set_low();

		self.d0.set(state & 0x10000 != 0).unwrap();
		wait();
		self.fd2.set_high();
		wait();
		self.fd2.set_low();
	}
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
	struct Resources {
		led: PC13<Output<PushPull>>,
		x3423: X3423
	}

	#[init]
	fn init(cx: init::Context) -> init::LateResources {
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
				d0: pb4.into_open_drain_output(&mut gpiob.crl),
				d1: gpiob.pb6.into_open_drain_output(&mut gpiob.crl),
				d2: gpiob.pb7.into_open_drain_output(&mut gpiob.crl),
				d3: gpiob.pb8.into_open_drain_output(&mut gpiob.crh),
				d4: gpiob.pb9.into_open_drain_output(&mut gpiob.crh),
				d5: gpiob.pb10.into_open_drain_output(&mut gpiob.crh),
				d6: gpiob.pb11.into_open_drain_output(&mut gpiob.crh),
				d7: gpiob.pb12.into_open_drain_output(&mut gpiob.crh),
			}
		};

		cortex_m::asm::delay(5);
		resources.x3423.reset.set_high();

		
		for i in 0..18 {
			resources.x3423.set_motor_enable(3 << i);
			delay.delay_ms(50_u16);
			resources.x3423.set_motor_enable(0);
			delay.delay_ms(1000_u16);
		}
		
		/*delay.delay_ms(1000_u16);
		resources.x3423.set_motor_enable(0x1FFFF);
		delay.delay_ms(300_u16);
		resources.x3423.set_motor_enable(0);*/

		resources
	}
};
