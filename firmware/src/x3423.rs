use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::gpio::{gpioa::*, gpiob::*, Analog, Alternate, Output, OpenDrain};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::adc::Adc;
use stm32f1xx_hal::pac::ADC1;
use core::convert::Infallible;

use micromath::F32Ext;

pub struct X3423DataPins {
	pub d0: PB4<Output<OpenDrain>>,
	pub d1: PB6<Output<OpenDrain>>,
	pub d2: PB7<Output<OpenDrain>>,
	pub d3: PB8<Output<OpenDrain>>,
	pub d4: PB9<Output<OpenDrain>>,
	pub d5: PB10<Output<OpenDrain>>,
	pub d6: PB11<Output<OpenDrain>>,
	pub d7: PB12<Output<OpenDrain>>,
}

pub struct X3423 {
	res: X3423Resources
}

pub struct X3423Resources {
	pub mfpos0: PA5<Analog>,
	pub mfpos1: PA6<Analog>,
	pub mfpos2: PA7<Analog>,
	pub mfpos3: PB0<Analog>,
	pub mfpos4: PB1<Analog>,
	pub reset: PB3<Output<OpenDrain>>,
	pub fd0: PA8<Output<OpenDrain>>,
	pub fd1: PA9<Output<OpenDrain>>,
	pub fd2: PA10<Output<OpenDrain>>,
	pub fd3: PA15<Output<OpenDrain>>,
	pub data: X3423DataPins,
	pub adc: Adc<ADC1>,
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
	pub fn new(res: X3423Resources) -> X3423 {
		X3423 { res }
	}

	pub fn reset(&mut self) {
		self.res.reset.set_low();
		cortex_m::asm::delay(5);
		self.res.reset.set_high();
	}

	pub fn set_motor_enable(&mut self, state: u32) {
		self.res.data.apply_data((state & 0xFF) as u8, &mut self.res.fd0);
		self.res.data.apply_data(((state >> 8) & 0xFF) as u8, &mut self.res.fd1);
		self.res.data.apply_data(((state >> 16) & 0x01) as u8, &mut self.res.fd2);
	}

	fn set_other(&mut self, dasel: u8, dainh: u8, adsel: u8) {
		self.res.data.apply_data((dasel & 7) | ((dainh & 7) << 3) | ((adsel & 3) << 6), &mut self.res.fd3);
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

	pub fn read_values(&mut self, mut callback: impl FnMut(usize, u16), delay: &mut impl embedded_hal::blocking::delay::DelayUs<u32>) {
		for i in 0..4 {
			self.select_analog_value(i as u8, delay);
			callback(0 + i, self.res.adc.read(&mut self.res.mfpos0).unwrap());
			callback(4 + i, self.res.adc.read(&mut self.res.mfpos1).unwrap());
			callback(8 + i, self.res.adc.read(&mut self.res.mfpos2).unwrap());
			callback(12 +i, self.res.adc.read(&mut self.res.mfpos3).unwrap());
		}
		callback(16, self.res.adc.read(&mut self.res.mfpos4).unwrap());
	}
}

