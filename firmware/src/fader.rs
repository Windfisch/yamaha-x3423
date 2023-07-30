use micromath::F32Ext;

enum FaderCalibrationPhase {
	NotCalibrating,
	Failed,
	Init,
	ApproachMin,
	ApproachMidFromBelow,
	ApproachMax,
	ApproachMidFromAbove,
	Approach75FromBelow,
	Approach25FromAbove
}

pub enum CalibrationStatus {
	Ok,
	Failed,
	Processing,
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
			calibration_phase: FaderCalibrationPhase::NotCalibrating
		}
	}

	pub fn start_calibration(&mut self) {
		self.calibration_phase = FaderCalibrationPhase::Init;
	}

	pub fn get_calibration_data(&self) -> [u8; 10] {
		[
			(self.write_low & 0xFF) as u8,
			(self.write_low >> 8) as u8,
			(self.write_high & 0xFF) as u8,
			(self.write_high >> 8) as u8,
			(self.write_deadzone & 0xFF) as u8,
			(self.write_deadzone >> 8) as u8,
			(self.read_low & 0xFF) as u8,
			(self.read_low >> 8) as u8,
			(self.read_high & 0xFF) as u8,
			(self.read_high >> 8) as u8,
		]
	}

	pub fn is_calibrating(&self) -> bool {
		match self.calibration_phase {
			FaderCalibrationPhase::NotCalibrating => false,
			FaderCalibrationPhase::Failed => false,
			_ => true
		}
	}

	pub fn calibration_status(&self) -> CalibrationStatus {
		match self.calibration_phase {
			FaderCalibrationPhase::NotCalibrating => CalibrationStatus::Ok,
			FaderCalibrationPhase::Failed => CalibrationStatus::Failed,
			_ => CalibrationStatus::Processing,
		}
	}

	pub fn set_calibration_data(&mut self, data: &[u8]) {
		self.write_low      = (data[0] as u16) + ((data[1] as u16) << 8);
		self.write_high     = (data[2] as u16) + ((data[3] as u16) << 8);
		self.write_deadzone = (data[4] as u16) + ((data[5] as u16) << 8);
		self.read_low       = (data[6] as u16) + ((data[7] as u16) << 8);
		self.read_high      = (data[8] as u16) + ((data[9] as u16) << 8);
	}

	pub fn update_value(&mut self, raw: u16) {
		self.last_value = self.cook_raw_read_value(raw);
		self.last_value_raw = raw;
	}

	fn cook_raw_read_value(&self, raw: u16) -> f32 {
		if raw <= self.read_low { 0. }
		else if raw >= self.read_high { 1. }
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
		if !self.is_calibrating() {
			self.target_value = Some(target.clamp(0., 1.));
			self.time_left = TIMEOUT;
		}
	}
	pub fn clear_target(&mut self) {
		if !self.is_calibrating() {
			self.target_value = None;
		}
	}
	pub fn target(&self) -> Option<f32> {
		self.target_value
	}
	pub fn process(&mut self) -> Option<u16> {
		if !self.is_calibrating() {
			self.process_normal()
		}
		else {
			self.process_calibration()
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

					// should be approximately 0.5, but can deviate because the WRITE_xx_APPROX setpoints are just coarsely guesstimated.
					let distance = v75_cooked - v25_cooked; 
					
					if distance < 0.1 {
						// this is definitely unplausible. calibration has failed.
						// we'll use some default values just for the sake of having something remotely plausible, yet wrong
						self.write_low = WRITE_25_APPROX;
						self.write_high = WRITE_75_APPROX;
						self.write_deadzone = 0;
						self.calibration_phase = Failed;
					}
					else {
						let gain = (WRITE_75_APPROX - WRITE_25_APPROX) as f32 / distance;
						assert!(0. < gain);
						self.write_low = WRITE_25_APPROX - ((v25_cooked * gain) as u16);
						self.write_high = self.write_low + (gain as u16);
						self.write_deadzone = ((self.cook_raw_read_value(self.mid_from_above) - self.cook_raw_read_value(self.mid_from_below)).max(0.0) * gain) as u16;
						self.calibration_phase = NotCalibrating;
					}

				}
				Some(WRITE_75_APPROX)
			}
			_ => { None }
		}
		
	}

	fn process_normal(&mut self) -> Option<u16> {
		if let Some(target) = self.target_value {
			if (self.last_value - target).abs() < 0.02 {
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

