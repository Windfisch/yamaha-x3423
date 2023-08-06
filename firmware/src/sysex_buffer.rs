pub struct SysexBuffer<const N: usize> {
	buffer: [u8; N],
	len: usize,
	done: bool,
}

impl<const N: usize> SysexBuffer<N> {
	pub fn new() -> Self {
		Self {
			buffer: [0; N],
			len: 0,
			done: false,
		}
	}

	/// Returns true when a sysex has finished receiving, false otherwise.
	pub fn push_midi(&mut self, usb_type: u8, message: &[u8]) -> bool {
		if self.done {
			return true;
		}

		let (len, done) = match usb_type {
			0x4 => (3, false),
			0x5 => (1, true),
			0x6 => (2, true),
			0x7 => (3, true),
			_ => return false
		};

		for i in 0..len {
			if self.len < N {
				self.buffer[self.len] = message[i+1];
			}
			self.len += 1;
		}

		if done {
			if self.len > N {
				// if the sysex was too long for our buffer, just discard it.
				self.len = 0;
			}
			else {
				self.done = true;
			}
		}

		self.done
	}

	pub fn get(&mut self) -> Option<&mut [u8]> {
		if self.done {
			let len = self.len;
			self.len = 0;
			self.done = false;
			Some(&mut self.buffer[0..len])
		}
		else {
			None
		}
	}
}

/// Squashes a MIDI 7-bit-per-byte message into an 8-bit-per-byte buffer, discarding up to 7 trailing bits.
///
/// This is done by laying out the 7-bit midi bytes as a stream of bits, LSB-first, and then regrouping them
/// into chunks of 8. Example:
///
/// ```
/// 0x42 0x13 0x37` = 0b01000010 0b00010011 0b00100111
/// becomes
/// 0100001 1100100 1110110
/// 01000011 10010011 10110xxx
/// becomes
/// 0xc2 0xc9
/// ```
pub fn bitsquash_inplace(buffer: &mut [u8]) -> &mut [u8]{
	let result_len = buffer.len() * 7 / 8;
	for w_byte in 0..result_len {
		let bit_begin = w_byte * 8;

		let r_byte_begin = bit_begin / 7;
		let r_bit_begin = bit_begin % 7;

		debug_assert!(w_byte <= r_byte_begin);

		buffer[w_byte] = ((buffer[r_byte_begin] & 0x7F) >> r_bit_begin) | (buffer[r_byte_begin + 1] << (7 - r_bit_begin));
	}
	&mut buffer[0..result_len]
}
