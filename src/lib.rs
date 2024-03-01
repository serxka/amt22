#![cfg_attr(not(feature = "std"), no_std)]

use embedded_hal::blocking::{
	delay::DelayMs,
	spi::{Transfer, Write},
};

#[derive(Debug)]
pub enum Error<E> {
	/// SPI bus error
	Spi(E),
	/// Checkbit error
	InvalidChecksum,
    /// Not a single-turn encoder error
    NotSingleTurn,
    /// Not a multi-turn encoder error
    NotMultiTurn
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Resolution {
    /// Single-turn 12-bit resolution encoder (AMT22XA)
	Single12Bit,
    /// Single-turn 14-bit resolution encoder (AMT22XB)
	Single14Bit,
    /// Multi-turn 12-bit resolution encoder (AMT22XC)
	Multi12Bit,
    /// Multi-turn 14-bit resolution encoder (AMT22XD)
	Multi14Bit,
}

/// A simple interface to the AMT22 Encoders, prefer SPI mode0 and a data rate below 2Mhz (500Khz
/// works great)
pub struct Amt22<S> {
	spi: S,
	res: Resolution,
}

impl<S> Amt22<S> {
	/// The number of ticks per rotation for 12-bit resolution units
	pub const TICKS_PER_ROTATION_12B: u16 = 4096;
	/// The number of ticks per rotation for 14-bit resolution units
	pub const TICKS_PER_ROTATION_14B: u16 = 16384;

	#[inline]
	fn checksum(x: u16) -> bool {
		let sm = |i: u16| -> u16 { (x >> i) & 0x01 };
		let k1 = sm(15) == ((sm(13) ^ sm(11) ^ sm(9) ^ sm(7) ^ sm(5) ^ sm(3) ^ sm(1)) == 0) as _;
		let k0 = sm(14) == ((sm(12) ^ sm(10) ^ sm(8) ^ sm(6) ^ sm(4) ^ sm(2) ^ sm(0)) == 0) as _;
		k1 && k0
	}

	#[inline]
	fn resolution_shift(&self) -> u16 {
		match self.res {
			Resolution::Single14Bit | Resolution::Multi14Bit => 0,
			Resolution::Single12Bit | Resolution::Multi12Bit => 2,
		}
	}

    /// Returns how many ticks per rotation of the encoder, this with either be
    /// `TICKS_PER_ROTATION_12B` or `TICKS_PER_ROTATION_14B` depending on the resolution set.
	#[inline]
	pub fn resolution_ticks(&self) -> u16 {
		match self.res {
			Resolution::Single14Bit | Resolution::Multi14Bit => Self::TICKS_PER_ROTATION_14B,
			Resolution::Single12Bit | Resolution::Multi12Bit => Self::TICKS_PER_ROTATION_12B,
		}
	}

    /// Is this a single-turn encoder
	#[inline]
    pub fn is_single_turn(&self) -> bool {
        matches!(self.res, Resolution::Single12Bit | Resolution::Single14Bit)
    }

    /// Is this a multi-turn encoder.
	#[inline]
    pub fn is_multi_turn(&self) -> bool {
        matches!(self.res, Resolution::Multi12Bit | Resolution::Multi14Bit)
    }
}

impl<S, E> Amt22<S>
where
	S: Transfer<u8, Error = E> + Write<u8, Error = E>,
{
	/// Pure constructor, no I/O is performed by this function. See [`Amt22::reset`]
	/// to reset the encoder.
	#[inline]
	pub fn new(spi: S, resolution: Resolution) -> Self {
		Self {
			spi,
			res: resolution,
		}
	}

	/// This will send a reset command to the AMT22. If `delay` is set to `None` then this function
	/// will not block and returns after the command is sent. Otherwise, if `delay` is set than a
	/// 250ms delay will occur to wait for the encoder to reset fully. If this delay is not passed
	/// then you should wait before trying to contact the encoder again.
	///
	/// The encoder must also be stationary to power on again.
	///
	/// # Errors
	/// Can return a SPI error on failure.
	pub fn reset(&mut self, delay: Option<&mut dyn DelayMs<u16>>) -> Result<(), Error<E>> {
		self.spi.write(&[0x00, 0x60]).map_err(Error::Spi)?;
		if let Some(delay) = delay {
			delay.delay_ms(250);
		}
		Ok(())
	}

	/// Reset the zero point of the encoder. This function operates similarly to [`Self::reset()`].
	/// The encoder will not power back on unless the encoder is stationary.
	///
	/// # Special Use
	/// This function is only supported on single-turn encoders.
	///
	/// # Errors
	/// Can return a SPI error on failure or mismatched encoder type.
	pub fn reset_zero_point(
		&mut self,
		delay: Option<&mut dyn DelayMs<u16>>,
	) -> Result<(), Error<E>> {
        if !self.is_single_turn() {
            return Err(Error::NotSingleTurn);
        }

		self.spi.write(&[0x00, 0x70]).map_err(Error::Spi)?;
		if let Some(delay) = delay {
			delay.delay_ms(250);
		}
		Ok(())
	}

	/// Read the rotation of the encoder relative to its current rotation. This number will wrap on
	/// each completed rotation. Values will be within `[0, TICKS_PER_ROTATION]`
	///
	/// # Errors
	/// Can return a SPI error on failure or a checksum error on failure.
	pub fn read_relative_position(&mut self) -> Result<u16, Error<E>> {
		let mut buf = [0x00, 0x00];
		self.spi.transfer(&mut buf).map_err(Error::Spi)?;
		let position: u16 = ((buf[0] as u16) << 8) | (buf[1] as u16);

		if Self::checksum(position) {
			Ok((position & 0x3FFF) >> self.resolution_shift())
		} else {
			Err(Error::InvalidChecksum)
		}
	}

	/// Read the number of turns the encoder has done and where in the current rotation it is.
	/// The number of rotation will always wrap mod tick_per_rotation, but the number of turns can
	/// overflow.
	///
	/// # Special Use
	/// This function is only supported on multi-turn encoders.
	///
	/// # Errors
	/// Can return a SPI error on failure or a checksum error on failure or mismatched encoder type.
	pub fn read_absolute_position_raw(&mut self) -> Result<(i16, u16), Error<E>> {
        if !self.is_multi_turn() {
            return Err(Error::NotMultiTurn);
        }

		let mut buf = [0x00, 0xA0, 0x00, 0x00];
		self.spi.transfer(&mut buf).map_err(Error::Spi)?;
		let position: u16 = ((buf[0] as u16) << 8) | (buf[1] as u16);
		let turns: u16 = ((buf[2] as u16) << 8) | (buf[3] as u16);

		if Self::checksum(position) && Self::checksum(turns) {
			let position = (position & 0x3FFF) >> self.resolution_shift();
			let turns = (turns << 2) as i16 >> 2;
			Ok((turns, position))
		} else {
			Err(Error::InvalidChecksum)
		}
	}

	/// This function is the same as [`Self::read_absolute_position_raw()`] but will convert each
	/// into the appropriate amount of ticks instead. This function also suffers from overflow when
	/// too many completed turns have occurred.
	///
	/// See [`Self::read_absolute_position_raw()`] for other errors and proper use cases.
	pub fn read_absolute_position(&mut self) -> Result<i32, Error<E>> {
		let (turns, position) = self.read_absolute_position_raw()?;
        let turns = turns as i32 * self.resolution_ticks() as i32 + position as i32;
		Ok(turns)
	}
}
