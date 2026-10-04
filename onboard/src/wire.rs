//! Little-endian primitives the trace is written in, over a fixed buffer: no allocation, so
//! the board decodes with the same code the host encodes with.

/// Writes into a borrowed buffer, and remembers running out rather than panicking.
pub struct Writer<'a> {
    buffer: &'a mut [u8],
    at: usize,
    overflowed: bool,
}

impl<'a> Writer<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self {
            buffer,
            at: 0,
            overflowed: false,
        }
    }

    /// The bytes written, or `None` if any write did not fit.
    pub fn finish(self) -> Option<usize> {
        (!self.overflowed).then_some(self.at)
    }

    pub fn bytes(&mut self, bytes: &[u8]) {
        match self.buffer.get_mut(self.at..self.at + bytes.len()) {
            Some(slot) => {
                slot.copy_from_slice(bytes);
                self.at += bytes.len();
            }
            None => self.overflowed = true,
        }
    }

    pub fn u8(&mut self, value: u8) {
        self.bytes(&[value]);
    }

    pub fn u64(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }

    pub fn f32(&mut self, value: f32) {
        self.bytes(&value.to_bits().to_le_bytes());
    }

    pub fn f64(&mut self, value: f64) {
        self.bytes(&value.to_bits().to_le_bytes());
    }

    pub fn bool(&mut self, value: bool) {
        self.u8(u8::from(value));
    }

    pub fn f32s<const N: usize>(&mut self, values: [f32; N]) {
        for value in values {
            self.f32(value);
        }
    }

    /// `None` as one zero byte, `Some(x)` as a one and `x`.
    pub fn option_f32(&mut self, value: Option<f32>) {
        match value {
            None => self.u8(0),
            Some(x) => {
                self.u8(1);
                self.f32(x);
            }
        }
    }
}

/// Reads from a borrowed buffer; every read past its end is `None`.
pub struct Reader<'a> {
    buffer: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    pub fn new(buffer: &'a [u8]) -> Self {
        Self { buffer, at: 0 }
    }

    /// Whether every byte was read, which a decode checks so a trailing byte is an error
    /// rather than the start of a record nobody reads.
    pub fn is_empty(&self) -> bool {
        self.at == self.buffer.len()
    }

    pub fn array<const N: usize>(&mut self) -> Option<[u8; N]> {
        let bytes = self.buffer.get(self.at..self.at + N)?;
        self.at += N;
        bytes.try_into().ok()
    }

    pub fn u8(&mut self) -> Option<u8> {
        self.array::<1>().map(|[b]| b)
    }

    pub fn u64(&mut self) -> Option<u64> {
        self.array().map(u64::from_le_bytes)
    }

    pub fn f32(&mut self) -> Option<f32> {
        self.array().map(|b| f32::from_bits(u32::from_le_bytes(b)))
    }

    pub fn f64(&mut self) -> Option<f64> {
        self.array().map(|b| f64::from_bits(u64::from_le_bytes(b)))
    }

    pub fn bool(&mut self) -> Option<bool> {
        match self.u8()? {
            0 => Some(false),
            1 => Some(true),
            _ => None,
        }
    }

    pub fn f32s<const N: usize>(&mut self) -> Option<[f32; N]> {
        let mut values = [0.0; N];
        for value in &mut values {
            *value = self.f32()?;
        }
        Some(values)
    }

    pub fn option_f32(&mut self) -> Option<Option<f32>> {
        match self.u8()? {
            0 => Some(None),
            1 => self.f32().map(Some),
            _ => None,
        }
    }
}
