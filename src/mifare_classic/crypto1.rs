use core::iter::repeat;

use bitvec::{index::BitIdx, prelude::*, store::BitStore, field::BitField};

use super::MfClassicKey;

#[derive(Clone, Copy, Default, Debug, defmt::Format)]
pub struct Crypto1 {
    even: u32,
    odd: u32,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum CryptoError {
    // Byte index of parity error
    Parity(usize),
    // Required buffer capacity
    Capacity(usize),
}

/// true if odd
#[inline]
fn parity_u32(w: u32) -> bool {
    (0..=4)
        .rev()
        .map(|pow2| 1u32 << pow2)
        .map(|shift| (1u32.checked_shl(shift).unwrap_or(0).wrapping_sub(1), shift))
        .fold(w, |w, (mask, shift)| {
            (w & mask) ^ (w.checked_shr(shift).unwrap_or(0))
        })
        != 0
}
#[inline]
fn parity_u8(mut w: u8) -> bool {
    w = (w & 0xf) ^ (w >> 4);
    w = (w & 0b11) ^ (w >> 2);
    w = (w & 0b1) ^ (w >> 1);
    w != 0
}

// i have some clue how this works but i may have the wrong bit order
impl Crypto1 {
    const LF_POLY_ODD: u32 = 0x29CE5C;
    const LF_POLY_EVEN: u32 = 0x870804;

    /// Initializes the stream from a 48 bit key
    pub fn init(MfClassicKey{key, ..}: MfClassicKey) -> Self {
        let mut ret = Self::default();
        for bit in (0u8..=47).rev().step_by(2) {
            ret.odd <<= 1;
            ret.odd |= key.get_bit::<Lsb0>(BitIdx::new((bit - 1) ^ 0b111).unwrap()) as u32;

            ret.even <<= 1;
            ret.even |= key.get_bit::<Lsb0>(BitIdx::new(bit ^ 0b111).unwrap()) as u32;
        }

        ret
    }

    fn filter(i: u32) -> bool {
        let mut out = 0xf22c0 >> (i & 0xf) & 16;
        out |= 0x6c9c0 >> (i >> 4 & 0xf) & 8;
        out |= 0x3c8b0 >> (i >> 8 & 0xf) & 4;
        out |= 0x1e458 >> (i >> 12 & 0xf) & 2;
        out |= 0x0d938 >> (i >> 16 & 0xf) & 1;
        0xEC57E80Au32.get_bit::<Lsb0>(BitIdx::new(out as u8).unwrap())
    }
    pub fn peek_bit(&self) -> bool {
        Self::filter(self.odd)
    }
    /// Produces th next bit of keystream
    ///
    /// Any nonzero input will be fed into the LFSR
    pub fn next_bit(&mut self, input: bool, is_encrypted: bool) -> bool {
        let out = Self::filter(self.odd as u32);
        let mut feed = out as u32 & is_encrypted as u32;
        feed ^= input as u32;
        feed ^= (self.odd & Self::LF_POLY_ODD) as u32;
        feed ^= (self.even & Self::LF_POLY_EVEN) as u32;

        self.even = (self.even << 1) | (!parity_u32(feed)) as u32;
        core::mem::swap(&mut self.even, &mut self.odd);

        out
    }
    pub fn next_byte(&mut self, input: u8, is_encrypted: bool) -> u8 {
        let mut out = 0;
        input
            .view_bits::<Lsb0>()
            .into_iter()
            .zip(out.view_bits_mut::<Lsb0>())
            .for_each(|(i, mut o)| *o = self.next_bit(*i, is_encrypted));
        out
    }
    /// Encrypts the data while inserting encrypted parity
    ///
    /// Optionally feeds provided data into the cipher state
    ///
    /// Retuns Err(required bits) if the output buffer was too small, Ok(bits written)
    /// otherwise
    pub fn encrypt_with_parity(
        &mut self,
        input: &[u8],
        output: &mut BitSlice<u8, Lsb0>,
        cipher_input: Option<&[u8]>,
    ) -> Result<usize, usize> {
        let bits = input.len() * 9;
        if output.len() < bits {
            return Err(bits);
        }

        for ((i, o), k) in input
            .iter()
            .zip(output.chunks_exact_mut(9))
            .zip(cipher_input.into_iter().flatten().chain(repeat(&0u8)))
        {
            let enc = i ^ self.next_byte(*k, false);
            let par = parity_u8(*i);
            o[..8].store(enc);
            o.set(8, par ^ self.peek_bit());
        }

        Ok(bits)
    }

    /// Decrypts the provided data while checking for parity errors after each byte
    ///
    /// Returns
    /// - Ok((bytes, bits)) if there were no errors
    /// - `CryptoError::Capacity` if the provided output buffer is insufficient
    ///
    /// If an error is detected decryption continues and the error is returned afterwards
    pub fn decrypt_with_parity(
        &mut self,
        input: &BitSlice<u8, Lsb0>,
        output: &mut [u8],
    ) -> Result<(usize, u8), CryptoError> {
        let bytes = input.len().div_ceil(9);
        if output.len() < bytes {
            return Err(CryptoError::Capacity(bytes));
        }

        let mut parity_error = None;
        let iter = input.chunks_exact(9);
        let rem = iter.remainder();

        for (num, (i, o)) in iter.zip(output.into_iter()).enumerate() {
            let enc_byte: u8 = i[..8].load();
            let enc_par = i[8];

            let dec_byte = enc_byte ^ self.next_byte(0, false);
            let dec_parity = enc_par ^ self.peek_bit();
            *o = dec_byte;

            if dec_parity != parity_u8(dec_byte) {
                parity_error.get_or_insert(num);
            }
        }

        let mut rem_dec = 0u8;
        rem.iter()
            .zip(rem_dec.view_bits_mut::<Lsb0>())
            .for_each(|(e, mut d)| *d = *e ^ self.next_bit(false, false));

        let bits = rem.len();
        if let Some(idx) = parity_error {
            Err(CryptoError::Parity(idx))
        } else {
            Ok((bytes, bits as u8))
        }
    }
}

pub fn prng_successor(x: u32, steps: u32) -> u32 {
    let mut s = x.swap_bytes();
    for _ in 0..steps {
        s = (s >> 1) | (s >> 16 ^ s >> 18 ^ s >> 19 ^ s >> 21) << 31;
    }

    s.swap_bytes()
}

#[cfg(test)]
mod tests {
    extern crate std;
    use rand::{RngCore, SeedableRng};

    #[test]
    fn parity_fuzz() {
        let mut rng = rand::rngs::SmallRng::from_seed([0; 32]);
        for i in 0..1_000_000 {
            let r = rng.next_u32();
            assert_eq!(
                super::parity_u32(r),
                r.count_ones() % 2 != 0,
                "Test for {r:0b} failed after {i} iterations"
            )
        }
    }
}
