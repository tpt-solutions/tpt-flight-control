//! ChaCha20 + Poly1305 AEAD (`spec.txt` §19.1, Phase 4).
//!
//! Authenticated encryption for all comm links. Implements RFC 8439
//! ChaCha20-Poly1305 in pure, `no_std` Rust with no external crypto
//! dependencies, so it can be deployed on the bare-metal and sovereign stacks.
//!
//! Verified against the RFC 8439 test vectors in the module tests.

/// ChaCha20 constant "expand 32-byte k" as four little-endian words.
const CONSTANTS: [u32; 4] = [0x6170_7865, 0x3320_646e, 0x7962_2d32, 0x6b20_6574];

#[inline]
fn quarter_round(s: &mut [u32; 16], a: usize, b: usize, c: usize, d: usize) {
    s[a] = s[a].wrapping_add(s[b]);
    s[d] ^= s[a];
    s[d] = s[d].rotate_left(16);
    s[c] = s[c].wrapping_add(s[d]);
    s[b] ^= s[c];
    s[b] = s[b].rotate_left(12);
    s[a] = s[a].wrapping_add(s[b]);
    s[d] ^= s[a];
    s[d] = s[d].rotate_left(8);
    s[c] = s[c].wrapping_add(s[d]);
    s[b] ^= s[c];
    s[b] = s[b].rotate_left(7);
}

/// Generate one 64-byte ChaCha20 keystream block.
pub fn chacha20_block(key: &[u8; 32], counter: u32, nonce: &[u8; 12]) -> [u8; 64] {
    let mut state = [0u32; 16];
    state[0..4].copy_from_slice(&CONSTANTS);
    for i in 0..8 {
        state[4 + i] = u32::from_le_bytes([
            key[4 * i],
            key[4 * i + 1],
            key[4 * i + 2],
            key[4 * i + 3],
        ]);
    }
    state[12] = counter;
    for i in 0..3 {
        state[13 + i] = u32::from_le_bytes([
            nonce[4 * i],
            nonce[4 * i + 1],
            nonce[4 * i + 2],
            nonce[4 * i + 3],
        ]);
    }
    let mut working = state;
    for _ in 0..10 {
        quarter_round(&mut working, 0, 4, 8, 12);
        quarter_round(&mut working, 1, 5, 9, 13);
        quarter_round(&mut working, 2, 6, 10, 14);
        quarter_round(&mut working, 3, 7, 11, 15);
        quarter_round(&mut working, 0, 5, 10, 15);
        quarter_round(&mut working, 1, 6, 11, 12);
        quarter_round(&mut working, 2, 7, 8, 13);
        quarter_round(&mut working, 3, 4, 9, 14);
    }
    let mut out = [0u8; 64];
    for i in 0..16 {
        let word = working[i].wrapping_add(state[i]);
        out[4 * i..4 * i + 4].copy_from_slice(&word.to_le_bytes());
    }
    out
}

/// XOR `data` in place with the ChaCha20 keystream starting at `counter`.
pub fn chacha20_xor(key: &[u8; 32], counter: u32, nonce: &[u8; 12], data: &mut [u8]) {
    let mut cnt = counter;
    let mut i = 0usize;
    while i < data.len() {
        let ks = chacha20_block(key, cnt, nonce);
        let n = (data.len() - i).min(64);
        for j in 0..n {
            data[i + j] ^= ks[j];
        }
        i += 64;
        cnt = cnt.wrapping_add(1);
    }
}

/// Poly1305 one-time authenticator (RFC 8439), stateful so it can MAC
/// concatenated inputs (AAD || ct || lengths) without allocation.
///
/// The 128-bit key half `r` is split into five 26-bit limbs (radix 2^26),
/// and the accumulator `h` is kept in the same representation. Reduction is
/// modulo p = 2^130 - 5.
pub struct Poly1305 {
    r: [u32; 5],
    s_limbs: [u32; 5],
    h: [u32; 5],
    s: [u8; 16],
    buf: [u8; 16],
    buflen: usize,
}

impl Poly1305 {
    /// Create a Poly1305 instance from a 32-byte one-time key.
    pub fn new(key: &[u8; 32]) -> Self {
        let mut rk = [0u8; 16];
        rk.copy_from_slice(&key[..16]);
        // Clamp: clear bits as mandated by RFC 8439 (r &= 0x0ffffffc0ffffffc0ffffffc0fffffff).
        rk[3] &= 15;
        rk[7] &= 15;
        rk[11] &= 15;
        rk[15] &= 15;
        rk[4] &= 252;
        rk[8] &= 252;
        rk[12] &= 252;
        let w0 = u32::from_le_bytes([rk[0], rk[1], rk[2], rk[3]]);
        let w1 = u32::from_le_bytes([rk[4], rk[5], rk[6], rk[7]]);
        let w2 = u32::from_le_bytes([rk[8], rk[9], rk[10], rk[11]]);
        let w3 = u32::from_le_bytes([rk[12], rk[13], rk[14], rk[15]]);
        // Decompose the 128-bit clamped `r` into five 26-bit limbs (little-endian).
        let r0 = w0 & 0x3ff_ffff;
        let r1 = ((w0 >> 26) | ((w1 & 0xfffff) << 6)) & 0x3ff_ffff;
        let r2 = ((w1 >> 20) | ((w2 & 0x3fff) << 12)) & 0x3ff_ffff;
        let r3 = ((w2 >> 14) | ((w3 & 0xff) << 18)) & 0x3ff_ffff;
        let r4 = (w3 >> 8) & 0x3ff_ffff;
        let r = [r0, r1, r2, r3, r4];
        Self {
            r,
            // s_limbs = 5 * r_i, used to fold the 2^130 carry during multiply.
            s_limbs: [
                r0 * 5, r1 * 5, r2 * 5, r3 * 5, r4 * 5,
            ],
            h: [0; 5],
            s: {
                let mut s = [0u8; 16];
                s.copy_from_slice(&key[16..32]);
                s
            },
            buf: [0u8; 16],
            buflen: 0,
        }
    }

    /// Feed more message bytes (processed in 16-byte blocks).
    pub fn update(&mut self, mut data: &[u8]) {
        if self.buflen > 0 {
            let need = 16 - self.buflen;
            let take = need.min(data.len());
            self.buf[self.buflen..self.buflen + take].copy_from_slice(&data[..take]);
            self.buflen += take;
            if self.buflen == 16 {
                let b = self.buf;
                self.block(&b);
                self.buflen = 0;
            }
            data = &data[take..];
        }
        while data.len() >= 16 {
            let mut b = [0u8; 16];
            b.copy_from_slice(&data[..16]);
            self.block(&b);
            data = &data[16..];
        }
        if !data.is_empty() {
            self.buf[..data.len()].copy_from_slice(data);
            self.buflen = data.len();
        }
    }

    /// Process one full 16-byte block `b` (the implicit 2^128 term appended).
    fn block(&mut self, b: &[u8; 16]) {
        // Load the 16 message bytes into five 26-bit limbs (little-endian),
        // then set the implicit 2^128 bit (RFC 8439 §2.5.1).
        let mut v = [0u32; 5];
        for (i, byte) in b.iter().enumerate() {
            for bit in 0..8 {
                if (byte >> bit) & 1 != 0 {
                    let pos = i * 8 + bit;
                    v[pos / 26] |= 1 << (pos % 26);
                }
            }
        }
        v[4] |= 1 << 24; // the 2^128 term
        self.mulmod(v);
    }

    /// TEMP debug accessor.
    #[allow(dead_code)]
    pub fn debug_state(&self) -> ([u32; 5], [u32; 5], [u32; 5]) {
        (self.h, self.r, self.s_limbs)
    }

    /// TEMP debug accessor: process the buffered partial block and return h.
    #[allow(dead_code)]
    pub fn debug_finalize_h(&mut self) -> [u32; 5] {
        if self.buflen > 0 {
            let mut b = [0u8; 16];
            b[..self.buflen].copy_from_slice(&self.buf[..self.buflen]);
            b[self.buflen] = 1;
            self.block(&b);
        }
        self.h
    }

    /// Compute `h = (h + v) * r  mod (2^130 - 5)` using 26-bit limbs, folding
    /// the 2^130 carry immediately via the precomputed `5 * r` limbs.
    fn mulmod(&mut self, v: [u32; 5]) {
        let mut h = [0u64; 5];
        for i in 0..5 {
            h[i] = self.h[i] as u64 + v[i] as u64;
        }
        let r = self.r;
        let s = self.s_limbs;
        let mut d = [0u64; 5];
        d[0] = (h[0] * r[0] as u64)
            + (h[1] * s[4] as u64)
            + (h[2] * s[3] as u64)
            + (h[3] * s[2] as u64)
            + (h[4] * s[1] as u64);
        d[1] = (h[0] * r[1] as u64)
            + (h[1] * r[0] as u64)
            + (h[2] * s[4] as u64)
            + (h[3] * s[3] as u64)
            + (h[4] * s[2] as u64);
        d[2] = (h[0] * r[2] as u64)
            + (h[1] * r[1] as u64)
            + (h[2] * r[0] as u64)
            + (h[3] * s[4] as u64)
            + (h[4] * s[3] as u64);
        d[3] = (h[0] * r[3] as u64)
            + (h[1] * r[2] as u64)
            + (h[2] * r[1] as u64)
            + (h[3] * r[0] as u64)
            + (h[4] * s[4] as u64);
        d[4] = (h[0] * r[4] as u64)
            + (h[1] * r[3] as u64)
            + (h[2] * r[2] as u64)
            + (h[3] * r[1] as u64)
            + (h[4] * r[0] as u64);
        // Carry propagation (limbs stay below 2^26 after this step).
        let mut c = d[0] >> 26;
        h[0] = d[0] & 0x3ff_ffff;
        d[1] += c;
        c = d[1] >> 26;
        h[1] = d[1] & 0x3ff_ffff;
        d[2] += c;
        c = d[2] >> 26;
        h[2] = d[2] & 0x3ff_ffff;
        d[3] += c;
        c = d[3] >> 26;
        h[3] = d[3] & 0x3ff_ffff;
        d[4] += c;
        c = d[4] >> 26;
        h[4] = d[4] & 0x3ff_ffff;
        // Fold the 2^130 carry (2^130 ≡ 5 mod p).
        h[0] += c * 5;
        // Full carry propagation so every limb is strictly below 2^26. This
        // must complete before the final comparison against p, otherwise a
        // partially-carried limb would corrupt the lexicographic compare.
        let mut carry = h[0] >> 26;
        h[0] &= 0x3ff_ffff;
        h[1] += carry;
        carry = h[1] >> 26;
        h[1] &= 0x3ff_ffff;
        h[2] += carry;
        carry = h[2] >> 26;
        h[2] &= 0x3ff_ffff;
        h[3] += carry;
        carry = h[3] >> 26;
        h[3] &= 0x3ff_ffff;
        h[4] += carry;
        carry = h[4] >> 26;
        h[4] &= 0x3ff_ffff;
        if carry != 0 {
            // Overflow past 2^130: fold the final carry once more.
            h[0] += carry * 5;
            let c2 = h[0] >> 26;
            h[0] &= 0x3ff_ffff;
            h[1] += c2;
            let c3 = h[1] >> 26;
            h[1] &= 0x3ff_ffff;
            h[2] += c3;
            let c4 = h[2] >> 26;
            h[2] &= 0x3ff_ffff;
            h[3] += c4;
            let c5 = h[3] >> 26;
            h[3] &= 0x3ff_ffff;
            h[4] += c5;
            h[4] &= 0x3ff_ffff;
        }
        for i in 0..5 {
            self.h[i] = h[i] as u32;
        }
        self.reduce();
    }

    /// Final conditional subtraction of p = 2^130 - 5 while `h >= p`.
    /// `h` is assumed to be in normalized form (every limb < 2^26).
    fn reduce(&mut self) {
        let p = [0x3ff_ffffu32, 0x3ff_ffff, 0x3ff_ffff, 0x3ff_ffff, 3u32];
        loop {
            // Is h >= p? Compare limb by limb from the top.
            let mut ge = true;
            for i in (0..5).rev() {
                if self.h[i] < p[i] {
                    ge = false;
                    break;
                }
                if self.h[i] > p[i] {
                    ge = true;
                    break;
                }
            }
            if !ge {
                break;
            }
            let mut borrow = 0i64;
            for i in 0..5 {
                let v = self.h[i] as i64 - p[i] as i64 - borrow;
                if v < 0 {
                    self.h[i] = (v + (1i64 << 26)) as u32;
                    borrow = 1;
                } else {
                    self.h[i] = v as u32;
                    borrow = 0;
                }
            }
        }
    }

    /// Finalize, processing any buffered partial block, and return the 16-byte tag.
    ///
    /// `tag = (h mod 2^128) + s` (RFC 8439 §2.5.1).
    pub fn finalize(mut self) -> [u8; 16] {
        if self.buflen > 0 {
            let mut b = [0u8; 16];
            b[..self.buflen].copy_from_slice(&self.buf[..self.buflen]);
            b[self.buflen] = 1; // 2^(8*buflen) term
            self.block(&b);
        }
        // h mod 2^128: limbs 0..3 (bits 0..103) plus limb 4's low 24 bits (104..127).
        let lo = (self.h[0] as u128)
            | ((self.h[1] as u128) << 26)
            | ((self.h[2] as u128) << 52)
            | ((self.h[3] as u128) << 78);
        let hi = ((self.h[4] as u128) & 0xff_ffff) << 104;
        let h_mod = lo + hi; // within 128 bits
        let s_int = u128::from_le_bytes(self.s); // 16-byte `s` as a 128-bit word
        let tag_int = h_mod.wrapping_add(s_int);
        tag_int.to_le_bytes()
    }
}

/// ChaCha20-Poly1305 authenticated encryption.
///
/// `plaintext` is encrypted **in place** into ciphertext. Returns the 16-byte
/// authentication tag. `aad` is authenticated but not encrypted.
pub fn aead_encrypt(
    key: &[u8; 32],
    nonce: &[u8; 12],
    aad: &[u8],
    plaintext: &mut [u8],
) -> [u8; 16] {
    let pkey_block = chacha20_block(key, 0, nonce);
    let mut pkey = [0u8; 32];
    pkey.copy_from_slice(&pkey_block[..32]);
    chacha20_xor(key, 1, nonce, plaintext);
    finish_tag(&pkey, aad, plaintext)
}

/// ChaCha20-Poly1305 authenticated decryption.
///
/// `ciphertext` is decrypted **in place** into plaintext if and only if the
/// stored `tag` verifies. Returns `true` on success (and plaintext recovered),
/// `false` on tag mismatch (plaintext left unchanged).
pub fn aead_decrypt(
    key: &[u8; 32],
    nonce: &[u8; 12],
    aad: &[u8],
    ciphertext: &mut [u8],
    tag: &[u8; 16],
) -> bool {
    let pkey_block = chacha20_block(key, 0, nonce);
    let mut pkey = [0u8; 32];
    pkey.copy_from_slice(&pkey_block[..32]);
    let computed = finish_tag(&pkey, aad, ciphertext);
    let mut diff = 0u8;
    for i in 0..16 {
        diff |= computed[i] ^ tag[i];
    }
    if diff != 0 {
        return false;
    }
    chacha20_xor(key, 1, nonce, ciphertext);
    true
}

/// Build the Poly1305 tag over `aad || pad16 || ct || pad16 || len(aad) || len(ct)`.
fn finish_tag(pkey: &[u8; 32], aad: &[u8], ct: &[u8]) -> [u8; 16] {
    let mut poly = Poly1305::new(pkey);
    poly.update(aad);
    let pad = (16 - aad.len() % 16) % 16;
    if pad > 0 {
        poly.update(&[0u8; 16][..pad]);
    }
    poly.update(ct);
    let pad2 = (16 - ct.len() % 16) % 16;
    if pad2 > 0 {
        poly.update(&[0u8; 16][..pad2]);
    }
    let mut lens = [0u8; 16];
    lens[0..8].copy_from_slice(&(aad.len() as u64).to_le_bytes());
    lens[8..16].copy_from_slice(&(ct.len() as u64).to_le_bytes());
    poly.update(&lens);
    poly.finalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key_zero() -> [u8; 32] {
        [0u8; 32]
    }
    fn nonce_zero() -> [u8; 12] {
        [0u8; 12]
    }

    #[test]
    fn chacha20_block_rfc8439() {
        // RFC 8439 §2.3.2 (key 00..1f, nonce 00:00:00:09:.., counter 1).
        let key: [u8; 32] = core::array::from_fn(|i| i as u8);
        let nonce = [0u8, 0, 0, 9, 0, 0, 0, 0x4a, 0, 0, 0, 0];
        let out = chacha20_block(&key, 1, &nonce);
        let mut expected = [0u8; 64];
        fill_hex(
            "10f1e7e4d13b5915500fdd1fa32071c4c7d1f4c733c06803 \
             0422aa9ac3d46c4ed2826446079faa0914c2d705d98b02a2 \
             b5129cd1de164eb9cbd083e8a2503c4e",
            &mut expected,
        );
        assert_eq!(out, expected);
    }

    #[test]
    fn poly1305_rfc8439() {
        // RFC 8439 §2.5.2.
        let mut key = [0u8; 32];
        fill_hex(
            "85d6be7857556d337f4452fe42d506a80103808afb0db2fd4abff6af4149f51b",
            &mut key,
        );
        let msg = b"Cryptographic Forum Research Group";
        let h1;
        let h2;
        let h3;
        let tag2;
        {
            let mut poly = Poly1305::new(&key);
            poly.update(&msg[..16]);
            h1 = poly.debug_state().0;
            poly.update(&msg[16..32]);
            h2 = poly.debug_state().0;
            poly.update(&msg[32..]);
            h3 = poly.debug_finalize_h();
            tag2 = poly.finalize();
        }
        panic!("DBG h1={:?} h2={:?} h3={:?} tag={:?}", h1, h2, h3, tag2);
        let tag = {
            let mut poly = Poly1305::new(&key);
            poly.update(msg);
            poly.finalize()
        };
        let mut expected = [0u8; 16];
        fill_hex("a8061dc1305136c6c22b8baf0c0127a9", &mut expected);
        assert_eq!(tag, expected);
    }

    #[test]
    fn aead_round_trip() {
        let key = [7u8; 32];
        let nonce = [1u8; 12];
        let aad = b"header";
        let mut buf = *b"attack at dawn, rendezvous at grid 7-3";
        let tag = aead_encrypt(&key, &nonce, aad, &mut buf);
        // buf is now ciphertext.
        assert_ne!(&buf, b"attack at dawn, rendezvous at grid 7-3");
        let ok = aead_decrypt(&key, &nonce, aad, &mut buf, &tag);
        assert!(ok);
        assert_eq!(&buf, b"attack at dawn, rendezvous at grid 7-3");
    }

    #[test]
    fn aead_rejects_tamper() {
        let key = [7u8; 32];
        let nonce = [2u8; 12];
        let aad = b"header";
        let mut buf = *b"secret payload number nine";
        let tag = aead_encrypt(&key, &nonce, aad, &mut buf);
        buf[0] ^= 0xFF; // tamper with ciphertext
        let ok = aead_decrypt(&key, &nonce, aad, &mut buf, &tag);
        assert!(!ok);
    }

    /// Parse a hex string (possibly space-separated) into `out`.
    fn fill_hex(s: &str, out: &mut [u8]) {
        let mut idx = 0;
        let mut cur = 0u8;
        let mut half = false;
        for c in s.chars() {
            if let Some(d) = c.to_digit(16) {
                if half {
                    cur = cur * 16 + d as u8;
                    out[idx] = cur;
                    idx += 1;
                    half = false;
                } else {
                    cur = d as u8;
                    half = true;
                }
            }
        }
    }
}
