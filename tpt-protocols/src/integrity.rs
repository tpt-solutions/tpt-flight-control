//! Map Data Integrity signing (`spec.txt` §19.1, Phase 4).
//!
//! Terrain/map databases are authenticated with a keyed hash so a corrupted or
//! forged database is rejected before it can influence navigation. A manifest
//! captures a Merkle-style root hash over the map tiles plus an HMAC-SHA256
//! signature over that root, issued by the (offline) signing authority.

use crate::sha256::{hmac_sha256, Sha256};

/// A signed map manifest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MapManifest {
    /// Root hash over all tiles (SHA-256 chain).
    pub root_hash: [u8; 32],
    /// HMAC-SHA256 tag over `root_hash` (the "signature").
    pub signature: [u8; 32],
    /// Row/column count of the DEM patch this manifest covers (diagnostic).
    pub tiles: u32,
}

/// Compute the Merkle root hash over a set of tile byte-slices.
///
/// The root is the SHA-256 of the concatenation of each tile's SHA-256, so any
/// single altered byte anywhere in any tile changes the root. (A full binary
/// Merkle tree is a straightforward extension; this chained form is enough to
/// detect tampering.)
pub fn compute_root_hash(tiles: &[&[u8]]) -> [u8; 32] {
    let mut acc = [0u8; 32];
    for tile in tiles {
        let h = Sha256::digest(tile);
        // Fold each tile hash into the running accumulator.
        for i in 0..32 {
            acc[i] ^= h[i];
        }
        acc = Sha256::digest(&acc);
    }
    acc
}

/// Sign a root hash with the signing key.
pub fn sign(root_hash: &[u8; 32], key: &[u8; 32]) -> [u8; 32] {
    hmac_sha256(key, root_hash)
}

/// Build a full manifest from tiles and a signing key.
pub fn build_manifest(tiles: &[&[u8]], key: &[u8; 32]) -> MapManifest {
    let root_hash = compute_root_hash(tiles);
    let signature = sign(&root_hash, key);
    MapManifest {
        root_hash,
        signature,
        tiles: tiles.len() as u32,
    }
}

/// Verify a manifest against the recomputed root hash and the signing key.
pub fn verify(manifest: &MapManifest, tiles: &[&[u8]], key: &[u8; 32]) -> bool {
    let root_hash = compute_root_hash(tiles);
    if root_hash != manifest.root_hash {
        return false;
    }
    let expected = sign(&root_hash, key);
    // Constant-time-ish comparison.
    let mut diff = 0u8;
    for i in 0..32 {
        diff |= expected[i] ^ manifest.signature[i];
    }
    diff == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manifest_verifies_for_good_tiles() {
        let tiles: [&[u8]; 3] = [b"tile-a", b"tile-b", b"tile-c"];
        let key = [1u8; 32];
        let m = build_manifest(&tiles, &key);
        assert!(verify(&m, &tiles, &key));
    }

    #[test]
    fn manifest_rejects_tampered_tile() {
        let tiles: [&[u8]; 3] = [b"tile-a", b"tile-b", b"tile-c"];
        let key = [1u8; 32];
        let m = build_manifest(&tiles, &key);
        let mut tampered: [&[u8]; 3] = [b"tile-a", b"TILE-B", b"tile-c"];
        assert!(!verify(&m, &tampered, &key));
        // Ensure we didn't mutate the original through the reference.
        tampered[1] = b"tile-b";
        assert!(verify(&m, &tampered, &key));
    }

    #[test]
    fn manifest_rejects_wrong_key() {
        let tiles: [&[u8]; 2] = [b"x", b"y"];
        let m = build_manifest(&tiles, &[2u8; 32]);
        assert!(!verify(&m, &tiles, &[9u8; 32]));
    }
}
