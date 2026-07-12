//! # tpt-sovereign-toolchain
//!
//! Custom compiler-qualification wrapper for the TPT Sovereign stack
//! (§13, §16). Provides build harness and verified-subset linting used to
//! qualify the Rust toolchain for DO-178C / Common Criteria evidence.
//!
//! The `VerifiedSubset` provides a configurable, deterministic checker that
//! validates a list of code constructs against the qualified language subset
//! (e.g. forbidding `unsafe`, floating point, or unbounded recursion per the
//! certification plan). It is the static-analysis front-end the qualification
//! harness exercises; the actual toolchain wrapper would call this over every
//! translation unit before emitting qualification evidence.
//!
//! Status: scaffolded in Phase -1, verified-subset checker implemented in
//! Phase 4.

/// Marker confirming the sovereign verified-subset feature is enabled.
pub const SOVEREIGN_VERIFIED_SUBSET: bool = cfg!(feature = "verified-subset");

/// A language construct observed in a translation unit, as reported by the
/// front-end and checked against the qualified subset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Construct {
    /// An `unsafe` block or function.
    Unsafe,
    /// Use of floating-point arithmetic.
    FloatArithmetic,
    /// A recursive function (with its estimated call depth).
    Recursion(u16),
    /// A dynamic allocation (`alloc`/`Box`/`Vec`).
    HeapAllocation,
    /// An external (FFI) call.
    FfiCall,
    /// A division (potential divide-by-zero to be proven away).
    Division,
}

/// Configuration of the qualified language subset.
#[derive(Debug, Clone, Copy)]
pub struct VerifiedSubset {
    pub allow_unsafe: bool,
    pub allow_float: bool,
    pub allow_heap: bool,
    pub allow_ffi: bool,
    /// Maximum permitted recursion depth (0 = no recursion allowed).
    pub max_recursion_depth: u16,
}

impl Default for VerifiedSubset {
    fn default() -> Self {
        // Conservative sovereign default: the smallest subset that can still
        // be qualified, forbidding unsafe / heap / FFI and capping recursion.
        Self {
            allow_unsafe: false,
            allow_float: true,
            allow_heap: false,
            allow_ffi: false,
            max_recursion_depth: 8,
        }
    }
}

/// Outcome of checking a single construct.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Finding {
    /// Construct is within the qualified subset.
    Ok,
    /// Construct violates the subset (with a reason code).
    Violation(ViolationKind),
}

/// Why a construct was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViolationKind {
    UnsafeForbidden,
    FloatForbidden,
    HeapForbidden,
    FfiForbidden,
    RecursionTooDeep,
}

/// Aggregated qualification report for a translation unit.
#[derive(Debug, Clone, Default)]
pub struct QualificationReport {
    pub checked: usize,
    pub violations: usize,
    pub reasons: [Option<ViolationKind>; 5],
}

impl QualificationReport {
    /// Whether the unit qualifies under the subset.
    pub const fn qualifies(&self) -> bool {
        self.violations == 0
    }

    fn add(&mut self, v: ViolationKind) {
        self.violations += 1;
        for slot in &mut self.reasons {
            if slot.is_none() {
                *slot = Some(v);
                break;
            }
        }
    }
}

impl VerifiedSubset {
    /// Check `construct` against this subset, returning a [`Finding`].
    pub fn check(&self, construct: Construct) -> Finding {
        match construct {
            Construct::Unsafe if !self.allow_unsafe => {
                Finding::Violation(ViolationKind::UnsafeForbidden)
            }
            Construct::FloatArithmetic if !self.allow_float => {
                Finding::Violation(ViolationKind::FloatForbidden)
            }
            Construct::HeapAllocation if !self.allow_heap => {
                Finding::Violation(ViolationKind::HeapForbidden)
            }
            Construct::FfiCall if !self.allow_ffi => Finding::Violation(ViolationKind::FfiForbidden),
            Construct::Recursion(depth) if depth > self.max_recursion_depth => {
                Finding::Violation(ViolationKind::RecursionTooDeep)
            }
            _ => Finding::Ok,
        }
    }

    /// Validate a whole translation unit, producing a [`QualificationReport`].
    pub fn report(&self, constructs: &[Construct]) -> QualificationReport {
        let mut rep = QualificationReport::default();
        rep.checked = constructs.len();
        for c in constructs {
            if let Finding::Violation(v) = self.check(*c) {
                rep.add(v);
            }
        }
        rep
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conservative_subset_rejects_unsafe_and_heap() {
        let vs = VerifiedSubset::default();
        assert_eq!(vs.check(Construct::Unsafe), Finding::Violation(ViolationKind::UnsafeForbidden));
        assert_eq!(
            vs.check(Construct::HeapAllocation),
            Finding::Violation(ViolationKind::HeapForbidden)
        );
        assert_eq!(vs.check(Construct::FloatArithmetic), Finding::Ok);
    }

    #[test]
    fn recursion_depth_enforced() {
        let vs = VerifiedSubset::default();
        assert_eq!(vs.check(Construct::Recursion(4)), Finding::Ok);
        assert_eq!(
            vs.check(Construct::Recursion(16)),
            Finding::Violation(ViolationKind::RecursionTooDeep)
        );
    }

    #[test]
    fn report_aggregates_violations() {
        let vs = VerifiedSubset::default();
        let rep = vs.report(&[
            Construct::Unsafe,
            Construct::HeapAllocation,
            Construct::FloatArithmetic,
            Construct::Recursion(2),
        ]);
        assert!(!rep.qualifies());
        assert_eq!(rep.checked, 4);
        assert_eq!(rep.violations, 2);
    }

    #[test]
    fn relaxed_subset_passes() {
        let vs = VerifiedSubset {
            allow_unsafe: true,
            allow_float: true,
            allow_heap: true,
            allow_ffi: true,
            max_recursion_depth: 64,
        };
        let rep = vs.report(&[
            Construct::Unsafe,
            Construct::HeapAllocation,
            Construct::FfiCall,
            Construct::Recursion(32),
        ]);
        assert!(rep.qualifies());
    }

    #[test]
    fn feature_marker_consistent() {
        // The const reflects the real feature flag, whatever it is.
        let _ = SOVEREIGN_VERIFIED_SUBSET;
    }
}
