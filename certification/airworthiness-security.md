# Airworthiness Security (DO-326A / ED-202A)

> Status: **Draft scaffold.** This document maps the anti-spoofing,
> map-integrity, and authenticated-encryption code already implemented to
> the DO-326A/ED-202A airworthiness security process. It is **not**
> certification credit — a formal Security Risk Assessment and independent
> security verification, performed outside this repository, are required
> for compliance.

## Standard in brief

- **DO-326A** (RTCA) / **ED-202A** (EUROCAE) — *Airworthiness Security
  Process Specification*. Jointly harmonized standards defining the
  security-equivalent of the ARP 4754A/4761 safety lifecycle:
  **Security Risk Assessment** (identify threats and vulnerabilities,
  analogous to FHA) → **Security Development** (design and implement
  mitigations, analogous to PSSA) → **Security Verification** (independently
  confirm the mitigations are effective, analogous to SSA).
- Companion standards **DO-355/ED-204** (continued airworthiness security)
  and **DO-356A/ED-203A** (methods) apply once in-service; out of scope for
  this scaffold.

`spec.txt` §16.1 names both standards; §19 ("Security Model") is the
narrative source this mapping traces back to.

## TPT artifact mapping

| Process activity | Threat addressed | TPT artifact |
|---|---|---|
| Security Development — GNSS spoofing/jamming detection | Malicious or unintentional GNSS signal manipulation causing navigation-loss hazard | `tpt-protocols::antispoof::RaimMonitor` — multi-constellation solution comparison (RAIM) flags inconsistent GNSS fixes; wired to `Gnss::is_jammed_or_spoofed()` in `tpt-backend-bare-metal`. |
| Security Development — GNSS message authentication | Spoofed GNSS position/velocity injected as genuine | `tpt-protocols::antispoof::GnssAuth` (`sign`/`verify`) and `GnssToken` — authenticates position/velocity/time-of-week against a shared key. |
| Security Development — map data integrity | Malicious or corrupted terrain/map data injected into TAN/SLAM | `tpt-protocols::integrity::MapManifest` / `build_manifest` / `verify` / `compute_root_hash` — cryptographically signs map tiles and verifies the signature before use. |
| Security Development — link confidentiality/authenticity | Tampering with or spoofing of MAVLink/TPT-Link telemetry and command traffic | `tpt-protocols::chacha` — `aead_encrypt`/`aead_decrypt`/`Poly1305` (ChaCha20-Poly1305 authenticated encryption), wired into `tptlink`/`mavlink` framing. |
| Common-Cause Analysis coordination | Security-induced common-mode failure (e.g. a single spoofed source defeating multiple "dissimilar" channels) | Cross-links to `certification/system-safety-assessment.md`'s CCA row — dissimilar nav sources (`tpt-sensor-fusion::dissimilar`) must be independently confirmed not to share a security vulnerability, not just a hardware/algorithm difference. |

## Open items before certification credit

- [ ] Formal Security Risk Assessment: enumerate threat actors, attack
      surfaces (RF links, companion-compute offload channel, ground-station
      link), and map each to a hazard severity — no such enumeration exists
      yet, only the mitigations themselves.
- [ ] Security Development artifact set (threat model, mitigation
      traceability) formalized beyond this mapping table.
- [ ] Independent Security Verification — penetration testing / formal
      review by a party other than the implementers.
- [ ] Reconcile with the ARP 4761 Common-Cause Analysis in
      `certification/system-safety-assessment.md` so security-induced
      common-mode failures are covered by *both* processes, not silently
      dropped between them.
