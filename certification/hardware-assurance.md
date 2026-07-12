# Hardware Design Assurance (DO-254)

> Status: **Not started.** Unlike the other documents in this directory,
> this is not a scaffold over existing evidence — there is currently no
> custom hardware design in this repository for DO-254 to apply to. This
> document exists to record that fact plainly, and to define the trigger
> for when it becomes a real gap rather than a name-check.

## Standard in brief

**DO-254** *Design Assurance Guidance for Airborne Electronic Hardware* is
the hardware-design counterpart to DO-178C: it governs custom/complex
electronic hardware (FPGAs, ASICs, and — depending on the certification
basis negotiated with the authority — custom circuit boards) with Hardware
Design Assurance Levels mirroring DO-178C's DAL A–D. A **PHAC** (Plan for
Hardware Aspects of Certification) is the hardware-side equivalent of the
PSAC.

DO-254 is not named in `spec.txt` §16.1's applicable-standards list. Given
`reference-hardware/`'s stated intent (open flight-computer designs), it
should be added there once real hardware design work begins.

## Current state

`reference-hardware/` is a placeholder crate: a `Board` enum
(`TptNucleus`/`TptSentinel`/`TptAether`) with an MCU-family lookup method,
and no schematic, layout, or other physical design source committed to the
repository. `README.md`'s repository-layout description currently calls
this directory "open flight-computer KiCad designs" / "PCBs" — that
describes the *intent*, not the current committed content; there is no
KiCad source in the tree today.

Because there is no custom hardware design yet, DO-254 has nothing to
apply to: the boards TPT currently targets (STM32, S32K3/TMS570, Zynq) are
COTS (commercial off-the-shelf) parts referenced by name, not
TPT-originated silicon or board designs requiring hardware design
assurance.

## What would trigger a real PHAC gap

The point at which this stops being "not applicable" and starts being an
actual open item:

- Real KiCad schematic/layout files land in `reference-hardware/` for a
  TPT-originated board (as opposed to referencing an existing COTS dev
  board by name).
- Any custom FPGA/ASIC logic is introduced (e.g. a custom sensor
  interface or redundancy-voting hardware block) — this is the case DO-254
  most directly governs.

At that point, this document should be replaced with an actual PHAC:
hardware design assurance level assignment, a hardware verification plan,
and — like every other document in this directory — a note that the final
DO-254 compliance credit requires an accredited hardware verification
process and, again, is not something a repository produces by itself.

## Open items

- [ ] Add DO-254 to `spec.txt` §16.1's applicable-standards list once
      custom hardware design work begins (not needed while
      `reference-hardware/` stays a COTS-board reference).
- [ ] Author a real PHAC when the trigger condition above is met.
