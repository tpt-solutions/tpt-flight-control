# Software Configuration Index (SCI)

> Status: **Draft scaffold.** Identifies the configuration items and
> baseline mechanism actually in use; it is **not** certification credit —
> DO-178C configuration management requires a controlled baseline and
> change process audited by the certification authority.

## Configuration items

The unit of configuration is the Cargo workspace member (crate). Current
crate list (see `Cargo.toml` workspace members and the crate map in
`CLAUDE.md`):

`tpt-abstractions`, `tpt-math`, `tpt-core`, `tpt-sensor-fusion`,
`tpt-mapping`, `tpt-mixer`, `tpt-backend-bare-metal`, `tpt-backend-freertos`,
`tpt-backend-zephyr`, `tpt-backend-pikeos`, `tpt-backend-sel4`,
`tpt-backend-vxworks`, `tpt-sovereign-toolchain`, `tpt-protocols`, `tpt-sim`,
`tpt-gcs`, `tpt-web`, `reference-hardware`, `certification`.

Each crate's `Cargo.toml` pins its own version; the workspace `Cargo.lock`
pins the exact resolved dependency graph.

## Baseline identification

- **Source baseline**: `git` commit hash / tag. No release tags exist yet
  (project is pre-Phase-1 in terms of physical deployment; see `todo.md`).
- **Toolchain baseline**: `rust-version = "1.85"`, `edition = "2024"`
  (pinned per `certification/ci-qualification.md`'s reproducibility
  controls).
- **Dependency baseline**: `Cargo.lock`, audited via
  `cargo deny --all-features check advisories licenses bans sources`
  (`deny.toml`, `scripts/audit.sh`, `scripts/vendor.sh`).
- **Generated artifact**: SBOM via the `cargo-cyclonedx` CI job (`sbom` job
  in `.github/workflows/ci.yml`). This is **CI-generated on each run, not
  committed to the repository** — there is no `sbom/bom.xml` checked into
  version control; a real baseline record would need to archive the
  CI-produced SBOM per release, which does not happen today.

## Change control

- All changes go through pull request review; `CODEOWNERS` requires
  domain-expert review on flight-critical crates
  (`tpt-core`, `tpt-sensor-fusion`, `tpt-mixer`, `tpt-mapping`,
  `tpt-abstractions`, `certification/`) and security-sensitive crates
  (`tpt-protocols`, `tpt-sovereign-toolchain`).
- DCO sign-off (`git commit -s`) is required on every commit
  (`CONTRIBUTING.md`); CI rejects unsigned commits.
- Changes touching a certified profile (`tpt-evtol`, `tpt-transport`)
  should include a matching `certification/` artifact update per
  `CLAUDE.md`.

## Open items

- [ ] Archive CI-generated SBOMs per release rather than regenerating them
      transiently — no persistent SBOM record exists today.
- [ ] Establish actual release tagging / baseline naming once a release
      process exists.
- [ ] Formal problem-reporting / change-request process tied to DO-178C
      change-impact analysis — currently just standard GitHub PR review,
      with no DO-178C-specific change-impact classification step.
