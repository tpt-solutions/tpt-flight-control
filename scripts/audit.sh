#!/usr/bin/env bash
# scripts/audit.sh — supply-chain audit for TPT Flight Control.
#
# Runs cargo-deny over the whole workspace. Requires `cargo-deny`:
#   cargo install cargo-deny --locked
#
# Exits non-zero on any advisory/license/source/bans violation so it can gate
# CI and pre-commit.
set -euo pipefail

cd "$(dirname "$0")/.."

if ! command -v cargo-deny >/dev/null 2>&1 && ! cargo deny --version >/dev/null 2>&1; then
    echo "cargo-deny not found; install with: cargo install cargo-deny --locked" >&2
    exit 1
fi

cargo deny --all-features check \
    --hide-inclusion-graph \
    advisories licenses bans sources
