# Copilot instructions for embassy-npcx

This repository is an Embassy HAL for the Nuvoton NPCX MCU family
(`#![no_std]`, Cortex-M4F, target `thumbv7em-none-eabihf`).

Read [`../AGENTS.md`](../AGENTS.md) before making changes. It is the
authoritative agent guide and covers:

- Repository layout and the workspace split between the root crate
  (`embassy-npcx`) and `./examples` (`npcx490m-examples`).
- Cargo features, including the mutual-exclusion rules for the
  `time-driver-mft16-*` features and the LPC vs eSPI init modes.
- Driver conventions: `Peri`/`Instance` typestate, `InterruptHandler<T>`,
  `bind_interrupts!`, cancellation safety, `defmt` gating.
- DMA buffer-lifetime and fence requirements (no driver uses DMA yet).
- The linker-script requirement (`link_flash.x`) for downstream firmware.
- The exact CI command matrix (fmt, clippy, check, examples, hack, deny,
  msrv, doc, no-std) — run the same commands locally before pushing.

When in doubt, mirror the patterns already used in
[`src/i2c.rs`](../src/i2c.rs), [`src/uart.rs`](../src/uart.rs), and
[`src/miwu.rs`](../src/miwu.rs).
