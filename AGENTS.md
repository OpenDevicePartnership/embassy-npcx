# AGENTS.md

Operational guide for AI coding agents working on **embassy-npcx**, the Embassy
HAL for the Nuvoton NPCX MCU family. Human contributors should read
[`CONTRIBUTING.md`](CONTRIBUTING.md) and [`README.md`](README.md) first; this
file captures the conventions and verification steps that an autonomous agent
needs in order to make safe changes.

## 1. Repository layout

This is a Cargo workspace with two members:

| Path | Purpose |
| --- | --- |
| `./` (root crate `embassy-npcx`) | The HAL library itself (`#![no_std]`). All peripheral drivers live under [`src/`](src/). |
| `./examples` (crate `npcx490m-examples`) | Runnable firmware examples for the NPCX498F (Ramon EVB). Built for `thumbv7em-none-eabihf`. |

Key files:

- [`Cargo.toml`](Cargo.toml) — crate metadata, feature flags, dependency pins.
- [`src/lib.rs`](src/lib.rs) — module declarations, `peripherals!` and
  `interrupt_mod!` invocations, `bind_interrupts!` macro, `init_lpc` /
  `init_espi` entry points, the bootloader header in the `rt` module.
- [`build.rs`](build.rs) — emits `link_flash.x` into `OUT_DIR`.
- [`link_flash.x`](link_flash.x) — replacement for `cortex-m-rt`'s `link.x`;
  required at link time (see §6).
- [`rust-toolchain.toml`](rust-toolchain.toml) — pins the target to
  `thumbv7em-none-eabihf` (Cortex-M4F).
- [`rustfmt.toml`](rustfmt.toml) — uses unstable options
  (`group_imports = StdExternalCrate`, `imports_granularity = Module`),
  so `cargo fmt` **must be run with nightly**.
- [`deny.toml`](deny.toml) — `cargo-deny` configuration.
- [`examples/.cargo/config.toml`](examples/.cargo/config.toml) — sets the
  examples build target, `probe-rs` runner, and link args.
- [`examples/npcx490m-chip.yaml`](examples/npcx490m-chip.yaml) — `probe-rs`
  chip description.

## 2. Target hardware and PAC

- Only chip currently supported: **NPCX490M family** (development on NPCX498F,
  Ramon EVB w/ 200-pin MECC). The single PAC dependency is
  [`npcx490m-pac`](https://github.com/OpenDevicePartnership/npcx490m-pac)
  re-exported as `embassy_npcx::pac`.
- Core: ARM Cortex-M4F. Build target: `thumbv7em-none-eabihf`.
- There is **no chip-selection feature flag**. If support for additional NPCX
  variants is added later, follow the embassy convention of one
  `chip-<name>` feature per chip with `#[cfg(feature = "chip-<name>")]` gates,
  and reflect any mutual exclusion in `Cargo.toml` and in this file.

## 3. Cargo features and mutual exclusion

Defined in [`Cargo.toml`](Cargo.toml):

| Feature | Default | Notes |
| --- | --- | --- |
| `rt` | yes | Enables `cortex-m-rt`, vector table setup, and the bootloader header in `src/lib.rs`. |
| `defmt` | no | Adds `defmt` logging derives on public types. |
| `time` | no | Adds `embassy-time` as a dependency for drivers that need it. |
| `_time-driver` | internal | Private; do not enable from downstream crates. Pulled in by any `time-driver-*` feature. |
| `time-driver-mft16-1` | no | Use MFT16-1 as the `embassy-time` driver. |
| `time-driver-mft16-2` | no | Use MFT16-2 as the `embassy-time` driver. |
| `time-driver-mft16-3` | no | Use MFT16-3 as the `embassy-time` driver. |

**Mutual exclusion rules** (the build will *not* fail, but behaviour is wrong
if you violate them):

- **At most one `time-driver-mft16-*` feature may be enabled.** The selector
  in [`src/time_driver.rs`](src/time_driver.rs) uses `cfg_if!` with `else if`
  chains, so enabling more than one silently selects the lowest-numbered
  timer and leaves the others marked as still-available peripherals — which
  will then race with the time driver. Document this in any new
  `time-driver-*` feature you add.
- Enabling a `time-driver-*` feature **removes** the matching `MFT16_n`
  entry from `peripherals!` (see the `#[cfg(not(feature = ...))]` guards at
  the bottom of `src/lib.rs`). When adding a new time driver, mirror this
  pattern.
- The init mode (LPC vs eSPI) is selected at **runtime** by calling either
  `init_lpc(config)` or `init_espi(config)`. The two modes are mutually
  exclusive and switching between them requires a **full power-cycle** of
  the chip — both functions `assert!` on `Sysconfig.devcnt.hif_typ_sel` to
  catch a re-init in the wrong mode. Never call both in the same firmware.
- Features must be **strictly additive** with respect to compilation — CI
  runs `cargo hack --feature-powerset check --locked`, so any new feature
  that breaks another combination's build will fail CI. If you must add a
  feature that conflicts with another at compile time, exclude the offending
  combination via `cargo-hack`'s `--mutually-exclusive-features` /
  `--skip` mechanism documented in
  [`.github/workflows/check.yml`](.github/workflows/check.yml).
- Underscore-prefixed features (`_time-driver`) are private; never enable
  them directly from `examples/` or downstream crates.

## 4. Driver conventions

The library follows embassy HAL conventions. When adding or modifying a
peripheral driver, keep these patterns consistent with the existing modules
([`i2c.rs`](src/i2c.rs), [`uart.rs`](src/uart.rs), [`spip.rs`](src/spip.rs),
[`miwu.rs`](src/miwu.rs), [`gpio.rs`](src/gpio.rs)):

- **Module documentation.** The crate is built with `#![warn(missing_docs)]`;
  every public item needs a doc comment. Module-level docs go at the top of
  the file using `//!`.
- **Peripheral typestate.** Drivers take a `Peri<'d, T>` from
  `embassy_hal_internal`. Use the existing `Instance` / sealed-trait
  pattern visible in `i2c.rs`. Bind interrupts through the crate's
  [`bind_interrupts!`](src/lib.rs) macro, never by hand.
- **`InterruptHandler<T>` structs.** Each peripheral that uses interrupts
  exposes a public `InterruptHandler<T>` and implements
  `crate::interrupt::typelevel::Handler<T::Interrupt>` for it. The handler
  must:
  1. Wake the per-instance `AtomicWaker` (`T::waker().wake()`).
  2. Disable or mask the level-triggered interrupt source before returning
     (otherwise the handler will re-fire immediately). The `i2c` driver
     does this via `inten().clear_bit()` and a `remediation` `AtomicU32`.
  3. Do the minimum work needed to satisfy the hardware; defer state
     processing to the async task that the waker re-polls.
- **Cancellation safety.** Any future returned from a public async API must
  be cancel-safe: dropping the future must leave the peripheral in a
  defined state. Use the `cancellation` module
  ([`src/cancellation.rs`](src/cancellation.rs)) and `on-drop` guards
  rather than relying on the caller to await completion.
- **`defmt`.** Gate `defmt::Format` derives behind
  `#[cfg_attr(feature = "defmt", derive(defmt::Format))]`, as
  [`src/uart.rs`](src/uart.rs) does. Never make `defmt` a hard dependency
  of a public type.
- **`Config` structs.** Mark them `#[non_exhaustive]` and provide a
  `Default` impl so that new fields can be added without breaking
  downstream code.
- **`pac` access.** Prefer `T::regs()` accessors on the `Instance` trait
  over `unsafe { pac::Foo::steal() }`. The `init_lpc` / `init_espi`
  functions in `src/lib.rs` are the only places that legitimately call
  `steal()` on shared system blocks.

## 5. DMA

The NPCX490M exposes two GDMA controllers (`GDMA1`, `GDMA2`) and a set of
peripheral-attached MDMA channels (`CR_UART1_MDMA1` … `CR_UART4_MDMA4`,
`I3C1_MDMA5`, `I3C2_MDMA6`, `I3C3_MDMA7`) — all listed in the
[`interrupt_mod!`](src/lib.rs) block. At time of writing **no driver uses
DMA**; the UART module explicitly notes `Does not (yet) support DMA
transactions`. When adding DMA support:

- Buffers passed to DMA must outlive the transfer. Take them as
  `&'a mut [u8]` and tie the returned future's lifetime to `'a`, or use
  the `cancellation` module's drop-guard pattern so that the channel is
  stopped synchronously on drop before the buffer is freed.
- Insert an `atomic::compiler_fence(Ordering::SeqCst)` (and on Cortex-M4F
  a `cortex_m::asm::dsb()` when crossing the MPU/DMA boundary) before
  starting a transfer and after it completes, to prevent the compiler
  from reordering loads/stores around the DMA window.
- Never hand out the `MDMA*` peripheral and the owning peripheral
  (`CR_UART*`, `I3C*`) to independent drivers. Either merge them into a
  single `Peri` tuple, or remove the MDMA from `peripherals!` when the
  parent peripheral takes ownership, the same way the time-driver
  features remove `MFT16_n`.

## 6. Linker script

Code is intended to run **from flash**, so the standard `cortex-m-rt`
`link.x` does not apply. Downstream crates must add
`-C link-arg=-Tlink_flash.x` to their rust flags
([`examples/.cargo/config.toml`](examples/.cargo/config.toml) shows the
canonical setup, including `-Tdefmt.x` and `--nmagic`). The script itself
is shipped through `build.rs`, which copies
[`link_flash.x`](link_flash.x) into `OUT_DIR` and emits the appropriate
`cargo:rustc-link-search` line.

## 7. Build, lint, test, and CI matrix

The MSRV is **Rust 1.87** (`rust-version` in `Cargo.toml`,
`msrv` matrix in [`.github/workflows/check.yml`](.github/workflows/check.yml)).

Run the **same commands CI runs** before pushing. All commands below are
executed from the repository root unless noted.

| Purpose | Command | Toolchain |
| --- | --- | --- |
| Formatting | `cargo +nightly fmt --check` | nightly (rustfmt uses unstable options) |
| Lints | `cargo clippy --locked -- -F clippy::suspicious -F clippy::correctness -F clippy::perf -D clippy::style` | stable & beta |
| Library check (host build) | `cargo check --locked` | stable |
| `no_std` target check | `cargo check --target thumbv8m.main-none-eabihf --no-default-features` | stable, target must be `rustup target add`-ed first |
| Examples | `cd examples && cargo check --locked` | stable, target `thumbv7em-none-eabihf` (set in `examples/.cargo/config.toml`) |
| Docs | `cargo doc --locked --no-deps --all-features` with `RUSTDOCFLAGS=--cfg docsrs` | nightly |
| Feature powerset | `cargo hack --feature-powerset check --locked` | stable (`cargo install cargo-hack`) |
| MSRV | `cargo +1.87 check --locked` | 1.87 |
| Supply chain | `cargo deny --all-features check` | stable (`cargo install cargo-deny`) |

There is **no test suite** (`cargo test` will currently build only doc
tests). Do not add a `cargo test` step to CI without first wiring up a host
or QEMU-based test harness — most code in this crate cannot link for the
host target.

### Notes about the CI configuration

- The `no-std` workflow targets `thumbv8m.main-none-eabihf`, while
  `rust-toolchain.toml` and the examples target `thumbv7em-none-eabihf`
  (the actual NPCX490M core). The `no-std` check is therefore a
  *portability* check, not a hardware build. Keep new code architecture
  -agnostic so that this check continues to pass.
- The `clippy` job uses [`giraffate/clippy-action`](https://github.com/giraffate/clippy-action)
  and posts annotations on PRs. Style lints are denied (`-D clippy::style`),
  so do not introduce style warnings to the codebase.
- The `commit_list` job re-runs each check **per commit** in the PR.
  Keep history clean: every commit must independently pass fmt, clippy,
  examples, docs, hack, deny, and the MSRV build. Either keep commits
  atomic and green, or squash before merge.

## 8. Git, branching, and commits

- The default branch is `main`. Fork-based PRs are the norm.
- Commit messages follow standard Git conventions; prefer a short
  imperative subject followed by a body explaining *why*. The repo does
  not enforce Conventional Commits.
- When committing on behalf of a contributor, set author info **per
  commit** with `git -c user.name=... -c user.email=... commit ...` —
  do not change the global git identity on a shared machine.
- AI-assisted commits should include an `Assisted-by:` trailer naming the
  model that helped produce them, for example:
  `Assisted-by: GitHub Copilot:<model-id>`.
- **Never force-push** to a shared branch (`main`, release branches, or
  any branch with an open PR). Force-push only to private topic branches
  on your own fork.

## 9. Things to avoid

- Adding a new public peripheral without `#![warn(missing_docs)]`-clean
  documentation — `cargo doc` will fail the CI doc job.
- Calling `pac::Foo::steal()` outside the `init_*` entry points.
- Enabling more than one `time-driver-mft16-*` feature in a single build.
- Re-using a `MFT16_n` peripheral while its matching time-driver feature
  is enabled.
- Holding a critical section across an `.await`. Critical sections in
  this crate are bounded blocks inside synchronous helpers (`i2c.rs`
  is the reference).
- Globally configuring `git` user identity inside an automated workflow.
- Force-pushing to `main` or to branches with open PRs.

## 10. Quick checklist before pushing

1. `cargo +nightly fmt --check`
2. `cargo clippy --locked -- -F clippy::suspicious -F clippy::correctness -F clippy::perf -D clippy::style`
3. `cargo check --locked`
4. `(cd examples && cargo check --locked)`
5. `cargo hack --feature-powerset check --locked` (if you touched features)
6. `cargo doc --locked --no-deps --all-features` (if you touched public APIs or docs)
7. Verify each commit passes the same checks (the CI `commit_list` job
   will).

## Model selection & cost discipline

Premium models (Opus, GPT-5 family, "high"/"xhigh" reasoning variants)
cost an order of magnitude more than standard models (Sonnet, Haiku,
mini). Most steps in a typical task do not need premium reasoning,
and over-using premium models wastes credits without improving
outcomes. The rules below apply to *all* model selection: your own
session, sub-agents launched via the `task` tool, and parallel work
launched via `/fleet`.

### Default posture

- **Default to the cheapest model that can do the job.** Reach for a
  premium model only when one of the escalation triggers below is hit.
- **Plan with premium, execute with cheap.** Spend at most one or two
  premium turns on design / planning, then downshift to a cheaper
  model for mechanical execution of the plan.
- **Never bump the model "just in case."** If you cannot articulate
  *why* a cheaper model would fail, use the cheaper model.

### Escalation triggers (use a premium model)

Reach for a premium model when *any* of these are true:

- Cross-module refactor, architectural design, or API design from
  scratch.
- Subtle correctness reasoning: concurrency, lifetimes, `unsafe`,
  FFI ABI, cryptography, safety-critical control paths.
- Debugging a failure that survived one prior cheap-model attempt.
- Reviewing code on a safety-, security-, or money-critical path.
- The diff cannot be predicted in advance — i.e. there is genuine
  creative or design work to do, not just typing.

### De-escalation triggers (use a cheap model)

Use the cheapest available model when *any* of these are true:

- Searching, reading, summarising files or docs.
- Single-file mechanical edits: rename, format, lint fix, dependency
  bump, boilerplate, scaffolding from a known template.
- Generating tests for code that already works.
- Running builds, tests, linters, or other commands where the model
  only needs to report success/failure.
- Routine commits, PR descriptions, changelog entries.
- The diff is essentially predictable before generation.

### Sub-agent routing (the `task` tool)

When delegating with the `task` tool, set `model:` explicitly. Do not
let sub-agents inherit a premium default for cheap work.

| Sub-agent type    | Default model             | Override to                                     |
|-------------------|---------------------------|-------------------------------------------------|
| `explore`         | cheap                     | keep cheap (`claude-haiku-4.5` or `gpt-5-mini`) |
| `task` (run cmd)  | cheap                     | keep cheap                                      |
| `research`        | cheap for breadth         | premium only for the final synthesis            |
| `general-purpose` | match task                | cheap for mechanical work; premium for design   |
| `rubber-duck`     | premium                   | keep premium — this is where reasoning pays off |
| `code-review`     | premium on critical paths | cheap on cosmetic / mechanical diffs            |

### `/fleet` (parallel sub-agents) rules

- Fleet mode multiplies cost by the fleet width. Apply the rules
  above *per worker*, not in aggregate.
- Split a fleet job along complexity lines: route the cheap,
  parallelisable workers (file edits, test runs, doc updates) to a
  cheap model; reserve premium models for the small number of
  workers that need real reasoning.
- If every worker in a fleet would need a premium model, the work is
  probably not a good fit for fleet mode — reconsider the
  decomposition before paying N× premium.

### Session hygiene

- Keep sessions short and focused. Long premium sessions are the
  single largest source of waste because every turn re-processes the
  full history.
- Use `/compact` when the conversation grows long, and `/new` for
  unrelated work.
- Prefer `/ask` for one-off side questions so they don't extend the
  main session.

### When in doubt

Ask: *"If a cheaper model produced the wrong answer here, would I
catch it in seconds (compiler, tests, my own review) or in
weeks (production incident)?"* If the former, use the cheap model
and let the feedback loop do its job.
