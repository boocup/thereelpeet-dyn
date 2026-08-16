# TheReelPeet-Evo

**TheReelPeet-Evo** is a single-lane genetic-algorithm sequencer. Instead of flat randomization, it holds **4 phrases** that evolve over time through elitism, crossover, and mutation — repetition with change, rather than noise.

Each phrase mutates according to a selectable **Model** (Gaussian, Markov, or Interval-locked), can occasionally recall its own original "genesis" version mid-performance, and treats rests and ties as genetic material alongside pitch, not a random roll at playback. A **Link** input lets two instances gently bias each other's pitch, for richer multi-voice patches.

TheReelPeet-Evo runs natively on both **VCV Rack 2** and the **4ms MetaModule** from a single codebase, sharing a plugin package with [TheReelPeet](./README.md).

TheReelPeet-Evo is licensed under the [MIT license](./LICENSE).

---

## Overview

The module holds **4 phrases**, each a 16-step sequence. All 4 share a common genetic lineage: on Run, a fresh "DNA" random walk (`master`) is generated, and every phrase is initially derived from it. Only whichever phrases are toggled **Active** take part in playback — each is held for a fixed number of loops before the next active phrase takes over.

That loop count is purely clock-driven — it counts elapsed clock edges through the step sequence, not notes actually played. If a long envelope skips a note's retrigger (see Rise/Fall below), the phrase-transition schedule doesn't wait for it or slip — it advances exactly on the same rhythm regardless of how many of those steps were actually audible.

Once every active phrase has played one full round, each active phrase evolves independently: it crosses over with a random other active phrase, mutates some of its steps, and — gated by its own **Entropy** — either keeps the result or discards it and tries again next round.

Rest and tie aren't rolled fresh each playthrough — they're baked into a phrase's genetic material the same way pitch is, so they cross over and mutate along with everything else. A tie is always immediately followed by a rest, capping a held note at one extra step rather than letting holds run indefinitely.

Each phrase also has a small, fixed chance (once it's actually diverged from its own frozen starting point) of **Recall** — playing that original version instead of its current evolved one for a single playthrough, then returning to the live, evolving version afterward.

Scale quantization happens only at the moment a note is actually played — the underlying genetics always evolve in continuous voltage space, so toggling quantizer notes on/off never disturbs the mutation/crossover math.

---

## Controls & I/O

### Global
- **Run** — starts/stops the sequencer. Green LED indicates running state.
- **Length** — steps per phrase (2–16), with a digital "Steps" readout.
- **Model** — cycling button + 3 lights selecting the mutation engine: **Gaussian**, **Markov**, or **Interval**. See [Mutation Engines](#mutation-engines-model) below.
- **Rise / Fall** — attack and decay time of the per-note envelope, both 0–4s. A new note never retriggers until the current envelope has fully finished (same gate TheReelPeet uses) — if Rise+Fall together outlast the gap between clock pulses, some steps may not audibly retrigger, though the step sequence and phrase timing keep advancing regardless (see Overview above).
- **Clock In** — external clock. Steps only advance on incoming pulses; with nothing patched, the sequence doesn't move.
- **Link** — optional CV input. When patched (typically from another instance's own 1V/Oct output), gently biases this module's generated pitches toward the incoming voltage. See [Link CV](#link-cv) below.
- **1V/Oct** — pitch CV output.
- **Envelope CV** — 0–10V AR envelope output.

### Per phrase (×4)
- **Active** — toggle + light. Enables or disables that phrase in the playback cycle; reactivating a phrase re-seeds it from the current shared DNA.
- **Entropy** — a single knob spanning drift and persist as one inverse-coupled axis: clockwise means more drift, less persistence, and more rests/ties; counterclockwise means the opposite, closer to frozen.
- **Recall light** — lit while that phrase is playing its original genesis version instead of its current evolved content.
- **Drift light** — flashes once that phrase's own turn to play begins, if its content changed since it last played (not the instant the change happens in the background).

### Quantizers (Left / Right)
Two independent 12-note toggle grids — Left governs phrases 1 & 3, Right governs phrases 2 & 4 — laid out like a piano keyboard on its side. Light up notes to build a scale directly; there's no separate key/root knob, since C is always the first toggle. Leave a side fully off to pass pitch through unquantized. If only one side has any notes active, that side governs all 4 phrases.

---

## Mutation Engines ("Model")

Selectable via the Model button, this changes *how* a phrase's pitch mutates when it deviates — rest/tie placement and everything else about the module stays the same regardless of choice.

- **Gaussian** (default) — a bounded random walk. Deltas are capped and reflect off the pitch range's edges instead of piling up against them.
- **Markov** — builds a live transition table from each phrase's *own current content* (no external data, no persistent state) and leans mutations toward whatever intervals that phrase already favors.
- **Interval** — mutates by whole scale-degree steps within that phrase's active quantizer notes, so drift stays diatonic-sounding even at high Entropy.

Markov and Interval both fall back to Gaussian behavior when there isn't enough information yet to use their own logic (no observed transitions, or no quantizer notes active).

---

## Link CV

Patch one module instance's **1V/Oct** output into another instance's **Link** input to gently pull the second module's generated pitches toward the first's current pitch — a partial nudge, not a hard override, so the two stay musically related without becoming unison copies of one another. It composes with everything else: same clock into both modules keeps them rhythmically locked, and matching quantizer scales keep them harmonically compatible — patch all three together for two voices that feel like part of the same idea rather than two coincidentally-aligned strangers.

---

## Intended Use

- Set a scale on one or both quantizers for melodic, in-key output rather than raw chromatic drift.
- Patch the Envelope CV into a VCA to hear rests and ties directly as volume shaping, not just pitch changes.
- Leave Entropy low on one or two phrases as a stable "anchor" while others drift more freely, for a mix of repetition and change within the same patch.
- Try each Model engine against the same phrase content — Gaussian for smooth wandering, Markov for content that leans into its own recent character, Interval for guaranteed in-scale drift.
- Run two instances with a shared clock, matching quantizer scales, and Link patched between them for a richer two-voice generative texture.

---

## Building

### VCV Rack 2

Requires a working [VCV Rack development environment](https://vcvrack.com/manual/PluginDevelopmentTutorial) with the Rack SDK at `../Rack`.

```bash
make
make install
```

### 4ms MetaModule

Requires the [MetaModule Plugin SDK](https://github.com/4ms/metamodule-plugin-sdk) and ARM GNU Toolchain 12.3.

```bash
cmake -B build-mm -DCMAKE_TOOLCHAIN_FILE=../MyModule-metamodule/sdk/cmake/arm-toolchain.cmake
cmake --build build-mm
```

Output: `build-mm/out/thereelpeet-seq.mmplugin`

Both commands above build the whole plugin package, which includes TheReelPeet-Evo alongside [TheReelPeet](./README.md) — there's no separate Evo-only build.
