#include "plugin.hpp"

using namespace rack;
using namespace rack::componentlibrary;
using namespace rack::ui;

// =======================
//   MODULE DEFINITION
// =======================
//
// Single-lane genetic-algorithm variant of TheReelPeet. Playback (AR
// envelope via Rise/Fall) reuses the same mechanism as
// TheReelPeet::processLane, just for one lane, always reading from
// whichever phrase is currently active, always triggering normally each
// step - the Dynamics feature (held-gate/note-drop probability) this
// module briefly had was dropped when its knob was repurposed for Jitter.
//
// No internal BPM knob or free-running clock - the module is externally
// clocked via Clock In. Each incoming pulse advances one step; with
// nothing patched in, steps simply don't advance (same as any other
// clocked Eurorack sequencer). clockPeriod (the measured time between the
// last two pulses) stands in for the old BPM-derived per-step duration
// wherever one's needed, e.g. computing Phrase Duration.
//
// Phrase-cycling design: pool[4][16] holds up to 4 phrases, each with its
// own Active toggle and Entropy.
//
// Entropy is a single per-phrase knob spanning drift (mutation rate) and
// persist (replacement resistance) as one inverse-coupled axis: CW = more
// drift + less persist (changes readily), CCW = less drift + more persist
// (nearly frozen). drift = entropy, persist = 1 - entropy.
//
// Jitter is a single global knob that adds a small random offset to each
// phrase's effective Entropy, re-rolled every generation (see entropyOf) -
// so 4 phrases with identical Entropy settings still drift apart from each
// other over time instead of evolving in lockstep. At Jitter=0 (default)
// this has no effect.
//
// Phrase Duration isn't a knob at all - each phrase holds for a fixed
// number of full loops through the pattern (see phraseLoops below),
// derived from Steps and BPM, so it always feels proportionate to the
// tempo without needing its own control.
//
// Recall isn't a knob either - every playthrough has a fixed, small chance
// (kRecallChance, see selectPhrase) of playing a phrase's frozen genesis
// snapshot instead of its current evolved content. Recall is deliberately
// always "Sometimes," never user-adjustable Never/Always.
//
// Genesis (Run off->on transition) randomizes master[16] fresh, then
// derives every one of the 4 backing pool slots from it (drift-gated
// per-step variation, using each phrase's own Entropy) — this is the one
// point where all phrases share common DNA. Flipping an individual
// phrase's Active toggle from off->on re-seeds just that phrase from the
// current master (not a fresh one), so phrases activated later still
// share the same lineage as those already running.
//
// Playback cycles through whichever phrases are Active, each held for its
// own Phrase Duration. Once every active phrase has played once (one full
// round), each active phrase evolves independently: crossover with a
// random other active phrase, drift-gated mutation, persist-gated
// replacement (its own Entropy, not a shared one).
//
// Each phrase also has a Drift light: it flashes red the instant that
// phrase's pool actually gets replaced (persist check passing), then
// decays like a hardware peak/clip LED (see mutationFlash).

struct TheReelPeetEvo : Module {
  enum ParamId {
    RUN_PARAM,
    LENGTH_PARAM,
    JITTER_PARAM,   // was DYNAMICS_PARAM - Dynamics dropped, this knob repurposed to drive Jitter instead
    RISE_PARAM,
    FALL_PARAM,
    ACTIVE_PARAM,                        // ACTIVE_PARAM + 0..3
    ENTROPY_PARAM = ACTIVE_PARAM + 4,    // ENTROPY_PARAM + 0..3
    PARAMS_LEN = ENTROPY_PARAM + 4
  };
  enum InputId { CLOCK_INPUT, INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, ENV_OUTPUT, OUTPUTS_LEN };
  enum LightId {
    RUN_LIGHT,
    ACTIVE_LIGHT,                      // ACTIVE_LIGHT + 0..3
    RECALL_LIGHT = ACTIVE_LIGHT + 4,   // RECALL_LIGHT + 0..3
    DRIFT_LIGHT = RECALL_LIGHT + 4,    // DRIFT_LIGHT + 0..3
    LIGHTS_LEN = DRIFT_LIGHT + 4
  };
  enum EnvPhase { ENV_IDLE, ENV_ATTACK, ENV_DECAY };

  bool running = false;
  int step = 0;
  float trigTimer = 0.f;
  float heldCV = 0.f;
  float envLevel = 0.f;
  int envPhase = ENV_IDLE;
  int len = 8;

  float master[16];
  float pool[4][16];
  float poolGenesis[4][16];  // frozen snapshot of each phrase at genesis/reseed time, for Recall
  bool phraseActive[4] = {true, true, true, true};
  float blinkPhase = 0.f;  // drives the currently-playing light's pulse; independent of Phrase Duration
  int currentPhraseIdx = 0;
  bool usingGenesisNow = false;  // this playthrough's Recall roll for currentPhraseIdx
  float phraseTimer = 0.f;
  int phrasesPlayedThisRound = 0;
  float mutationFlash[4] = {0.f, 0.f, 0.f, 0.f};  // decays after a phrase's pool actually gets replaced
  float clockPeriod = 0.5f;    // measured time between the last two Clock In pulses, drives Phrase Duration
  float timeSinceClock = 0.f;  // time since the last Clock In pulse; becomes the next clockPeriod

  dsp::SchmittTrigger onTrig;
  dsp::SchmittTrigger activeTrig[4];
  dsp::SchmittTrigger clockTrig;

  TheReelPeetEvo() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

    configParam(RUN_PARAM, 0.f, 1.f, 0.f, "Run toggle");
    configParam(LENGTH_PARAM, 2.f, 16.f, 8.f, "Length (2-16 steps)");
    configParam(JITTER_PARAM, 0.f, 1.f, 0.f,
                "Jitter (randomizes each phrase's effective Entropy per generation)",
                "%", 0.f, 100.f);
    configParam(RISE_PARAM, 0.f, 2.f, 0.f, "Rise time", " s");
    configParam(FALL_PARAM, 0.f, 4.f, 0.5f, "Fall time", " s");

    for (int p = 0; p < 4; p++) {
      std::string n = std::to_string(p + 1);
      configParam(ACTIVE_PARAM + p, 0.f, 1.f, 0.f, "Phrase " + n + " active toggle");
      configParam(ENTROPY_PARAM + p, 0.f, 1.f, 0.25f,
                  "Phrase " + n + " entropy (CW: more drift/less persist, CCW: less drift/more persist)",
                  "%", 0.f, 100.f);
      configLight(ACTIVE_LIGHT + p, "Phrase " + n + " active");
      configLight(RECALL_LIGHT + p, "Phrase " + n + " recall active");
      configLight(DRIFT_LIGHT + p, "Phrase " + n + " drift active");
    }

    configInput(CLOCK_INPUT, "Clock");
    configOutput(OUT_OUTPUT, "Pitch CV (1V/Oct)");
    configOutput(ENV_OUTPUT, "Envelope CV (0-10V)");
    configLight(RUN_LIGHT, "Running");

    genesis();
  }

  // Genesis: fresh random master, then every one of the 4 backing pool
  // slots is derived from it (drift-gated per-step variation, using each
  // phrase's own drift) — this is the one point where all phrases share
  // common DNA.
  void genesis() {
    for (int i = 0; i < 16; i++)
      master[i] = random::uniform() * 5.f;
    for (int p = 0; p < 4; p++)
      reseedPhrase(p);

    int firstActive = 0;
    for (int p = 0; p < 4; p++) {
      if (phraseActive[p]) {
        firstActive = p;
        break;
      }
    }
    selectPhrase(firstActive);
    phraseTimer = 0.f;
    phrasesPlayedThisRound = 0;
  }

  // Effective entropy for phrase p on this roll: the knob's set value plus
  // a random offset scaled by the global Jitter knob (max +-0.3 at full
  // CW), re-rolled every call. This is what keeps 4 phrases with identical
  // Entropy settings from evolving in lockstep, without any per-phrase
  // Jitter knob. At Jitter=0 this is exactly the knob's value, unchanged.
  float entropyOf(int p) {
    float base = params[ENTROPY_PARAM + p].getValue();
    float jitterAmt = params[JITTER_PARAM].getValue() * 0.3f;
    float delta = (random::uniform() * 2.f - 1.f) * jitterAmt;
    return clamp(base + delta, 0.f, 1.f);
  }

  // Re-derives a single phrase from the current master (not a fresh one),
  // gated by that phrase's own entropy (drift half). Used both by genesis()
  // and when a phrase's Active toggle goes off->on, so phrases activated
  // later still share the same lineage as those already playing. The
  // result is also saved as that phrase's Recall snapshot.
  void reseedPhrase(int p) {
    float driftP = entropyOf(p);
    for (int i = 0; i < 16; i++) {
      pool[p][i] = (random::uniform() < driftP) ? random::uniform() * 5.f : master[i];
      poolGenesis[p][i] = pool[p][i];
    }
  }

  // Makes idx the currently-playing phrase and rolls its Recall check for
  // this playthrough: a fixed chance (kRecallChance, not user-adjustable -
  // Recall is deliberately always "Sometimes") of picking the frozen
  // genesis snapshot instead of the current evolved content for this one
  // playthrough (evolution keeps happening in the background regardless).
  // Does NOT touch phraseTimer - callers that need a hard reset (genesis,
  // jumping because the current phrase went inactive) do that themselves;
  // the normal round-advance path preserves the remainder instead of
  // zeroing it, to avoid timing drift.
  static constexpr float kRecallChance = 0.15f;
  void selectPhrase(int idx) {
    currentPhraseIdx = idx;
    usingGenesisNow = random::uniform() < kRecallChance;
  }

  int findNextActivePhrase(int fromIdx) {
    for (int s = 1; s <= 4; s++) {
      int idx = (fromIdx + s) % 4;
      if (phraseActive[idx]) return idx;
    }
    return fromIdx;
  }

  // Fires once per full run-through of the active phrases. Each active
  // phrase evolves from ITS OWN current content (elitism) crossed with a
  // random other active phrase, gated by its own entropy: drift (mutation)
  // and persist (whether it actually gets replaced) are the same knob,
  // inversely coupled (drift = entropy, persist = 1 - entropy). Entropy is
  // rolled once per phrase per generation (via entropyOf) and reused for
  // both halves, so Jitter can't decouple drift/persist from each other
  // within a single generation - it only varies entropy generation to
  // generation. Candidates are computed from a snapshot of the pool before
  // any writes happen, so every phrase's crossover partner is last round's
  // content, not a partially-updated mix.
  void evolveAllPhrases() {
    int activeIdx[4];
    int n = 0;
    for (int p = 0; p < 4; p++)
      if (phraseActive[p]) activeIdx[n++] = p;
    if (n == 0) return;

    float entropyRoll[4];
    for (int k = 0; k < n; k++) {
      int p = activeIdx[k];
      entropyRoll[p] = entropyOf(p);
    }

    float candidates[4][16];
    for (int k = 0; k < n; k++) {
      int p = activeIdx[k];
      float driftP = entropyRoll[p];
      int partnerK = (int)(random::uniform() * n);
      if (partnerK >= n) partnerK = n - 1;
      int partner = activeIdx[partnerK];
      int split = 1 + (int)(random::uniform() * 14.f);  // 1..14
      for (int i = 0; i < 16; i++) {
        candidates[p][i] = (i < split) ? pool[p][i] : pool[partner][i];
        if (random::uniform() < driftP)
          candidates[p][i] = random::uniform() * 5.f;
      }
    }
    for (int k = 0; k < n; k++) {
      int p = activeIdx[k];
      float persistP = 1.f - entropyRoll[p];
      if (random::uniform() >= persistP) {
        for (int i = 0; i < 16; i++)
          pool[p][i] = candidates[p][i];
        mutationFlash[p] = 1.f;
      }
    }
  }

  void process(const ProcessArgs &args) override {
    len = clamp((int)std::round(params[LENGTH_PARAM].getValue()), 1, 16);

    bool wasRunning = running;
    if (onTrig.process(params[RUN_PARAM].getValue()))
      running = !running;
    bool justStarted = running && !wasRunning;

    for (int p = 0; p < 4; p++) {
      if (activeTrig[p].process(params[ACTIVE_PARAM + p].getValue())) {
        phraseActive[p] = !phraseActive[p];
        if (phraseActive[p])
          reseedPhrase(p);
      }
    }

    if (justStarted)
      genesis();

    if (!phraseActive[currentPhraseIdx]) {
      int next = findNextActivePhrase(currentPhraseIdx);
      if (next != currentPhraseIdx) {
        selectPhrase(next);
        phraseTimer = 0.f;
      }
    }

    float riseTime = params[RISE_PARAM].getValue();
    float fallTime = params[FALL_PARAM].getValue();

    float outCV = 0.f;

    if (trigTimer > 0.f) {
      trigTimer -= args.sampleTime;
      if (trigTimer < 0.f) trigTimer = 0.f;
    }

    // Clock is measured regardless of Run state (avoids a stuck-trigger
    // edge case if a pulse arrives while stopped), but only acted on below
    // while running. clockPeriod is the measured time between the last two
    // pulses - stands in for the old BPM-derived stepTime everywhere a
    // per-step duration is needed (there's no free-running internal clock
    // anymore; steps only advance on an actual incoming pulse).
    timeSinceClock += args.sampleTime;
    bool clockEdge = clockTrig.process(inputs[CLOCK_INPUT].getVoltage());
    if (clockEdge) {
      clockPeriod = timeSinceClock;
      timeSinceClock = 0.f;
    }

    if (running) {
      int activeCount = 0;
      for (int p = 0; p < 4; p++)
        if (phraseActive[p]) activeCount++;

      if (activeCount > 0) {
        // Phrase Duration is no longer a knob - each phrase holds for a
        // fixed number of full loops through the pattern, so it always
        // feels proportionate to Length/Clock rather than needing its own
        // control. phraseLoops is a fixed constant, not user-adjustable.
        const float phraseLoops = 4.f;
        float phraseDuration = len * clockPeriod * phraseLoops;
        phraseTimer += args.sampleTime;
        if (phraseTimer >= phraseDuration) {
          phraseTimer -= phraseDuration;
          selectPhrase(findNextActivePhrase(currentPhraseIdx));
          phrasesPlayedThisRound++;
          if (phrasesPlayedThisRound >= activeCount) {
            phrasesPlayedThisRound = 0;
            evolveAllPhrases();
          }
        }
      }

      if (clockEdge) {
        step = (step + 1) % len;
        trigTimer = 0.f;

        if (envPhase == ENV_IDLE) {
          const float *playingSeq = usingGenesisNow ? poolGenesis[currentPhraseIdx] : pool[currentPhraseIdx];
          trigTimer = 0.01f;
          heldCV = playingSeq[step];
          envPhase = ENV_ATTACK;
        }
      }
      outCV = heldCV;
    } else {
      step = 0;
      trigTimer = 0.f;
      envLevel = 0.f;
      envPhase = ENV_IDLE;
      phraseTimer = 0.f;
      timeSinceClock = 0.f;
    }

    // Active-light brightness: off = deactivated, steady = active but
    // waiting its turn, pulsing = currently playing. A brightness climb
    // tied to Phrase Duration (up to 60s) turned out too slow to read at
    // a glance - a fixed-rate pulse (independent of Phrase Duration) is
    // far more perceptible on the hardware screen.
    const float blinkRateHz = 2.5f;
    blinkPhase += args.sampleTime * blinkRateHz;
    if (blinkPhase > 1.f) blinkPhase -= std::floor(blinkPhase);
    float pulse = 0.5f + 0.5f * std::sin(6.28318530718f * blinkPhase);

    for (int p = 0; p < 4; p++) {
      float brightness = 0.f;
      if (phraseActive[p]) {
        brightness = (p == currentPhraseIdx) ? (0.4f + 0.6f * pulse) : 0.4f;
      }
      lights[ACTIVE_LIGHT + p].setBrightness(brightness);
      // Recall light: only meaningful for whichever phrase is currently
      // playing, since usingGenesisNow is only rolled/valid for that one -
      // it's not a per-phrase memory of past rolls, just "is THIS
      // playthrough using the frozen snapshot right now."
      lights[RECALL_LIGHT + p].setBrightness((p == currentPhraseIdx && usingGenesisNow) ? 1.f : 0.f);

      // Drift light: flashes full-bright the instant evolveAllPhrases
      // actually replaces a phrase's pool, then decays like a hardware
      // peak/clip LED - a longer window than a typical LED flash, since
      // real mutations are infrequent enough (minutes apart at default
      // settings) that a brief flash was too easy to miss entirely.
      const float mutationDecayTime = 4.f;
      mutationFlash[p] -= args.sampleTime / mutationDecayTime;
      if (mutationFlash[p] < 0.f) mutationFlash[p] = 0.f;
      lights[DRIFT_LIGHT + p].setBrightness(mutationFlash[p]);
    }

    if (envPhase == ENV_ATTACK) {
      if (riseTime < 0.001f) {
        envLevel = 10.f;
      } else {
        envLevel += (10.f / riseTime) * args.sampleTime;
        envLevel = std::min(envLevel, 10.f);
      }
      if (envLevel >= 10.f)
        envPhase = ENV_DECAY;
    } else if (envPhase == ENV_DECAY) {
      if (fallTime < 0.001f) {
        envLevel = 0.f;
        envPhase = ENV_IDLE;
      } else {
        envLevel -= (10.f / fallTime) * args.sampleTime;
        if (envLevel <= 0.f) {
          envLevel = 0.f;
          envPhase = ENV_IDLE;
        }
      }
    }

    outputs[OUT_OUTPUT].setVoltage(outCV);
    outputs[ENV_OUTPUT].setVoltage(envLevel);
    lights[RUN_LIGHT].setBrightness(running ? 1.f : 0.f);
  }
};

// =======================
//   DISPLAY WIDGETS
// =======================

struct EvoLengthDisplay : TransparentWidget {
  int *value = nullptr;

  void draw(const DrawArgs &args) override {
    if (!value) return;
    nvgFontFaceId(args.vg, APP->window->uiFont->handle);
    nvgFillColor(args.vg, nvgRGB(0x00, 0x00, 0x00));
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

    char buf[4];
    snprintf(buf, sizeof(buf), "%d", *value);

    nvgFontSize(args.vg, 10.f);
    nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.35f, buf, nullptr);
    nvgFontSize(args.vg, 9.f);
    nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.65f, "Steps", nullptr);
  }
};

// Large stylistic background numeral (1-4) filling most of a pool box,
// drawn behind that pool's Active/Entropy/Recall/Drift controls (added to
// the widget tree before them, so they layer on top) rather than competing
// with them - replaces the small "1"/"2"/"3"/"4" label that used to sit
// above the Active button. Filled with a solid, opaque pastel tint of that
// pool's accent color (see poolColors below) - no outline currently.
struct EvoPoolNumeral : TransparentWidget {
  std::string text;
  NVGcolor color;

  void draw(const DrawArgs &args) override {
    nvgFontFaceId(args.vg, APP->window->uiFont->handle);
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    float fontSize = box.size.y * 0.72f;
    nvgFontSize(args.vg, fontSize);
    float cx = box.size.x * 0.5f;
    float cy = box.size.y * 0.5f;

    // Outline temporarily disabled to preview the numeral without it -
    // re-enable this block to restore the light-gray stamped outline.
#if 0
    const float outlineWidth = fontSize * 0.02f;
    const int steps = 16;
    nvgFillColor(args.vg, nvgRGB(0xaa, 0xaa, 0xaa));
    for (int i = 0; i < steps; i++) {
      float angle = 6.28318530718f * (float)i / (float)steps;
      float ox = cx + std::cos(angle) * outlineWidth;
      float oy = cy + std::sin(angle) * outlineWidth;
      nvgText(args.vg, ox, oy, text.c_str(), nullptr);
    }
#endif

    nvgFillColor(args.vg, color);
    nvgText(args.vg, cx, cy, text.c_str(), nullptr);
  }
};

// Rack's SVG panel loader (nanosvg) doesn't parse <text> elements at all —
// only path/rect/circle/line/polygon/g. So every static panel label has to
// be drawn here in C++, not baked into the SVG as <text>. (TheReelPeet.svg
// works around this by hand-tracing its wordmark/labels into <path> data;
// this module just draws them at runtime instead, same idea as TheReelPeet's
// existing "Dyn" StaticLabel widget.)
struct EvoStaticLabel : TransparentWidget {
  std::string text;
  float fontSize = 9.f;
  bool bold = false;

  void draw(const DrawArgs &args) override {
    nvgFontFaceId(args.vg, APP->window->uiFont->handle);
    nvgFillColor(args.vg, nvgRGB(0x28, 0x0b, 0x0b));
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgFontSize(args.vg, fontSize);
    if (bold)
      nvgFontBlur(args.vg, 0.15f);
    nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.5f, text.c_str(), nullptr);
  }
};

// =======================
//   WIDGET LAYOUT
// =======================
//
// Coordinates below (mm, fed through mm2px) were chosen to match the
// label positions baked into res/TheReelPeetEvo.svg — see the comment
// there for the conversion (1mm = 2.9528px).

// -O3 was confirmed (via -S assembly inspection) to miscompile this
// constructor: several string-literal args to the addLabel lambda
// ("Rise"/"Fall"/"1v/O"/"Gate"/"Phrase L") were silently dropped, while
// others survived — reproducible, and gone entirely at -O0. Scoped
// optimize-off here since this constructor only runs once at patch load,
// not at audio rate, so there's no performance cost to disabling it.
#pragma clang optimize off
struct TheReelPeetEvoWidget : ModuleWidget {
  TheReelPeetEvoWidget(TheReelPeetEvo *module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/TheReelPeetEvo.svg")));

    addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    // Left lane = shared/global playback controls. Panel narrowed (280->258
    // raw), trimming excess left/right margin while keeping a small gap on
    // both sides - laneXL shifted from 14 to match the lane's new raw
    // x=7 position (was x=14).
    const float laneXL = 11.629f;
    const float cvDX   = 4.5f;

    const float runY      = 15.5f;
    const float lengthY   = 33.f;
    const float dispY     = 37.f;
    // BPM knob is gone - the module is externally clocked now (see Clock
    // In below) - so Jitter moved up into BPM's old row, and Rise/Fall
    // moved up to close the gap that used to hold Jitter's own row.
    const float jitterY   = 54.f;
    const float riseFallY = 70.f;
    // The CV row (outputs + new Clock In) stays anchored near the bottom
    // rather than following Jitter/Rise/Fall up, leaving open space above it.
    const float outY      = 102.f;
    const float clockInY  = 113.f;

    // 4 phrase pools on the right, arranged as a 2x2 grid (pool1 top-left,
    // pool2 top-right, pool3 bottom-left, pool4 bottom-right) rather than
    // the old 4-across layout. Columns are horizontal design values (fed
    // through mm2px), matching res/TheReelPeetEvo.svg's pool1-4 rects (raw
    // px / 2.9528, same conversion used throughout this file). Row 1's
    // Active/Entropy is aligned with Run in the global lane; row 2 stacks
    // directly below row 1's Recall/Drift rather than using half the panel
    // height each, since there's no longer per-pool display content that
    // needs the extra room (see the removed EvoMutationDisplay).
    // Within each pool box: Active+Entropy share a row (left/right
    // sub-columns), Recall+Drift share a row below in the same two
    // sub-columns (so Active/Recall align vertically, and Entropy/Drift
    // align vertically).
    const float poolColLeft[2] = {24.723f, 55.202f};      // left edge of each pool column (raw x=73, 163)
    const float rowAY[2] = {runY, runY + 32.f};           // Active+Entropy row, per pool-grid-row
    const float rowBY[2] = {rowAY[0] + 16.f, rowAY[1] + 16.f};  // Recall+Drift row, per pool-grid-row

    // Pool box bounds (matching res/TheReelPeetEvo.svg's pool1-4 rects,
    // raw px / 2.9528) for the background numerals below.
    const float poolRowTop[2] = {8.467f, 42.667f};  // top edge of each pool row (raw y=25, 126)
    const float poolBoxW = 29.80f;                  // raw width 88
    const float poolBoxH = 31.6f;                   // raw height ~93-94

    // Large background numerals, added before the per-phrase controls
    // below so they render behind the knobs/lights, not on top of them.
    // Solid, fully opaque pastel tints - each pool's accent hue (matching
    // res/TheReelPeetEvo.svg's pool1-4 fill colors) blended ~55% toward
    // white, not a transparency blend over the panel.
    NVGcolor poolColors[4] = {
      nvgRGB(0xee, 0xae, 0xae), nvgRGB(0x89, 0xa7, 0xe6),
      nvgRGB(0xee, 0xdd, 0xae), nvgRGB(0xd2, 0xb7, 0xee),
    };
    for (int p = 0; p < 4; p++) {
      auto *numeral = new EvoPoolNumeral;
      numeral->box.pos = mm2px(Vec(poolColLeft[p % 2], poolRowTop[p / 2]));
      numeral->box.size = mm2px(Vec(poolBoxW, poolBoxH));
      numeral->text = std::to_string(p + 1);
      numeral->color = poolColors[p];
      addChild(numeral);
    }

    addParam(createParamCentered<LEDButton>(mm2px(Vec(laneXL, runY)), module, TheReelPeetEvo::RUN_PARAM));
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(laneXL, runY)), module, TheReelPeetEvo::RUN_LIGHT));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, lengthY)), module, TheReelPeetEvo::LENGTH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, jitterY)), module, TheReelPeetEvo::JITTER_PARAM));

    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL - cvDX, riseFallY)), module, TheReelPeetEvo::RISE_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL + cvDX, riseFallY)), module, TheReelPeetEvo::FALL_PARAM));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneXL - cvDX, outY)), module, TheReelPeetEvo::OUT_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneXL + cvDX, outY)), module, TheReelPeetEvo::ENV_OUTPUT));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(laneXL, clockInY)), module, TheReelPeetEvo::CLOCK_INPUT));

    for (int p = 0; p < 4; p++) {
      float colLeft = poolColLeft[p % 2];
      int row       = p / 2;
      float ctrlX0  = colLeft + 7.f;   // left sub-column: Active, Recall
      float ctrlX1  = colLeft + 22.f;  // right sub-column: Entropy, Drift

      addParam(createParamCentered<LEDButton>(mm2px(Vec(ctrlX0, rowAY[row])), module, TheReelPeetEvo::ACTIVE_PARAM + p));
      addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(ctrlX0, rowAY[row])), module, TheReelPeetEvo::ACTIVE_LIGHT + p));

      addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(ctrlX1, rowAY[row])), module, TheReelPeetEvo::ENTROPY_PARAM + p));
      addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(ctrlX0, rowBY[row])), module, TheReelPeetEvo::RECALL_LIGHT + p));
      addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(ctrlX1, rowBY[row])), module, TheReelPeetEvo::DRIFT_LIGHT + p));
    }

    if (module) {
      const float dispW = 12.f;
      const float dispW2 = 8.f;

      auto addLabel = [&](const std::string &text, float x, float y, float w,
                           float fontSize, bool bold = false) {
        auto *label = new EvoStaticLabel;
        label->text = text;
        label->fontSize = fontSize;
        label->bold = bold;
        label->box.pos = mm2px(Vec(x - w * 0.5f, y));
        label->box.size = mm2px(Vec(w, 5.f));
        addChild(label);
      };

      // "THEREELPEET" wordmark is the real path art in the SVG, recentered
      // on the new panel width — see res/TheReelPeetEvo.svg.

      addLabel("Run", laneXL, runY + 4.f, dispW, 9.f);
      addLabel("Jitter", laneXL, jitterY + 6.f, dispW, 9.f);
      addLabel("Rise", laneXL - cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("Fall", laneXL + cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("1v/O", laneXL - cvDX, outY + 3.5f, dispW2, 8.f);
      addLabel("Gate", laneXL + cvDX, outY + 3.5f, dispW2, 8.f);
      addLabel("Clock", laneXL, clockInY + 3.5f, dispW, 8.f);

      for (int p = 0; p < 4; p++) {
        float colLeft = poolColLeft[p % 2];
        int row       = p / 2;
        float ctrlX0  = colLeft + 7.f;
        float ctrlX1  = colLeft + 22.f;

        addLabel("Entropy", ctrlX1, rowAY[row] + 6.f, dispW, 8.f);
        addLabel("Recall", ctrlX0, rowBY[row] + 2.5f, dispW, 7.f);
        addLabel("Drift", ctrlX1, rowBY[row] + 2.5f, dispW, 7.f);
      }

      auto *lenDisplay = new EvoLengthDisplay;
      lenDisplay->box.pos = mm2px(Vec(laneXL - dispW * 0.5f, dispY));
      lenDisplay->box.size = mm2px(Vec(dispW, 11.f));
      lenDisplay->value = &module->len;
      addChild(lenDisplay);

      // The evolution-cycle flowchart (EvoFlowDiagram) that used to fill
      // this space has been removed - reserved for a scale-quantizer
      // control instead. Open area: raw x=73-251, y=220-360
      // (poolColLeft[0] is that same raw x=73 left edge, in mm), i.e.
      // mm2px(Vec(poolColLeft[0], 74.51f)) sized mm2px(Vec(60.27f, 47.41f)).
    }
  }
};
#pragma clang optimize on

Model *modelTheReelPeetEvo =
    createModel<TheReelPeetEvo, TheReelPeetEvoWidget>("thereelpeet-evo");
