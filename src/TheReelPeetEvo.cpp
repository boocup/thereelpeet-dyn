#include "plugin.hpp"

using namespace rack;
using namespace rack::componentlibrary;
using namespace rack::ui;

// =======================
//   MODULE DEFINITION
// =======================
//
// Single-lane genetic-algorithm variant of TheReelPeet. Playback (Dyn
// hold/drop probability, AR envelope via Rise/Fall) reuses the same
// mechanism as TheReelPeet::processLane, just for one lane, always
// reading from whichever phrase is currently active.
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
// Each phrase also has a Mutate display (see EvoMutationDisplay): the
// instant that phrase's pool actually gets replaced (persist check
// passing), it briefly overlays the old shape (faded) against the new one
// (bold) - a real before/after of the mutation, not just an on/off event
// light - then fades back to blank. A single 16-point line survives being
// squeezed into a narrow column far better than the discrete bar-graph
// this replaced, and only appearing during a real event means it's never
// showing stale detail to misread.

struct TheReelPeetEvo : Module {
  enum ParamId {
    RUN_PARAM,
    LENGTH_PARAM,
    BPM_PARAM,
    DYNAMICS_PARAM,
    RISE_PARAM,
    FALL_PARAM,
    JITTER_PARAM,
    ACTIVE_PARAM,                        // ACTIVE_PARAM + 0..3
    ENTROPY_PARAM = ACTIVE_PARAM + 4,    // ENTROPY_PARAM + 0..3
    PARAMS_LEN = ENTROPY_PARAM + 4
  };
  enum InputId { INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, ENV_OUTPUT, OUTPUTS_LEN };
  enum LightId {
    RUN_LIGHT,
    ACTIVE_LIGHT,                      // ACTIVE_LIGHT + 0..3
    RECALL_LIGHT = ACTIVE_LIGHT + 4,   // RECALL_LIGHT + 0..3
    LIGHTS_LEN = RECALL_LIGHT + 4
  };
  enum EnvPhase { ENV_IDLE, ENV_ATTACK, ENV_SUSTAIN, ENV_DECAY };

  bool running = false;
  int step = 0;
  float timer = 0.f;
  float trigTimer = 0.f;
  float holdTimer = 0.f;
  float heldCV = 0.f;
  bool stepMuted = false;
  float envLevel = 0.f;
  int envPhase = ENV_IDLE;
  int len = 8;

  float master[16];
  float pool[4][16];
  float poolGenesis[4][16];  // frozen snapshot of each phrase at genesis/reseed time, for Recall
  float mutationOldSnapshot[4][16];  // pool[p] just before its last mutation, for the before/after flash
  bool phraseActive[4] = {true, true, true, true};
  float blinkPhase = 0.f;  // drives the currently-playing light's pulse; independent of Phrase Duration
  int currentPhraseIdx = 0;
  bool usingGenesisNow = false;  // this playthrough's Recall roll for currentPhraseIdx
  float phraseTimer = 0.f;
  int phrasesPlayedThisRound = 0;
  float mutationFlash[4] = {0.f, 0.f, 0.f, 0.f};  // decays after a phrase's pool actually gets replaced

  dsp::SchmittTrigger onTrig;
  dsp::SchmittTrigger activeTrig[4];

  TheReelPeetEvo() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

    configParam(RUN_PARAM, 0.f, 1.f, 0.f, "Run toggle");
    configParam(LENGTH_PARAM, 2.f, 16.f, 8.f, "Length (2-16 steps)");
    configParam(BPM_PARAM, 1.f, 240.f, 120.f, "BPM", " BPM");
    configParam(DYNAMICS_PARAM, -1.f, 1.f, 0.f,
                "Dynamics. CW: held gates, CCW: note drops");
    configParam(RISE_PARAM, 0.f, 2.f, 0.f, "Rise time", " s");
    configParam(FALL_PARAM, 0.f, 4.f, 0.5f, "Fall time", " s");
    configParam(JITTER_PARAM, 0.f, 1.f, 0.f,
                "Jitter (randomizes each phrase's effective Entropy per generation)",
                "%", 0.f, 100.f);

    for (int p = 0; p < 4; p++) {
      std::string n = std::to_string(p + 1);
      configParam(ACTIVE_PARAM + p, 0.f, 1.f, 0.f, "Phrase " + n + " active toggle");
      configParam(ENTROPY_PARAM + p, 0.f, 1.f, 0.25f,
                  "Phrase " + n + " entropy (CW: more drift/less persist, CCW: less drift/more persist)",
                  "%", 0.f, 100.f);
      configLight(ACTIVE_LIGHT + p, "Phrase " + n + " active");
      configLight(RECALL_LIGHT + p, "Phrase " + n + " recall active");
    }

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
        for (int i = 0; i < 16; i++) {
          mutationOldSnapshot[p][i] = pool[p][i];
          pool[p][i] = candidates[p][i];
        }
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

    float bpm = clamp(params[BPM_PARAM].getValue(), 1.f, 240.f);
    const float stepTime = 60.f / bpm;
    float riseTime = params[RISE_PARAM].getValue();
    float fallTime = params[FALL_PARAM].getValue();
    float dynamics = params[DYNAMICS_PARAM].getValue();

    float outCV = 0.f;

    if (trigTimer > 0.f) {
      trigTimer -= args.sampleTime;
      if (trigTimer < 0.f) trigTimer = 0.f;
    }
    if (holdTimer > 0.f) {
      holdTimer -= args.sampleTime;
      if (holdTimer < 0.f) holdTimer = 0.f;
    }

    if (running) {
      int activeCount = 0;
      for (int p = 0; p < 4; p++)
        if (phraseActive[p]) activeCount++;

      if (activeCount > 0) {
        // Phrase Duration is no longer a knob - each phrase holds for a
        // fixed number of full loops through the pattern, so it always
        // feels proportionate to Length/BPM rather than needing its own
        // control. phraseLoops is a fixed constant, not user-adjustable.
        const float phraseLoops = 4.f;
        float phraseDuration = len * stepTime * phraseLoops;
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

      timer += args.sampleTime;
      if (timer >= stepTime) {
        timer -= stepTime;
        step = (step + 1) % len;

        stepMuted = false;
        trigTimer = 0.f;

        if (holdTimer <= 0.f && envPhase == ENV_IDLE) {
          const float *playingSeq = usingGenesisNow ? poolGenesis[currentPhraseIdx] : pool[currentPhraseIdx];
          const float dynVal = clamp(dynamics, -1.f, 1.f);
          const float dynCurved = dynVal * dynVal * std::abs(dynVal);
          if (dynVal > 0.f && random::uniform() < dynCurved) {
            float jitter = 1.f + (random::uniform() * 0.2f - 0.1f);
            holdTimer = riseTime + len * stepTime * jitter;
            heldCV = playingSeq[step];
            envPhase = ENV_ATTACK;
          } else if (dynVal < 0.f && random::uniform() < dynCurved) {
            stepMuted = true;
            envLevel = 0.f;
            envPhase = ENV_IDLE;
          } else {
            trigTimer = 0.01f;
            heldCV = playingSeq[step];
            envPhase = ENV_ATTACK;
          }
        }
      }
      outCV = stepMuted ? 0.f : heldCV;
    } else {
      timer = 0.f;
      step = 0;
      trigTimer = 0.f;
      holdTimer = 0.f;
      stepMuted = false;
      envLevel = 0.f;
      envPhase = ENV_IDLE;
      phraseTimer = 0.f;
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

      // Mutation flash: decays after evolveAllPhrases actually replaces a
      // phrase's pool (see mutationOldSnapshot). Drives EvoMutationDisplay's
      // opacity directly (read via pointer, not a Light) - a longer window
      // than the old single LED had, since real mutations are infrequent
      // enough (minutes apart at default settings) that a brief flash was
      // too easy to miss entirely.
      const float mutationDecayTime = 4.f;
      mutationFlash[p] -= args.sampleTime / mutationDecayTime;
      if (mutationFlash[p] < 0.f) mutationFlash[p] = 0.f;
    }

    if (envPhase == ENV_ATTACK) {
      if (riseTime < 0.001f) {
        envLevel = 10.f;
      } else {
        envLevel += (10.f / riseTime) * args.sampleTime;
        envLevel = std::min(envLevel, 10.f);
      }
      if (envLevel >= 10.f)
        envPhase = (holdTimer > 0.f) ? ENV_SUSTAIN : ENV_DECAY;
    } else if (envPhase == ENV_SUSTAIN) {
      envLevel = 10.f;
      if (holdTimer <= 0.f)
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

struct EvoBPMDisplay : TransparentWidget {
  Param *param = nullptr;

  void draw(const DrawArgs &args) override {
    if (!param) return;
    int bpm = (int)std::round(param->getValue());

    nvgFontFaceId(args.vg, APP->window->uiFont->handle);
    nvgFillColor(args.vg, nvgRGB(0x00, 0x00, 0x00));
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", bpm);

    nvgFontSize(args.vg, 9.f);
    nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.35f, buf, nullptr);
    nvgFontSize(args.vg, 8.f);
    nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.65f, "BPM", nullptr);
  }
};

// Flashes a phrase's before/after shape the instant it actually mutates
// (see evolveAllPhrases/mutationFlash), then fades back to blank. Invisible
// the rest of the time - a single 16-point line survives being squeezed
// into a ~12mm-wide column far better than discrete bars did (no per-bar
// gaps eating into visual weight), and only appearing during a real event
// means there's never stale fine detail sitting on screen to misread.
struct EvoMutationDisplay : TransparentWidget {
  float *oldData = nullptr;  // module->mutationOldSnapshot[p][0..15]
  float *newData = nullptr;  // module->pool[p][0..15]
  float *flash = nullptr;    // module->mutationFlash[p], 1 at the moment of mutation, decaying to 0

  void drawSeq(const DrawArgs &args, float *data, NVGcolor color, float lineWidth) {
    const int n = 16;
    nvgBeginPath(args.vg);
    for (int i = 0; i < n; i++) {
      float v = clamp(data[i] / 5.f, 0.f, 1.f);
      float x = (i + 0.5f) * (box.size.x / n);
      float y = box.size.y * (1.f - v);
      if (i == 0) nvgMoveTo(args.vg, x, y);
      else nvgLineTo(args.vg, x, y);
    }
    nvgStrokeColor(args.vg, color);
    nvgStrokeWidth(args.vg, lineWidth);
    nvgStroke(args.vg);
  }

  void draw(const DrawArgs &args) override {
    if (!oldData || !newData || !flash || *flash <= 0.f) return;
    float a = clamp(*flash, 0.f, 1.f);
    drawSeq(args, oldData, nvgRGBA(0x28, 0x0b, 0x0b, (unsigned char)(90.f * a)), 1.5f);
    drawSeq(args, newData, nvgRGBA(0xd0, 0x20, 0x20, (unsigned char)(220.f * a)), 2.5f);
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

    // Left lane = shared/global playback controls (unchanged from before).
    // Reuses TheReelPeet's exact laneAX/cvDX values.
    const float laneXL = 14.f;
    const float cvDX   = 4.5f;

    const float runY      = 22.5f;
    const float lengthY   = 40.f;
    const float dispY     = 44.f;
    const float bpmY      = 61.f;
    const float bpmDispY  = 65.f;
    const float dynY      = 80.f;   // was 82; nudged up 2mm to help make room for Jitter below
    const float jitterY   = 95.f;   // new row between Dyn and Rise/Fall
    const float riseFallY = 108.f;  // was 105; nudged down 3mm to clear Jitter's label
    const float outY      = 121.f;  // was 117; nudged down 4mm to clear Rise/Fall's label

    // 4 phrase-pool columns on the right. Rows align with the left lane's
    // top two rows (Active with Run, Entropy with Length) for a clean grid
    // look, now that Persist, Phrase L, and Recall are all gone (merged
    // into Entropy / derived from Steps+BPM / a fixed constant). Bottom of
    // each column left empty for future per-phrase CV inputs.
    // These are mm design values (fed through mm2px like everything else
    // here), NOT the SVG's raw rect coordinates (80/126/172/218, width 40,
    // in res/TheReelPeetEvo.svg) — raw SVG units are Rack's own internal
    // pixel space directly, already equal to mm2px()'s output, not its
    // input. Each value here is (raw rect center in that panel) / 2.9528
    // (the mm2px scale), so the knobs land inside their lane's tint.
    const float colX[4] = {33.9f, 49.4f, 65.0f, 80.6f};
    const float activeY      = runY;
    const float entropyY     = lengthY;
    const float recallLightY = entropyY + 17.f;   // below Entropy's own label
    const float mutateLabelY = recallLightY + 9.f;
    const float mutateDispY  = mutateLabelY + 5.f;
    const float mutateDispH  = 44.f;  // fills the rest of the column, down near the bottom margin

    addParam(createParamCentered<LEDButton>(mm2px(Vec(laneXL, runY)), module, TheReelPeetEvo::RUN_PARAM));
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(laneXL, runY)), module, TheReelPeetEvo::RUN_LIGHT));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, lengthY)), module, TheReelPeetEvo::LENGTH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, bpmY)), module, TheReelPeetEvo::BPM_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, dynY)), module, TheReelPeetEvo::DYNAMICS_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL, jitterY)), module, TheReelPeetEvo::JITTER_PARAM));

    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL - cvDX, riseFallY)), module, TheReelPeetEvo::RISE_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL + cvDX, riseFallY)), module, TheReelPeetEvo::FALL_PARAM));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneXL - cvDX, outY)), module, TheReelPeetEvo::OUT_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneXL + cvDX, outY)), module, TheReelPeetEvo::ENV_OUTPUT));

    for (int p = 0; p < 4; p++) {
      addParam(createParamCentered<LEDButton>(mm2px(Vec(colX[p], activeY)), module, TheReelPeetEvo::ACTIVE_PARAM + p));
      addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(colX[p], activeY)), module, TheReelPeetEvo::ACTIVE_LIGHT + p));

      addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(colX[p], entropyY)), module, TheReelPeetEvo::ENTROPY_PARAM + p));
      addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(colX[p], recallLightY)), module, TheReelPeetEvo::RECALL_LIGHT + p));
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
      addLabel("Dyn", laneXL, dynY + 6.f, dispW, 9.f);
      addLabel("Jitter", laneXL, jitterY + 4.f, dispW, 8.f);
      addLabel("Rise", laneXL - cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("Fall", laneXL + cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("1v/O", laneXL - cvDX, outY + 3.5f, dispW2, 8.f);
      addLabel("Gate", laneXL + cvDX, outY + 3.5f, dispW2, 8.f);

      for (int p = 0; p < 4; p++) {
        addLabel(std::to_string(p + 1), colX[p], activeY + 4.f, dispW, 9.f);
        addLabel("Entropy", colX[p], entropyY + 6.f, dispW, 9.f);
        addLabel("Recall", colX[p], recallLightY + 2.5f, dispW, 7.f);
        addLabel("Mutate", colX[p], mutateLabelY, dispW, 7.f);

        auto *mutationDisplay = new EvoMutationDisplay;
        mutationDisplay->box.pos = mm2px(Vec(colX[p] - 6.f, mutateDispY));
        mutationDisplay->box.size = mm2px(Vec(12.f, mutateDispH));
        mutationDisplay->oldData = module->mutationOldSnapshot[p];
        mutationDisplay->newData = module->pool[p];
        mutationDisplay->flash = &module->mutationFlash[p];
        addChild(mutationDisplay);
      }

      auto *lenDisplay = new EvoLengthDisplay;
      lenDisplay->box.pos = mm2px(Vec(laneXL - dispW * 0.5f, dispY));
      lenDisplay->box.size = mm2px(Vec(dispW, 11.f));
      lenDisplay->value = &module->len;
      addChild(lenDisplay);

      auto *bpmDisplay = new EvoBPMDisplay();
      bpmDisplay->param = &module->params[TheReelPeetEvo::BPM_PARAM];
      bpmDisplay->box.pos = mm2px(Vec(laneXL - dispW * 0.5f, bpmDispY));
      bpmDisplay->box.size = mm2px(Vec(dispW, 11.f));
      addChild(bpmDisplay);
    }
  }
};
#pragma clang optimize on

Model *modelTheReelPeetEvo =
    createModel<TheReelPeetEvo, TheReelPeetEvoWidget>("thereelpeet-evo");
