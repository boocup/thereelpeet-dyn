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
// own Active toggle, Drift, Persist, and Phrase Duration.
//
// Genesis (Run off->on transition) randomizes master[16] fresh, then
// derives every one of the 4 backing pool slots from it (drift-gated
// per-step variation, using each phrase's own Drift) — this is the one
// point where all phrases share common DNA. Flipping an individual
// phrase's Active toggle from off->on re-seeds just that phrase from the
// current master (not a fresh one), so phrases activated later still
// share the same lineage as those already running.
//
// Playback cycles through whichever phrases are Active, each held for its
// own Phrase Duration. Once every active phrase has played once (one full
// round), each active phrase evolves independently: crossover with a
// random other active phrase, drift-gated mutation, persist-gated
// replacement (its own Drift/Persist, not a shared one).

struct TheReelPeetEvo : Module {
  enum ParamId {
    RUN_PARAM,
    LENGTH_PARAM,
    BPM_PARAM,
    DYNAMICS_PARAM,
    RISE_PARAM,
    FALL_PARAM,
    ACTIVE_PARAM,                        // ACTIVE_PARAM + 0..3
    DRIFT_PARAM = ACTIVE_PARAM + 4,      // DRIFT_PARAM + 0..3
    PERSIST_PARAM = DRIFT_PARAM + 4,     // PERSIST_PARAM + 0..3
    PHRASE_L_PARAM = PERSIST_PARAM + 4,  // PHRASE_L_PARAM + 0..3
    RECALL_PARAM = PHRASE_L_PARAM + 4,   // RECALL_PARAM + 0..3
    PARAMS_LEN = RECALL_PARAM + 4
  };
  enum InputId { INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, ENV_OUTPUT, OUTPUTS_LEN };
  enum LightId {
    RUN_LIGHT,
    ACTIVE_LIGHT,  // ACTIVE_LIGHT + 0..3
    LIGHTS_LEN = ACTIVE_LIGHT + 4
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
  bool phraseActive[4] = {true, true, true, true};
  float blinkPhase = 0.f;  // drives the currently-playing light's pulse; independent of Phrase Duration
  int currentPhraseIdx = 0;
  bool usingGenesisNow = false;  // this playthrough's Recall roll for currentPhraseIdx
  float phraseTimer = 0.f;
  int phrasesPlayedThisRound = 0;

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

    for (int p = 0; p < 4; p++) {
      std::string n = std::to_string(p + 1);
      configParam(ACTIVE_PARAM + p, 0.f, 1.f, 0.f, "Phrase " + n + " active toggle");
      configParam(DRIFT_PARAM + p, 0.f, 1.f, 0.25f, "Phrase " + n + " drift (mutation rate)", "%", 0.f, 100.f);
      configParam(PERSIST_PARAM + p, 0.f, 1.f, 0.75f, "Phrase " + n + " persist (stability)", "%", 0.f, 100.f);
      configParam(PHRASE_L_PARAM + p, 6.f, 60.f, 20.f, "Phrase " + n + " duration", " s");
      configParam(RECALL_PARAM + p, 0.f, 1.f, 0.f,
                  "Phrase " + n + " recall (chance of playing the genesis snapshot instead of current)",
                  "%", 0.f, 100.f);
      configLight(ACTIVE_LIGHT + p, "Phrase " + n + " active");
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

  // Re-derives a single phrase from the current master (not a fresh one),
  // gated by that phrase's own drift. Used both by genesis() and when a
  // phrase's Active toggle goes off->on, so phrases activated later still
  // share the same lineage as those already playing. The result is also
  // saved as that phrase's Recall snapshot.
  void reseedPhrase(int p) {
    float driftP = params[DRIFT_PARAM + p].getValue();
    for (int i = 0; i < 16; i++) {
      pool[p][i] = (random::uniform() < driftP) ? random::uniform() * 5.f : master[i];
      poolGenesis[p][i] = pool[p][i];
    }
  }

  // Makes idx the currently-playing phrase and rolls its Recall check for
  // this playthrough: at Recall=1 this always picks the frozen genesis
  // snapshot (evolution keeps happening in the background, you just never
  // hear it); at Recall=0 it always plays the current evolved content.
  // Does NOT touch phraseTimer - callers that need a hard reset (genesis,
  // jumping because the current phrase went inactive) do that themselves;
  // the normal round-advance path preserves the remainder instead of
  // zeroing it, to avoid timing drift.
  void selectPhrase(int idx) {
    currentPhraseIdx = idx;
    float recallP = params[RECALL_PARAM + idx].getValue();
    usingGenesisNow = random::uniform() < recallP;
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
  // random other active phrase, gated by its own drift (mutation) and
  // persist (whether it actually gets replaced). Candidates are computed
  // from a snapshot of the pool before any writes happen, so every
  // phrase's crossover partner is last round's content, not a partially-
  // updated mix.
  void evolveAllPhrases() {
    int activeIdx[4];
    int n = 0;
    for (int p = 0; p < 4; p++)
      if (phraseActive[p]) activeIdx[n++] = p;
    if (n == 0) return;

    float candidates[4][16];
    for (int k = 0; k < n; k++) {
      int p = activeIdx[k];
      float driftP = params[DRIFT_PARAM + p].getValue();
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
      float persistP = params[PERSIST_PARAM + p].getValue();
      if (random::uniform() >= persistP) {
        for (int i = 0; i < 16; i++)
          pool[p][i] = candidates[p][i];
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
        float phraseDuration = params[PHRASE_L_PARAM + currentPhraseIdx].getValue();
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
    const float dynY      = 82.f;
    const float riseFallY = 105.f;
    const float outY      = 117.f;

    // 4 phrase-pool columns on the right. Rows align with the left lane's
    // rows (Active with Run, Drift with Length, Persist with BPM, Phrase L
    // with Dyn) for a clean grid look. Bottom of each column left empty for
    // future per-phrase CV inputs.
    // These are mm design values (fed through mm2px like everything else
    // here), NOT the SVG's raw rect coordinates (80/126/172/218, width 40,
    // in res/TheReelPeetEvo.svg) — raw SVG units are Rack's own internal
    // pixel space directly, already equal to mm2px()'s output, not its
    // input. Each value here is (raw rect center in that panel) / 2.9528
    // (the mm2px scale), so the knobs land inside their lane's tint.
    const float colX[4] = {33.9f, 49.4f, 65.0f, 80.6f};
    const float activeY = runY;
    const float driftY  = lengthY;
    const float persistY = bpmY;
    const float phraseLY = dynY;
    const float recallY  = phraseLY + 21.f;  // continues the ~21mm row spacing pattern

    addParam(createParamCentered<LEDButton>(mm2px(Vec(laneXL, runY)), module, TheReelPeetEvo::RUN_PARAM));
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(laneXL, runY)), module, TheReelPeetEvo::RUN_LIGHT));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, lengthY)), module, TheReelPeetEvo::LENGTH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, bpmY)), module, TheReelPeetEvo::BPM_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, dynY)), module, TheReelPeetEvo::DYNAMICS_PARAM));

    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL - cvDX, riseFallY)), module, TheReelPeetEvo::RISE_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL + cvDX, riseFallY)), module, TheReelPeetEvo::FALL_PARAM));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneXL - cvDX, outY)), module, TheReelPeetEvo::OUT_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneXL + cvDX, outY)), module, TheReelPeetEvo::ENV_OUTPUT));

    for (int p = 0; p < 4; p++) {
      addParam(createParamCentered<LEDButton>(mm2px(Vec(colX[p], activeY)), module, TheReelPeetEvo::ACTIVE_PARAM + p));
      addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(colX[p], activeY)), module, TheReelPeetEvo::ACTIVE_LIGHT + p));

      addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(colX[p], driftY)), module, TheReelPeetEvo::DRIFT_PARAM + p));
      addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(colX[p], persistY)), module, TheReelPeetEvo::PERSIST_PARAM + p));
      addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(colX[p], phraseLY)), module, TheReelPeetEvo::PHRASE_L_PARAM + p));
      addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(colX[p], recallY)), module, TheReelPeetEvo::RECALL_PARAM + p));
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
      addLabel("Rise", laneXL - cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("Fall", laneXL + cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("1v/O", laneXL - cvDX, outY + 3.5f, dispW2, 8.f);
      addLabel("Gate", laneXL + cvDX, outY + 3.5f, dispW2, 8.f);

      std::string phraseLLabel = "Phrase L";
      for (int p = 0; p < 4; p++) {
        addLabel(std::to_string(p + 1), colX[p], activeY + 4.f, dispW, 9.f);
        addLabel("Drift", colX[p], driftY + 6.f, dispW, 9.f);
        addLabel("Persist", colX[p], persistY + 6.f, dispW, 9.f);
        addLabel(phraseLLabel, colX[p], phraseLY + 6.f, dispW, 9.f);
        addLabel("Recall", colX[p], recallY + 6.f, dispW, 9.f);
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
