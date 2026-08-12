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
// mechanism as TheReelPeet::processLane, just for one lane.
//
// Phrase-cycling design: pool[4][16] is now the live playback content, not
// hidden background storage. On genesis (Run off->on transition, or a Gen
// press at any time) a fresh master[16] is randomized, and each active
// pool slot is derived from it (drift-gated per-step variation) — this is
// the "genesis" connection all phrases share. Playback then cycles through
// the active pool slots, each held for Phrase Duration seconds. Once a
// full run-through of the pool completes, every active phrase evolves
// independently — elitism (its own current content) crossed with a random
// other active phrase, drift-gated mutation, persist-gated replacement —
// same genetic mechanism as before, just applied per-phrase instead of to
// one global master.

struct TheReelPeetEvo : Module {
  enum ParamId {
    RUN_PARAM,
    GENERATE_PARAM,  // momentary trigger: re-run genesis (same as Run off->on)
    LENGTH_PARAM,
    BPM_PARAM,
    DYNAMICS_PARAM,
    DRIFT_PARAM,
    PERSIST_PARAM,
    PHRASE_DURATION_PARAM,
    POOL_SIZE_PARAM,   // hidden state, 0/1/2 => pool size 2/3/4; not bound to a visible knob
    POOL_CYCLE_PARAM,  // momentary button: advances POOL_SIZE_PARAM
    RISE_PARAM,
    FALL_PARAM,
    PARAMS_LEN
  };
  enum InputId { INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, ENV_OUTPUT, OUTPUTS_LEN };
  enum LightId { RUN_LIGHT, POOL_LIGHT_2, POOL_LIGHT_3, POOL_LIGHT_4, LIGHTS_LEN };
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
  int currentPhraseIdx = 0;
  float phraseTimer = 0.f;

  dsp::SchmittTrigger onTrig, genTrig, poolCycleTrig;

  TheReelPeetEvo() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

    configParam(RUN_PARAM, 0.f, 1.f, 0.f, "Run toggle");
    configParam(GENERATE_PARAM, 0.f, 1.f, 0.f, "Gen (re-run genesis)");
    configParam(LENGTH_PARAM, 2.f, 16.f, 8.f, "Length (2-16 steps)");
    configParam(BPM_PARAM, 1.f, 240.f, 120.f, "BPM", " BPM");
    configParam(DYNAMICS_PARAM, -1.f, 1.f, 0.f,
                "Dynamics. CW: held gates, CCW: note drops");
    configParam(DRIFT_PARAM, 0.f, 1.f, 0.25f, "Drift (mutation rate)", "%", 0.f, 100.f);
    configParam(PERSIST_PARAM, 0.f, 1.f, 0.75f, "Persist (per-phrase stability)", "%", 0.f, 100.f);
    configParam(PHRASE_DURATION_PARAM, 6.f, 60.f, 20.f, "Phrase duration", " s");
    configSwitch(POOL_SIZE_PARAM, 0.f, 2.f, 2.f, "Pool size", {"2", "3", "4"});
    configParam(POOL_CYCLE_PARAM, 0.f, 1.f, 0.f, "Cycle pool size");
    configParam(RISE_PARAM, 0.f, 2.f, 0.f, "Rise time", " s");
    configParam(FALL_PARAM, 0.f, 4.f, 0.5f, "Fall time", " s");

    configOutput(OUT_OUTPUT, "Pitch CV (1V/Oct)");
    configOutput(ENV_OUTPUT, "Envelope CV (0-10V)");

    configLight(RUN_LIGHT, "Running");
    configLight(POOL_LIGHT_2, "Pool size 2");
    configLight(POOL_LIGHT_3, "Pool size 3");
    configLight(POOL_LIGHT_4, "Pool size 4");

    genesis(0.25f);
  }

  // Genesis: fresh random master, then each of the 4 backing pool slots is
  // derived from it (drift-gated per-step variation from the master, not a
  // flat copy) — this is the one point where all phrases share common DNA.
  void genesis(float drift) {
    for (int i = 0; i < 16; i++)
      master[i] = random::uniform() * 5.f;
    for (int p = 0; p < 4; p++) {
      for (int i = 0; i < 16; i++)
        pool[p][i] = (random::uniform() < drift) ? random::uniform() * 5.f : master[i];
    }
    currentPhraseIdx = 0;
    phraseTimer = 0.f;
  }

  // Fires once per full run-through of the active pool. Each active phrase
  // evolves from ITS OWN current content (elitism) crossed with a random
  // other active phrase, gated by drift (mutation) and persist (whether it
  // actually gets replaced). Candidates are computed from a snapshot of the
  // pool before any writes happen, so every phrase's crossover partner is
  // last round's content, not a partially-updated mix.
  void evolveAllPhrases(float drift, float persist, int poolSize) {
    float candidates[4][16];
    for (int p = 0; p < poolSize; p++) {
      int partner = (int)(random::uniform() * poolSize);
      if (partner >= poolSize) partner = poolSize - 1;
      int split = 1 + (int)(random::uniform() * 14.f);  // 1..14
      for (int i = 0; i < 16; i++) {
        candidates[p][i] = (i < split) ? pool[p][i] : pool[partner][i];
        if (random::uniform() < drift)
          candidates[p][i] = random::uniform() * 5.f;
      }
    }
    for (int p = 0; p < poolSize; p++) {
      if (random::uniform() >= persist) {
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

    float drift = params[DRIFT_PARAM].getValue();
    float persist = params[PERSIST_PARAM].getValue();

    if (poolCycleTrig.process(params[POOL_CYCLE_PARAM].getValue())) {
      int idx = (int)std::round(params[POOL_SIZE_PARAM].getValue());
      idx = (idx + 1) % 3;
      params[POOL_SIZE_PARAM].setValue((float)idx);
    }
    int poolIdx = clamp((int)std::round(params[POOL_SIZE_PARAM].getValue()), 0, 2);
    lights[POOL_LIGHT_2].setBrightness(poolIdx == 0 ? 1.f : 0.f);
    lights[POOL_LIGHT_3].setBrightness(poolIdx == 1 ? 1.f : 0.f);
    lights[POOL_LIGHT_4].setBrightness(poolIdx == 2 ? 1.f : 0.f);
    int poolSize = 2 + poolIdx;
    if (currentPhraseIdx >= poolSize) currentPhraseIdx = 0;

    bool genPressed = genTrig.process(params[GENERATE_PARAM].getValue());
    bool justStarted = running && !wasRunning;
    if (genPressed || justStarted)
      genesis(drift);

    float bpm = clamp(params[BPM_PARAM].getValue(), 1.f, 240.f);
    const float stepTime = 60.f / bpm;
    float riseTime = params[RISE_PARAM].getValue();
    float fallTime = params[FALL_PARAM].getValue();
    float dynamics = params[DYNAMICS_PARAM].getValue();
    float phraseDuration = params[PHRASE_DURATION_PARAM].getValue();

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
      phraseTimer += args.sampleTime;
      if (phraseTimer >= phraseDuration) {
        phraseTimer -= phraseDuration;
        currentPhraseIdx++;
        if (currentPhraseIdx >= poolSize) {
          currentPhraseIdx = 0;
          evolveAllPhrases(drift, persist, poolSize);
        }
      }

      timer += args.sampleTime;
      if (timer >= stepTime) {
        timer -= stepTime;
        step = (step + 1) % len;

        stepMuted = false;
        trigTimer = 0.f;

        if (holdTimer <= 0.f && envPhase == ENV_IDLE) {
          const float dynVal = clamp(dynamics, -1.f, 1.f);
          const float dynCurved = dynVal * dynVal * std::abs(dynVal);
          if (dynVal > 0.f && random::uniform() < dynCurved) {
            float jitter = 1.f + (random::uniform() * 0.2f - 0.1f);
            holdTimer = riseTime + len * stepTime * jitter;
            heldCV = pool[currentPhraseIdx][step];
            envPhase = ENV_ATTACK;
          } else if (dynVal < 0.f && random::uniform() < dynCurved) {
            stepMuted = true;
            envLevel = 0.f;
            envPhase = ENV_IDLE;
          } else {
            trigTimer = 0.01f;
            heldCV = pool[currentPhraseIdx][step];
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

struct TheReelPeetEvoWidget : ModuleWidget {
  TheReelPeetEvoWidget(TheReelPeetEvo *module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/TheReelPeetEvo.svg")));

    addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    // Left "lane" = playback controls, right "lane" = evolution controls.
    // Purely cosmetic split (mirrors TheReelPeet's dual-lane look) — this
    // is still one voice/one output, not two independent lanes. Reuses
    // TheReelPeet's exact laneAX/laneBX/cvDX values so the panel matches
    // its width (150 = 10HP) and lane geometry precisely.
    const float laneXL = 14.f;
    const float laneXR = 36.5f;
    const float cvDX   = 4.5f;

    // Left-lane Y values match TheReelPeet's Lane A exactly (onY, randY,
    // knobY, stepsDispY, bpmKnobY, bpmDispY, dynKnobY, riseKnobY, outY).
    const float runY      = 22.5f;  // midpoint between TheReelPeet's 20 and the 25 tried before
    const float lengthY   = 40.f;   // pulled up 8mm total from TheReelPeet's 48
    const float dispY     = 44.f;
    const float bpmY      = 61.f;   // pulled up 8mm total from TheReelPeet's 69
    const float bpmDispY  = 65.f;
    const float dynY      = 82.f;   // pulled up 8mm total from TheReelPeet's 90
    const float riseFallY = 105.f;  // matches TheReelPeet's knob position, unchanged
    const float outY      = 117.f;  // matches TheReelPeet's port position, unchanged

    // Right lane: Drift/Persist align with the top two knobs on the left
    // (Length, BPM). Pool aligns with the Rise/Fall row.
    const float driftY    = lengthY;
    const float persistY  = bpmY;
    const float poolY     = riseFallY;

    addParam(createParamCentered<LEDButton>(mm2px(Vec(laneXL, runY)), module, TheReelPeetEvo::RUN_PARAM));
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(laneXL, runY)), module, TheReelPeetEvo::RUN_LIGHT));

    // Gen sits on the right lane, same row as Run on the left.
    addParam(createParamCentered<TL1105>(mm2px(Vec(laneXR, runY)), module, TheReelPeetEvo::GENERATE_PARAM));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, lengthY)), module, TheReelPeetEvo::LENGTH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, bpmY)), module, TheReelPeetEvo::BPM_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXL, dynY)), module, TheReelPeetEvo::DYNAMICS_PARAM));

    // Right lane: evolution controls (Drift, Persist, Phrase duration, Pool size)
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXR, driftY)), module, TheReelPeetEvo::DRIFT_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXR, persistY)), module, TheReelPeetEvo::PERSIST_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneXR, dynY)), module, TheReelPeetEvo::PHRASE_DURATION_PARAM));

    // Pool size: cycling button + 3 state lights underneath (2/3/4)
    addParam(createParamCentered<TL1105>(mm2px(Vec(laneXR, poolY)), module, TheReelPeetEvo::POOL_CYCLE_PARAM));
    const float poolLightY = poolY + 7.f;
    addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(laneXR - 4.f, poolLightY)), module, TheReelPeetEvo::POOL_LIGHT_2));
    addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(laneXR, poolLightY)), module, TheReelPeetEvo::POOL_LIGHT_3));
    addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(laneXR + 4.f, poolLightY)), module, TheReelPeetEvo::POOL_LIGHT_4));

    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL - cvDX, riseFallY)), module, TheReelPeetEvo::RISE_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneXL + cvDX, riseFallY)), module, TheReelPeetEvo::FALL_PARAM));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneXL - cvDX, outY)), module, TheReelPeetEvo::OUT_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneXL + cvDX, outY)), module, TheReelPeetEvo::ENV_OUTPUT));

    if (module) {
      const float dispW = 12.f;
      const float dispW2 = 8.f;  // narrower, for the closely-paired labels

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

      // "THEREELPEET" itself is now the real wordmark path in the SVG
      // (see res/TheReelPeetEvo.svg), not drawn here.
      addLabel("EVO", 75.f, 14.f, 30.f, 16.f, true);

      addLabel("Run", laneXL, runY + 4.f, dispW, 9.f);
      addLabel("Gen", laneXR, runY + 4.f, dispW, 9.f);

      addLabel("Dyn", laneXL, dynY + 6.f, dispW, 9.f);
      addLabel("Phrase L", laneXR, dynY + 6.f, dispW, 9.f);
      addLabel("Drift", laneXR, driftY + 6.f, dispW, 9.f);
      addLabel("Persist", laneXR, persistY + 6.f, dispW, 9.f);
      addLabel("Rise", laneXL - cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("Fall", laneXL + cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("1v/O", laneXL - cvDX, outY + 3.5f, dispW2, 8.f);
      addLabel("Gate", laneXL + cvDX, outY + 3.5f, dispW2, 8.f);

      // "2"/"3"/"4" pulled up close to the lights; "Pool" pushed down below
      // them so the two rows don't collide.
      addLabel("2", laneXR - 4.f, poolLightY + 1.5f, 4.f, 7.f);
      addLabel("3", laneXR, poolLightY + 1.5f, 4.f, 7.f);
      addLabel("4", laneXR + 4.f, poolLightY + 1.5f, 4.f, 7.f);
      addLabel("Pool", laneXR, poolLightY + 8.f, dispW, 9.f);

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

Model *modelTheReelPeetEvo =
    createModel<TheReelPeetEvo, TheReelPeetEvoWidget>("thereelpeet-evo");
