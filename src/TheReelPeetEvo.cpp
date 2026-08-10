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
// mechanism as TheReelPeet::processLane, just for one lane. What differs
// is how seq[] gets its content: instead of a flat randomize(), a
// candidate pool evolves it via elitism + crossover + mutation, gated by
// Drift (mutation rate) and Persist (probability the master pattern is
// NOT replaced by a new candidate).

struct TheReelPeetEvo : Module {
  enum ParamId {
    RUN_PARAM,
    MODE_PARAM,      // 0 = Rnd (classic full randomize), 1 = Evo (genetic)
    GENERATE_PARAM,  // momentary trigger: one randomize or one generation
    LENGTH_PARAM,
    BPM_PARAM,
    DYNAMICS_PARAM,
    DRIFT_PARAM,
    PERSIST_PARAM,
    RISE_PARAM,
    FALL_PARAM,
    PARAMS_LEN
  };
  enum InputId { INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, ENV_OUTPUT, OUTPUTS_LEN };
  enum LightId { RUN_LIGHT, LIGHTS_LEN };
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

  float seq[16];
  float pool[4][16];
  int poolWriteIdx = 0;

  dsp::SchmittTrigger onTrig, genTrig;

  TheReelPeetEvo() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

    configParam(RUN_PARAM, 0.f, 1.f, 0.f, "Run toggle");
    configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Mode", {"Rnd", "Evo"});
    configParam(GENERATE_PARAM, 0.f, 1.f, 0.f, "Randomize / Generate");
    configParam(LENGTH_PARAM, 2.f, 16.f, 8.f, "Length (2-16 steps)");
    configParam(BPM_PARAM, 1.f, 240.f, 120.f, "BPM", " BPM");
    configParam(DYNAMICS_PARAM, -1.f, 1.f, 0.f,
                "Dynamics. CW: held gates, CCW: note drops");
    configParam(DRIFT_PARAM, 0.f, 1.f, 0.25f, "Drift (mutation rate)", "%", 0.f, 100.f);
    configParam(PERSIST_PARAM, 0.f, 1.f, 0.75f, "Persist (master stability)", "%", 0.f, 100.f);
    configParam(RISE_PARAM, 0.f, 2.f, 0.f, "Rise time", " s");
    configParam(FALL_PARAM, 0.f, 4.f, 0.5f, "Fall time", " s");

    configOutput(OUT_OUTPUT, "Pitch CV (1V/Oct)");
    configOutput(ENV_OUTPUT, "Envelope CV (0-10V)");

    configLight(RUN_LIGHT, "Running");

    for (int i = 0; i < 16; i++) {
      seq[i] = random::uniform() * 5.f;
      for (int p = 0; p < 4; p++)
        pool[p][i] = random::uniform() * 5.f;
    }
  }

  void randomizeFull(float *s) {
    for (int i = 0; i < 16; i++)
      s[i] = random::uniform() * 5.f;
  }

  // Elitism (seq[] is always parent1) + crossover with a random pool slot
  // + per-step mutation gated by drift. Result goes into the oldest pool
  // slot (rotating buffer) and may be promoted to seq[] depending on persist.
  void runGeneration(float drift, float persist) {
    int p2idx = (int)(random::uniform() * 4.f);
    if (p2idx > 3) p2idx = 3;

    int split = 1 + (int)(random::uniform() * 14.f);  // 1..14

    float candidate[16];
    for (int i = 0; i < 16; i++) {
      candidate[i] = (i < split) ? seq[i] : pool[p2idx][i];
      if (random::uniform() < drift)
        candidate[i] = random::uniform() * 5.f;
    }

    for (int i = 0; i < 16; i++)
      pool[poolWriteIdx][i] = candidate[i];
    poolWriteIdx = (poolWriteIdx + 1) % 4;

    if (random::uniform() >= persist) {
      for (int i = 0; i < 16; i++)
        seq[i] = candidate[i];
    }
  }

  void process(const ProcessArgs &args) override {
    len = clamp((int)std::round(params[LENGTH_PARAM].getValue()), 1, 16);
    bool modeEvo = params[MODE_PARAM].getValue() > 0.5f;

    if (onTrig.process(params[RUN_PARAM].getValue()))
      running = !running;

    if (genTrig.process(params[GENERATE_PARAM].getValue())) {
      if (modeEvo)
        runGeneration(params[DRIFT_PARAM].getValue(), params[PERSIST_PARAM].getValue());
      else
        randomizeFull(seq);
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
            heldCV = seq[step];
            envPhase = ENV_ATTACK;
          } else if (dynVal < 0.f && random::uniform() < dynCurved) {
            stepMuted = true;
            envLevel = 0.f;
            envPhase = ENV_IDLE;
          } else {
            trigTimer = 0.01f;
            heldCV = seq[step];
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

    const float laneX = 9.5f;
    const float cvDX  = 4.5f;

    const float runY      = 14.f;
    const float modeY     = 25.5f;
    const float genY      = 37.f;
    const float lengthY   = 48.5f;
    const float dispY     = 52.5f;   // steps display, just below length knob
    const float bpmY      = 60.f;
    const float bpmDispY  = 64.f;
    const float dynY      = 71.5f;
    const float driftY    = 83.f;
    const float persistY  = 94.5f;
    const float riseFallY = 106.f;
    const float outY      = 117.5f;

    addParam(createParamCentered<LEDButton>(mm2px(Vec(laneX, runY)), module, TheReelPeetEvo::RUN_PARAM));
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(laneX, runY)), module, TheReelPeetEvo::RUN_LIGHT));

    addParam(createParamCentered<CKSS>(mm2px(Vec(laneX, modeY)), module, TheReelPeetEvo::MODE_PARAM));
    addParam(createParamCentered<TL1105>(mm2px(Vec(laneX, genY)), module, TheReelPeetEvo::GENERATE_PARAM));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneX, lengthY)), module, TheReelPeetEvo::LENGTH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneX, bpmY)), module, TheReelPeetEvo::BPM_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneX, dynY)), module, TheReelPeetEvo::DYNAMICS_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneX, driftY)), module, TheReelPeetEvo::DRIFT_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneX, persistY)), module, TheReelPeetEvo::PERSIST_PARAM));

    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneX - cvDX, riseFallY)), module, TheReelPeetEvo::RISE_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneX + cvDX, riseFallY)), module, TheReelPeetEvo::FALL_PARAM));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneX - cvDX, outY)), module, TheReelPeetEvo::OUT_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneX + cvDX, outY)), module, TheReelPeetEvo::ENV_OUTPUT));

    if (module) {
      const float dispW = 12.f;

      auto *lenDisplay = new EvoLengthDisplay;
      lenDisplay->box.pos = mm2px(Vec(laneX - dispW * 0.5f, dispY));
      lenDisplay->box.size = mm2px(Vec(dispW, 11.f));
      lenDisplay->value = &module->len;
      addChild(lenDisplay);

      auto *bpmDisplay = new EvoBPMDisplay();
      bpmDisplay->param = &module->params[TheReelPeetEvo::BPM_PARAM];
      bpmDisplay->box.pos = mm2px(Vec(laneX - dispW * 0.5f, bpmDispY));
      bpmDisplay->box.size = mm2px(Vec(dispW, 11.f));
      addChild(bpmDisplay);
    }
  }
};

Model *modelTheReelPeetEvo =
    createModel<TheReelPeetEvo, TheReelPeetEvoWidget>("thereelpeet-evo");
