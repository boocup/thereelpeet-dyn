#include "plugin.hpp"

using namespace rack;
using namespace rack::componentlibrary;
using namespace rack::ui;

// =======================
//   MODULE DEFINITION
// =======================

struct TheReelPeet : Module {
  enum ParamId {
    BUTTON_A_PARAM,
    RAND_A_PARAM,
    LENGTH_A_PARAM,

    BUTTON_B_PARAM,
    RAND_B_PARAM,
    LENGTH_B_PARAM,

    BPM_A_PARAM,
    BPM_B_PARAM,

    DYNAMICS_A_PARAM,
    DYNAMICS_B_PARAM,

    RISE_A_PARAM,
    FALL_A_PARAM,
    RISE_B_PARAM,
    FALL_B_PARAM,

    PARAMS_LEN
  };
  enum InputId { INPUTS_LEN };
  enum OutputId {
    OUT_A_OUTPUT,
    ENV_A_OUTPUT,
    OUT_B_OUTPUT,
    ENV_B_OUTPUT,
    OUTPUTS_LEN
  };
  enum LightId { RUN_A_LIGHT, RUN_B_LIGHT, LIGHTS_LEN };
  enum EnvPhase { ENV_IDLE, ENV_ATTACK, ENV_SUSTAIN, ENV_DECAY };

  // Lane A state
  bool runningA = false;
  int stepA = 0;
  float timerA = 0.f;
  float trigTimerA = 0.f;
  float seqA[16];
  float holdTimerA = 0.f;
  float heldCVA = 0.f;
  bool stepMutedA = false;
  float envLevelA = 0.f;
  int envPhaseA = ENV_IDLE;

  // Lane B state
  bool runningB = false;
  int stepB = 0;
  float timerB = 0.f;
  float trigTimerB = 0.f;
  float seqB[16];
  float holdTimerB = 0.f;
  float heldCVB = 0.f;
  bool stepMutedB = false;
  float envLevelB = 0.f;
  int envPhaseB = ENV_IDLE;

  int lenA = 2;
  int lenB = 2;

  dsp::SchmittTrigger onA, randA;
  dsp::SchmittTrigger onB, randB;

  TheReelPeet() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

    configParam(BUTTON_A_PARAM, 0.f, 1.f, 0.f, "Lane A Run toggle");
    configParam(RAND_A_PARAM, 0.f, 1.f, 0.f, "Lane A Randomize");
    configParam(LENGTH_A_PARAM, 2.f, 16.f, 3.f, "Lane A Length (2-16 steps)");

    configParam(BUTTON_B_PARAM, 0.f, 1.f, 0.f, "Lane B Run toggle");
    configParam(RAND_B_PARAM, 0.f, 1.f, 0.f, "Lane B Randomize");
    configParam(LENGTH_B_PARAM, 2.f, 16.f, 3.f, "Lane B Length (2-16 steps)");

    configParam(BPM_A_PARAM, 1.f, 240.f, 120.f, "Lane A BPM", " BPM");
    configParam(BPM_B_PARAM, 1.f, 240.f, 120.f, "Lane B BPM", " BPM");

    configParam(DYNAMICS_A_PARAM, -1.f, 1.f, 0.f,
                "Lane A Dynamics. CW: held gates, CCW: note drops");
    configParam(DYNAMICS_B_PARAM, -1.f, 1.f, 0.f,
                "Lane B Dynamics. CW: held gates, CCW: note drops");

    configParam(RISE_A_PARAM, 0.f, 2.f, 0.f, "Lane A Rise time", " s");
    configParam(FALL_A_PARAM, 0.f, 4.f, 0.5f, "Lane A Fall time", " s");
    configParam(RISE_B_PARAM, 0.f, 2.f, 0.f, "Lane B Rise time", " s");
    configParam(FALL_B_PARAM, 0.f, 4.f, 0.5f, "Lane B Fall time", " s");

    configOutput(OUT_A_OUTPUT, "Pitch CV A (1V/Oct)");
    configOutput(ENV_A_OUTPUT, "Envelope CV A (0-10V)");
    configOutput(OUT_B_OUTPUT, "Pitch CV B (1V/Oct)");
    configOutput(ENV_B_OUTPUT, "Envelope CV B (0-10V)");

    configLight(RUN_A_LIGHT, "Lane A running");
    configLight(RUN_B_LIGHT, "Lane B running");

    for (int i = 0; i < 16; i++) {
      seqA[i] = random::uniform() * 5.f;
      seqB[i] = random::uniform() * 5.f;
    }
  }

  void randomize(float *seq) {
    for (int i = 0; i < 16; i++)
      seq[i] = random::uniform() * 5.f;
  }

  void processLane(bool &running, int &step, float &timer, float &trigTimer,
                   float bpm, float *seq, int length, float &outCV,
                   float &holdTimer, float &heldCV,
                   bool &stepMuted, float &envLevel, int &envPhase,
                   dsp::SchmittTrigger &onTrig, dsp::SchmittTrigger &randTrig,
                   float onVal, float randVal, float dynamics,
                   float riseTime, float fallTime, float &outEnv,
                   const ProcessArgs &args) {

    // RUN STATE
    if (onTrig.process(onVal))
      running = !running;

    // RANDOMIZE
    if (randTrig.process(randVal))
      randomize(seq);

    // TIMING
    bpm = clamp(bpm, 1.f, 240.f);
    const float stepTime = 60.f / bpm;

    outCV = 0.f;

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
        step = (step + 1) % length;

        stepMuted = false;
        trigTimer = 0.f;

        if (holdTimer <= 0.f && envPhase == ENV_IDLE) {
          const float dynVal = clamp(dynamics, -1.f, 1.f);
          const float dynCurved = dynVal * dynVal * std::abs(dynVal);
          if (dynVal > 0.f && random::uniform() < dynCurved) {
            float jitter = 1.f + (random::uniform() * 0.2f - 0.1f);
            holdTimer = riseTime + length * stepTime * jitter;
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

    // ENVELOPE (AR with sustain during holds)
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

    outEnv = envLevel;
  }

  void process(const ProcessArgs &args) override {
    lenA = clamp((int)std::round(params[LENGTH_A_PARAM].getValue()), 1, 16);
    lenB = clamp((int)std::round(params[LENGTH_B_PARAM].getValue()), 1, 16);

    float outA = 0.f, envAout = 0.f;
    float outB = 0.f, envBout = 0.f;

    processLane(runningA, stepA, timerA, trigTimerA,
                params[BPM_A_PARAM].getValue(), seqA, lenA,
                outA, holdTimerA, heldCVA, stepMutedA,
                envLevelA, envPhaseA, onA, randA,
                params[BUTTON_A_PARAM].getValue(),
                params[RAND_A_PARAM].getValue(),
                params[DYNAMICS_A_PARAM].getValue(),
                params[RISE_A_PARAM].getValue(),
                params[FALL_A_PARAM].getValue(),
                envAout, args);

    processLane(runningB, stepB, timerB, trigTimerB,
                params[BPM_B_PARAM].getValue(), seqB, lenB,
                outB, holdTimerB, heldCVB, stepMutedB,
                envLevelB, envPhaseB, onB, randB,
                params[BUTTON_B_PARAM].getValue(),
                params[RAND_B_PARAM].getValue(),
                params[DYNAMICS_B_PARAM].getValue(),
                params[RISE_B_PARAM].getValue(),
                params[FALL_B_PARAM].getValue(),
                envBout, args);

    outputs[OUT_A_OUTPUT].setVoltage(outA);
    outputs[ENV_A_OUTPUT].setVoltage(envAout);
    outputs[OUT_B_OUTPUT].setVoltage(outB);
    outputs[ENV_B_OUTPUT].setVoltage(envBout);

    lights[RUN_A_LIGHT].setBrightness(runningA ? 1.f : 0.f);
    lights[RUN_B_LIGHT].setBrightness(runningB ? 1.f : 0.f);
  }
};

// =======================
//   DISPLAY WIDGETS
// =======================

struct LengthDisplay : TransparentWidget {
  int *value = nullptr;

  void draw(const DrawArgs &args) override {
    if (!value)
      return;

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

struct BPMDisplay : TransparentWidget {
  Param *param = nullptr;

  void draw(const DrawArgs &args) override {
    if (!param)
      return;

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

struct StaticLabel : TransparentWidget {
  std::string text;

  void draw(const DrawArgs &args) override {
    nvgFontFaceId(args.vg, APP->window->uiFont->handle);
    nvgFillColor(args.vg, nvgRGB(0x00, 0x00, 0x00));
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgFontSize(args.vg, 9.f);
    nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.5f, text.c_str(), nullptr);
  }
};

// =======================
//   WIDGET LAYOUT
// =======================

struct TheReelPeetWidget : ModuleWidget {
  TheReelPeetWidget(TheReelPeet *module) {
    setModule(module);
#ifdef METAMODULE_BUILTIN
    setPanel(createPanel("thereelpeet-seq-mm/TheReelPeet.png"));
#else
    setPanel(createPanel(asset::plugin(pluginInstance, "res/TheReelPeet.svg")));
#endif

    addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    const float laneAX   = 14.f;
    const float laneBX   = 36.5f;
    const float cvDX     = 4.5f;

    const float onY       = 20.f;
    const float randY     = 32.f;
    const float knobY     = 48.f;
    const float bpmKnobY  = 69.f;
    const float dynKnobY  = 90.f;
    const float riseKnobY = 105.f;  // Rise (left) / Fall (right) small knobs
    const float outY      = 117.f;  // Pitch (left) + Envelope (right) outputs

    // --- Lane A ---
    addParam(createParamCentered<LEDButton>(mm2px(Vec(laneAX, onY)), module, TheReelPeet::BUTTON_A_PARAM));
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(laneAX, onY)), module, TheReelPeet::RUN_A_LIGHT));

    addParam(createParamCentered<TL1105>(mm2px(Vec(laneAX, randY)), module, TheReelPeet::RAND_A_PARAM));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneAX, knobY)), module, TheReelPeet::LENGTH_A_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneAX, bpmKnobY)), module, TheReelPeet::BPM_A_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneAX, dynKnobY)), module, TheReelPeet::DYNAMICS_A_PARAM));

    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneAX - cvDX, riseKnobY)), module, TheReelPeet::RISE_A_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneAX + cvDX, riseKnobY)), module, TheReelPeet::FALL_A_PARAM));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneAX - cvDX, outY)), module, TheReelPeet::OUT_A_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneAX + cvDX, outY)), module, TheReelPeet::ENV_A_OUTPUT));

    // --- Lane B ---
    addParam(createParamCentered<LEDButton>(mm2px(Vec(laneBX, onY)), module, TheReelPeet::BUTTON_B_PARAM));
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(laneBX, onY)), module, TheReelPeet::RUN_B_LIGHT));

    addParam(createParamCentered<TL1105>(mm2px(Vec(laneBX, randY)), module, TheReelPeet::RAND_B_PARAM));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneBX, knobY)), module, TheReelPeet::LENGTH_B_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneBX, bpmKnobY)), module, TheReelPeet::BPM_B_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneBX, dynKnobY)), module, TheReelPeet::DYNAMICS_B_PARAM));

    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneBX - cvDX, riseKnobY)), module, TheReelPeet::RISE_B_PARAM));
    addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(laneBX + cvDX, riseKnobY)), module, TheReelPeet::FALL_B_PARAM));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneBX - cvDX, outY)), module, TheReelPeet::OUT_B_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(laneBX + cvDX, outY)), module, TheReelPeet::ENV_B_OUTPUT));

    // --- Displays ---
    if (module) {
      const float dispW = 12.f;
      const float stepsDispY = 52.f;
      const float bpmDispY   = 73.f;

      auto *lenADisplay = new LengthDisplay;
      lenADisplay->box.pos = mm2px(Vec(laneAX - dispW * 0.5f, stepsDispY));
      lenADisplay->box.size = mm2px(Vec(dispW, 11.f));
      lenADisplay->value = &module->lenA;
      addChild(lenADisplay);

      auto *lenBDisplay = new LengthDisplay;
      lenBDisplay->box.pos = mm2px(Vec(laneBX - dispW * 0.5f, stepsDispY));
      lenBDisplay->box.size = mm2px(Vec(dispW, 11.f));
      lenBDisplay->value = &module->lenB;
      addChild(lenBDisplay);

      auto *bpmDispA = new BPMDisplay();
      bpmDispA->param = &module->params[TheReelPeet::BPM_A_PARAM];
      bpmDispA->box.pos = mm2px(Vec(laneAX - dispW * 0.5f, bpmDispY));
      bpmDispA->box.size = mm2px(Vec(dispW, 11.f));
      addChild(bpmDispA);

      auto *bpmDispB = new BPMDisplay();
      bpmDispB->param = &module->params[TheReelPeet::BPM_B_PARAM];
      bpmDispB->box.pos = mm2px(Vec(laneBX - dispW * 0.5f, bpmDispY));
      bpmDispB->box.size = mm2px(Vec(dispW, 11.f));
      addChild(bpmDispB);

      auto *dynLabelA = new StaticLabel;
      dynLabelA->text = "Dyn";
      dynLabelA->box.pos = mm2px(Vec(laneAX - dispW * 0.5f, dynKnobY + 5.f));
      dynLabelA->box.size = mm2px(Vec(dispW, 5.f));
      addChild(dynLabelA);

      auto *dynLabelB = new StaticLabel;
      dynLabelB->text = "Dyn";
      dynLabelB->box.pos = mm2px(Vec(laneBX - dispW * 0.5f, dynKnobY + 5.f));
      dynLabelB->box.size = mm2px(Vec(dispW, 5.f));
      addChild(dynLabelB);
    }
  }
};

Model *modelTheReelPeet =
    createModel<TheReelPeet, TheReelPeetWidget>("thereelpeet-seq");
