#include "plugin.hpp"
#include <atomic>

using namespace rack;
using namespace rack::componentlibrary;
using namespace rack::ui;

// =======================
//   MODULE DEFINITION
// =======================
//
// Two-lane audio phrasing module: each lane is a built-in VCA whose gain
// follows a slow, breathing presence envelope - long rests interrupted by
// smooth rises and falls. Timing/envelope behavior is carried over from
// PhrasingModule (Phrasing.cpp), but with audio in/out instead of CV out,
// so it no longer needs an external VCA/Morph 4 to be useful.
//
// Each lane also has an Env output (0-10V), so the envelope can drive
// things outside the module. The intended patch is Env -> Compare 2 ->
// A-166 (ANDed with the note source's trigger) -> a quantizer's trigger,
// so notes only change while the lane is fully silent - the same "no note
// change mid-envelope" behavior Evo gets internally from its A-166-style
// gate. To make that reliable, a lane only re-enters once its fade-out has
// fully finished (see kDoneLevel), and Env reads exactly 0V when it has.
//
// Knob count is kept low for MetaModule, borrowing Evo's tricks for
// collapsing controls:
//
// - Pace couples Phrasing's Density and Duration into one axis. It sets a
//   base time (5-90s); every gap and lane duration is a multiple of it.
//   CW = shorter gaps and shorter events (busy), CCW = long and slow.
//
// - Entropy is Evo's inverse-coupled drift/persist axis, applied to a
//   "form" rather than a melody. The form is kFormLen events, each holding
//   per-lane entry and duration genes plus a gap gene. It plays in a loop;
//   at the end of each pass it may evolve. Low Entropy = the same form keeps
//   returning (recognizable sections), high = Phrasing's old memoryless
//   behavior. Gap/duration jitter knobs are gone - Entropy covers them.
//
// - Rest is a cycling button (same pattern as Evo's Model) replacing
//   Phrasing's per-lane Floor knobs and Guarantee switch: Silent (lanes fade
//   fully out), Bed (resting lanes sit at kBedFloor), One (at least one
//   lane is always playing).
//
// - Recall is a fixed chance, no control, same as Evo: once the form has
//   diverged from its genesis version, a pass occasionally plays that
//   original instead.
//
// Weight (per lane) is read live against each event's entry gene: a lane
// enters on an event when entry < Weight. So turning Weight changes how
// often a lane appears immediately, while the form (which events favor
// which lanes) stays the same.
//
// Clock In is optional. Unpatched, everything runs free in seconds. Patched,
// timing is still in seconds (Pace means the same thing either way), but
// anything that starts or ends a voice waits for the next clock pulse, so
// entrances and exits land on the beat. Fades themselves are not quantized.

struct TheReelPeetUndertow : Module {
  static constexpr int kLanes = 2;

  enum ParamId {
    PACE_PARAM,
    ENTROPY_PARAM,
    REST_PARAM,
    ENABLE_PARAM,                        // ENABLE_PARAM + lane
    WEIGHT_PARAM = ENABLE_PARAM + kLanes,  // WEIGHT_PARAM + lane
    PARAMS_LEN = WEIGHT_PARAM + kLanes
  };
  enum InputId {
    CLOCK_INPUT,
    AUDIO_INPUT,                         // AUDIO_INPUT + lane
    INPUTS_LEN = AUDIO_INPUT + kLanes
  };
  enum OutputId {
    MIX_OUTPUT,
    AUDIO_OUTPUT,                        // AUDIO_OUTPUT + lane
    ENV_OUTPUT = AUDIO_OUTPUT + kLanes,  // ENV_OUTPUT + lane
    OUTPUTS_LEN = ENV_OUTPUT + kLanes
  };
  enum LightId {
    REST_LIGHT,                          // REST_LIGHT + 0..2, one per RestMode
    RECALL_LIGHT = REST_LIGHT + 3,
    DRIFT_LIGHT,
    ENABLE_LIGHT,                        // ENABLE_LIGHT + lane, also shows the lane's level
    LIGHTS_LEN = ENABLE_LIGHT + kLanes
  };
  enum RestMode { REST_SILENT, REST_BED, REST_ONE, NUM_REST_MODES };

  static constexpr int kFormLen = 8;
  struct Event {
    float entry[kLanes];  // 0-1; lane enters on this event when entry < its Weight
    float dur[kLanes];    // lane on-time, as a multiple of Pace's base time
    float gap;            // time until the next event, as a multiple of Pace's base time
  };

  // Gene ranges. With Pace at noon (base ~21s) that's gaps of ~6-19s and
  // on-times of ~10-42s, close to Phrasing's own defaults.
  static constexpr float kDurMin = 0.5f;
  static constexpr float kDurMax = 2.f;
  static constexpr float kGapMin = 0.3f;
  static constexpr float kGapMax = 0.9f;

  static constexpr float kBedFloor = 0.125f;  // ~-18dB
  static constexpr float kEnableFade = 0.02f;  // declick for Enable toggles, seconds
  static constexpr float kRecallChance = 0.25f;
  static constexpr float kAnchorMutationScale = 0.35f;
  static constexpr float kDriftDecayTime = 7.f;
  // A one-pole fade-out never actually reaches zero. Below this envelope
  // level (gain -80dB, since gain is env squared) the lane counts as fully
  // faded: the envelope snaps to 0, Env out reads 0V, and the lane may
  // re-enter.
  static constexpr float kDoneLevel = 0.01f;

  Event form[kFormLen];
  Event genesisForm[kFormLen];  // frozen snapshot from the last newForm(), for Recall
  int eventIdx = 0;
  bool playingGenesis = false;  // this pass's Recall roll
  float gapPhase = 0.f;         // 0-1 progress toward the next event

  bool laneEnabled[kLanes] = {true, true};
  bool laneOn[kLanes] = {false, false};
  float lanePhase[kLanes] = {0.f, 0.f};   // 0-1 progress through the lane's on-time
  float laneDurMul[kLanes] = {1.f, 1.f};  // on-time (and fade times) of the current/last entry
  float laneEnv[kLanes] = {0.f, 0.f};
  float enableGain[kLanes] = {1.f, 1.f};
  int restMode = REST_ONE;
  float driftFlash = 0.f;

  // Set from the UI thread (context menu), consumed in process().
  std::atomic<bool> newFormRequested{false};

  dsp::SchmittTrigger enableTrig[kLanes];
  dsp::SchmittTrigger restTrig;
  dsp::SchmittTrigger clockTrig;

  TheReelPeetUndertow() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

    configParam(PACE_PARAM, 0.f, 1.f, 0.5f,
                "Pace (CW: shorter gaps and entrances, CCW: long and slow)", "%", 0.f, 100.f);
    configParam(ENTROPY_PARAM, 0.f, 1.f, 0.25f,
                "Entropy (CW: form changes more each pass, CCW: form repeats)", "%", 0.f, 100.f);
    configButton(REST_PARAM, "Cycle rest mode (Silent/Bed/One)");
    configLight(REST_LIGHT + (int)REST_SILENT, "Silent: resting lanes fade fully out");
    configLight(REST_LIGHT + (int)REST_BED, "Bed: resting lanes sit at a low level");
    configLight(REST_LIGHT + (int)REST_ONE, "One: at least one lane always playing");
    configLight(RECALL_LIGHT, "Recall: playing the original form this pass");
    configLight(DRIFT_LIGHT, "Drift: the form just changed");

    configInput(CLOCK_INPUT, "Clock (optional: entrances and exits wait for the next pulse)");
    configOutput(MIX_OUTPUT, "Mix");

    for (int i = 0; i < kLanes; i++) {
      std::string n = std::to_string(i + 1);
      configButton(ENABLE_PARAM + i, "Lane " + n + " enable toggle");
      configParam(WEIGHT_PARAM + i, 0.f, 1.f, 0.5f, "Lane " + n + " weight", "%", 0.f, 100.f);
      configInput(AUDIO_INPUT + i, "Lane " + n + " audio");
      configOutput(AUDIO_OUTPUT + i, "Lane " + n + " audio");
      configOutput(ENV_OUTPUT + i, "Lane " + n + " envelope (0-10V, exactly 0V once fully faded out)");
      configBypass(AUDIO_INPUT + i, AUDIO_OUTPUT + i);
      configLight(ENABLE_LIGHT + i, "Lane " + n + " (off: disabled, dim: resting, bright: playing)");
    }

    newForm();
  }

  // Same 5-90s logarithmic range as Phrasing's Density/Duration knobs.
  float baseSeconds() {
    float pace = clamp(params[PACE_PARAM].getValue(), 0.f, 1.f);
    return 5.f * std::pow(18.f, 1.f - pace);
  }

  static float randomDur() {
    return kDurMin * std::pow(kDurMax / kDurMin, random::uniform());
  }
  static float randomGap() {
    return kGapMin + (kGapMax - kGapMin) * random::uniform();
  }

  // Bounded gaussian nudge, reflected off the range edges rather than
  // clamped (same reasoning as Evo's musicalStep: clamping piles values up
  // at the edges).
  static float reflectInto(float v, float lo, float hi) {
    if (v > hi) v = hi - (v - hi);
    if (v < lo) v = lo + (lo - v);
    return clamp(v, lo, hi);
  }
  static float nudge(float v, float sigma, float lo, float hi) {
    float delta = clamp(random::normal() * sigma, -2.f * sigma, 2.f * sigma);
    return reflectInto(v + delta, lo, hi);
  }

  void newForm() {
    for (int k = 0; k < kFormLen; k++) {
      for (int i = 0; i < kLanes; i++) {
        form[k].entry[i] = random::uniform();
        form[k].dur[i] = randomDur();
      }
      form[k].gap = randomGap();
      genesisForm[k] = form[k];
    }
    eventIdx = 0;
    playingGenesis = false;
    // Fire the first event right away (or on the first clock pulse) rather
    // than sitting silent for a whole gap after the module is added.
    gapPhase = 1.f;
  }

  // The downbeat of the form and its halfway point mutate less often, so an
  // evolving form keeps a recognizable backbone (Evo's isAnchorStep).
  static bool isAnchorEvent(int k) {
    return k == 0 || k == kFormLen / 2;
  }

  // Runs once per full pass through the form. Entropy is both the chance
  // the form gets replaced at all (persist = 1 - entropy) and the per-gene
  // mutation chance (drift = entropy), as in Evo. Entry genes are also
  // redrawn outright with probability entropy^2, so full CW is effectively
  // Phrasing's memoryless coin flip every event. Returns whether anything
  // changed.
  bool evolveForm() {
    float entropy = clamp(params[ENTROPY_PARAM].getValue(), 0.f, 1.f);
    if (random::uniform() >= entropy)
      return false;
    bool changed = false;
    for (int k = 0; k < kFormLen; k++) {
      float driftP = isAnchorEvent(k) ? entropy * kAnchorMutationScale : entropy;
      Event &e = form[k];
      for (int i = 0; i < kLanes; i++) {
        if (random::uniform() < driftP) {
          e.entry[i] = (random::uniform() < entropy * entropy) ? random::uniform() : nudge(e.entry[i], 0.2f, 0.f, 1.f);
          changed = true;
        }
        if (random::uniform() < driftP) {
          // Nudged in log space, so a step means the same ratio at any length.
          float logDur = nudge(std::log(e.dur[i]), 0.25f, std::log(kDurMin), std::log(kDurMax));
          e.dur[i] = std::exp(logDur);
          changed = true;
        }
      }
      if (random::uniform() < driftP) {
        e.gap = nudge(e.gap, 0.1f, kGapMin, kGapMax);
        changed = true;
      }
    }
    return changed;
  }

  bool formHasDiverged() {
    for (int k = 0; k < kFormLen; k++) {
      if (form[k].gap != genesisForm[k].gap) return true;
      for (int i = 0; i < kLanes; i++) {
        if (form[k].entry[i] != genesisForm[k].entry[i]) return true;
        if (form[k].dur[i] != genesisForm[k].dur[i]) return true;
      }
    }
    return false;
  }

  float weightOf(int i) {
    return clamp(params[WEIGHT_PARAM + i].getValue(), 0.f, 1.f);
  }

  // A lane is present while it's on or still fading out. It can't re-enter
  // until it's fully faded - the Evo/A-166 rule, applied to the lane
  // envelope, so every fade-out rings out completely.
  bool lanePresent(int i) {
    return laneOn[i] || laneEnv[i] > 0.f;
  }

  void startLane(int i, float durMul) {
    laneOn[i] = true;
    lanePhase[i] = 0.f;
    laneDurMul[i] = durMul;
  }

  void fireEvent() {
    const Event &e = (playingGenesis ? genesisForm : form)[eventIdx];
    for (int i = 0; i < kLanes; i++) {
      if (!laneEnabled[i] || lanePresent(i)) continue;
      if (e.entry[i] < weightOf(i))
        startLane(i, e.dur[i]);
    }
    gapPhase = 0.f;
    eventIdx++;
    if (eventIdx >= kFormLen) {
      eventIdx = 0;
      if (evolveForm())
        driftFlash = 1.f;
      playingGenesis = formHasDiverged() && random::uniform() < kRecallChance;
    }
  }

  // Rest mode "One": once every enabled lane has fully faded, bring one in
  // right away. A lane still fading out counts as present, so this waits
  // rather than pulling it back up mid-fade. Weighted-random by Weight
  // (uniform if all weights are zero), with that lane's on-time taken from
  // the current event so it still follows the form.
  void guaranteeOne() {
    int enabled[kLanes];
    int n = 0;
    for (int i = 0; i < kLanes; i++) {
      if (!laneEnabled[i]) continue;
      if (lanePresent(i)) return;
      enabled[n++] = i;
    }
    if (n == 0) return;
    float sumW = 0.f;
    for (int k = 0; k < n; k++)
      sumW += weightOf(enabled[k]);
    int pick = enabled[std::min((int)(random::uniform() * n), n - 1)];
    if (sumW > 1e-6f) {
      float r = random::uniform() * sumW;
      for (int k = 0; k < n; k++) {
        r -= weightOf(enabled[k]);
        if (r < 0.f) { pick = enabled[k]; break; }
      }
    }
    const Event &e = (playingGenesis ? genesisForm : form)[eventIdx];
    startLane(pick, e.dur[pick]);
  }

  void process(const ProcessArgs &args) override {
    const float dt = args.sampleTime;

    if (newFormRequested.exchange(false))
      newForm();

    for (int i = 0; i < kLanes; i++) {
      if (enableTrig[i].process(params[ENABLE_PARAM + i].getValue())) {
        laneEnabled[i] = !laneEnabled[i];
        if (!laneEnabled[i])
          laneOn[i] = false;
      }
    }
    if (restTrig.process(params[REST_PARAM].getValue()))
      restMode = (restMode + 1) % NUM_REST_MODES;

    // With a clock patched, starts and ends only happen on a pulse. Timers
    // keep running in between; a due event just waits for the next edge.
    bool clockEdge = clockTrig.process(inputs[CLOCK_INPUT].getVoltage());
    bool canChange = !inputs[CLOCK_INPUT].isConnected() || clockEdge;

    // Timers advance as phase (0-1) rather than counting down seconds, so
    // turning Pace takes effect immediately, even mid-gap.
    const float base = baseSeconds();
    for (int i = 0; i < kLanes; i++) {
      if (!laneOn[i]) continue;
      lanePhase[i] += dt / (base * laneDurMul[i]);
      if (lanePhase[i] >= 1.f && canChange)
        laneOn[i] = false;
    }
    gapPhase += dt / (base * (playingGenesis ? genesisForm : form)[eventIdx].gap);
    if (gapPhase >= 1.f && canChange)
      fireEvent();
    if (restMode == REST_ONE && canChange)
      guaranteeOne();

    // Envelope: one-pole smoothing with attack/release time constants
    // derived from the lane's on-time. Attack is Phrasing's (20%, clamped
    // 30ms-2s). Release is 20% (clamped 0.15-12s) - half Phrasing's - since
    // a lane now has to wait for its fade to reach kDoneLevel (~4.6 time
    // constants) before re-entering; at Phrasing's 40% that wait would run
    // longer than the entrance itself.
    const float restFloor = (restMode == REST_BED) ? kBedFloor : 0.f;
    const float enableCoeff = std::exp(-dt / kEnableFade);
    float gain[kLanes];
    for (int i = 0; i < kLanes; i++) {
      float durSec = base * laneDurMul[i];
      float target = laneOn[i] ? 1.f : 0.f;
      float t = (target > laneEnv[i]) ? clamp(durSec * 0.2f, 0.03f, 2.f) : clamp(durSec * 0.2f, 0.15f, 12.f);
      laneEnv[i] = target + (laneEnv[i] - target) * std::exp(-dt / t);
      if (!laneOn[i] && laneEnv[i] < kDoneLevel)
        laneEnv[i] = 0.f;

      float enableTarget = laneEnabled[i] ? 1.f : 0.f;
      enableGain[i] = enableTarget + (enableGain[i] - enableTarget) * enableCoeff;

      // Squared so fades sound even to the ear instead of dropping off
      // suddenly at the tail, as a linear gain would.
      float shaped = enableGain[i] * laneEnv[i] * laneEnv[i];
      gain[i] = enableGain[i] * restFloor + (1.f - restFloor) * shaped;
      // Env follows what you hear from the lane's own swell, but ignores the
      // Bed floor, so it's 0V whenever the lane is resting in any Rest mode.
      outputs[ENV_OUTPUT + i].setVoltage(10.f * shaped);
    }

    float mix[PORT_MAX_CHANNELS] = {};
    int mixChannels = 1;
    for (int i = 0; i < kLanes; i++) {
      Input &in = inputs[AUDIO_INPUT + i];
      Output &out = outputs[AUDIO_OUTPUT + i];
      int channels = in.getChannels();
      out.setChannels(channels);
      for (int c = 0; c < channels; c++) {
        float v = in.getVoltage(c) * gain[i];
        out.setVoltage(v, c);
        mix[c] += v;
      }
      mixChannels = std::max(mixChannels, channels);
    }
    outputs[MIX_OUTPUT].setChannels(mixChannels);
    for (int c = 0; c < mixChannels; c++)
      outputs[MIX_OUTPUT].setVoltage(mix[c], c);

    driftFlash = std::max(0.f, driftFlash - dt / kDriftDecayTime);
    for (int m = 0; m < NUM_REST_MODES; m++)
      lights[REST_LIGHT + m].setBrightness(m == restMode ? 1.f : 0.f);
    lights[RECALL_LIGHT].setBrightness(playingGenesis ? 1.f : 0.f);
    // Hidden during Recall, as in Evo - "just changed" is contradictory
    // while you're hearing the untouched original.
    lights[DRIFT_LIGHT].setBrightness(playingGenesis ? 0.f : driftFlash);
    // One light per lane: off = disabled, dim = enabled but resting,
    // brightening with the fade as the lane plays.
    const float kRestingBrightness = 0.2f;
    for (int i = 0; i < kLanes; i++) {
      float brightness = laneEnabled[i] ? kRestingBrightness + (1.f - kRestingBrightness) * laneEnv[i] : 0.f;
      lights[ENABLE_LIGHT + i].setBrightness(brightness);
    }
  }

  // The form is saved with the patch, so a patch reopens with the same
  // piece rather than a fresh random one. Each event is saved as
  // [entry x kLanes, dur x kLanes, gap].
  static constexpr int kEventJsonSize = 2 * kLanes + 1;

  static json_t *eventToJson(const Event &e) {
    json_t *j = json_array();
    for (int i = 0; i < kLanes; i++)
      json_array_append_new(j, json_real(e.entry[i]));
    for (int i = 0; i < kLanes; i++)
      json_array_append_new(j, json_real(e.dur[i]));
    json_array_append_new(j, json_real(e.gap));
    return j;
  }

  static bool eventFromJson(json_t *j, Event &e) {
    if (!j || json_array_size(j) != kEventJsonSize) return false;
    for (int i = 0; i < kLanes; i++) {
      e.entry[i] = clamp((float)json_number_value(json_array_get(j, i)), 0.f, 1.f);
      e.dur[i] = clamp((float)json_number_value(json_array_get(j, kLanes + i)), kDurMin, kDurMax);
    }
    e.gap = clamp((float)json_number_value(json_array_get(j, 2 * kLanes)), kGapMin, kGapMax);
    return true;
  }

  static json_t *formToJson(const Event *f) {
    json_t *j = json_array();
    for (int k = 0; k < kFormLen; k++)
      json_array_append_new(j, eventToJson(f[k]));
    return j;
  }

  // All-or-nothing, so a malformed save can't leave a half-loaded form.
  static bool formFromJson(json_t *j, Event *f) {
    if (!j || json_array_size(j) != kFormLen) return false;
    Event loaded[kFormLen];
    for (int k = 0; k < kFormLen; k++)
      if (!eventFromJson(json_array_get(j, k), loaded[k])) return false;
    for (int k = 0; k < kFormLen; k++)
      f[k] = loaded[k];
    return true;
  }

  json_t *dataToJson() override {
    json_t *rootJ = json_object();
    json_t *laneJ = json_array();
    for (int i = 0; i < kLanes; i++)
      json_array_append_new(laneJ, json_boolean(laneEnabled[i]));
    json_object_set_new(rootJ, "laneEnabled", laneJ);
    json_object_set_new(rootJ, "restMode", json_integer(restMode));
    json_object_set_new(rootJ, "form", formToJson(form));
    json_object_set_new(rootJ, "genesisForm", formToJson(genesisForm));
    return rootJ;
  }

  void dataFromJson(json_t *rootJ) override {
    json_t *laneJ = json_object_get(rootJ, "laneEnabled");
    if (laneJ) {
      for (int i = 0; i < kLanes && i < (int)json_array_size(laneJ); i++)
        laneEnabled[i] = json_boolean_value(json_array_get(laneJ, i));
    }
    json_t *restJ = json_object_get(rootJ, "restMode");
    if (restJ) {
      int v = (int)json_integer_value(restJ);
      if (v >= 0 && v < NUM_REST_MODES)
        restMode = v;
    }
    Event loadedForm[kFormLen];
    Event loadedGenesis[kFormLen];
    if (formFromJson(json_object_get(rootJ, "form"), loadedForm) &&
        formFromJson(json_object_get(rootJ, "genesisForm"), loadedGenesis)) {
      for (int k = 0; k < kFormLen; k++) {
        form[k] = loadedForm[k];
        genesisForm[k] = loadedGenesis[k];
      }
      eventIdx = 0;
      playingGenesis = false;
      gapPhase = 1.f;
    }
  }
};

// =======================
//   DISPLAY WIDGETS
// =======================

// nanosvg doesn't render <text>, so panel labels are drawn here (same
// approach as EvoStaticLabel).
struct UndertowStaticLabel : TransparentWidget {
  std::string text;
  float fontSize = 8.f;
  bool bold = false;
  bool leftAlign = false;
  // Greyscale on purpose: MetaModule boosts saturation on any non-grey text
  // color (see the poolColors note in TheReelPeetEvo.cpp).
  NVGcolor color = nvgRGB(0x00, 0x00, 0x00);

  void draw(const DrawArgs &args) override {
    nvgFontFaceId(args.vg, APP->window->uiFont->handle);
    nvgFillColor(args.vg, color);
    nvgFontSize(args.vg, fontSize);
    if (bold)
      nvgFontBlur(args.vg, 0.15f);
    if (leftAlign) {
      nvgTextAlign(args.vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
      nvgText(args.vg, 0.f, box.size.y * 0.5f, text.c_str(), nullptr);
    } else {
      nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
      nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.5f, text.c_str(), nullptr);
    }
  }
};

// =======================
//   WIDGET LAYOUT
// =======================
//
// 12HP. Coordinates in mm (fed through mm2px), matching the boxes in
// res/TheReelPeetUndertow.svg (1mm = 2.9528px). Global controls sit in the
// top box; the two lanes are columns below it.

// Same scoped optimize-off as TheReelPeetEvoWidget (string-literal args to
// the addLabel lambda were miscompiled at -O3 there). Constructor only runs
// at patch load, so there's no performance cost.
#pragma clang optimize off
struct TheReelPeetUndertowWidget : ModuleWidget {
  TheReelPeetUndertowWidget(TheReelPeetUndertow *module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/TheReelPeetUndertow.svg")));

    addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    // Global box (14-52.5mm)
    const float paceX = 12.f;
    const float entropyX = 49.f;
    const float knobY = 24.5f;
    // Rest group: heading, then button and the three mode lights in one
    // column, mode names to their right.
    const float restX = 26.f;
    const float restHeadingY = 17.f;
    const float restBtnY = 22.f;
    const float restRowY[3] = {28.f, 32.f, 36.f};
    // Clock / Recall / Drift / Mix row, evenly spaced across the box.
    const float globalRowX[4] = {9.5f, 23.5f, 37.5f, 51.5f};
    const float globalJackY = 43.5f;

    // Lane columns (54.5-121.5mm, clear of the bottom screws). Each lane has
    // a left sub-column (lane number, audio In) and a right one (Enable,
    // then Out and Env stacked on one output plate); Weight is centered
    // between them.
    const float laneX[TheReelPeetUndertow::kLanes] = {16.09f, 44.87f};
    const float subDX = 6.5f;
    const float laneTopY = 61.f;
    const float weightY = 75.f;
    const float outY = 96.f;
    const float envY = 110.f;
    const float inY = (outY + envY) * 0.5f;

    // Jack labels sit this far below the jack's center, on every row.
    const float jackLabelDY = 6.f;

    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(paceX, knobY)), module, TheReelPeetUndertow::PACE_PARAM));
    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(entropyX, knobY)), module, TheReelPeetUndertow::ENTROPY_PARAM));

    addParam(createParamCentered<LEDButton>(mm2px(Vec(restX, restBtnY)), module, TheReelPeetUndertow::REST_PARAM));
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(restX, restRowY[TheReelPeetUndertow::REST_SILENT])), module, TheReelPeetUndertow::REST_LIGHT + (int)TheReelPeetUndertow::REST_SILENT));
    addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(restX, restRowY[TheReelPeetUndertow::REST_BED])), module, TheReelPeetUndertow::REST_LIGHT + (int)TheReelPeetUndertow::REST_BED));
    addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(restX, restRowY[TheReelPeetUndertow::REST_ONE])), module, TheReelPeetUndertow::REST_LIGHT + (int)TheReelPeetUndertow::REST_ONE));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(globalRowX[0], globalJackY)), module, TheReelPeetUndertow::CLOCK_INPUT));
    addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(globalRowX[1], globalJackY)), module, TheReelPeetUndertow::RECALL_LIGHT));
    addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(globalRowX[2], globalJackY)), module, TheReelPeetUndertow::DRIFT_LIGHT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(globalRowX[3], globalJackY)), module, TheReelPeetUndertow::MIX_OUTPUT));

    for (int i = 0; i < TheReelPeetUndertow::kLanes; i++) {
      float left = laneX[i] - subDX;
      float right = laneX[i] + subDX;
      addParam(createParamCentered<LEDButton>(mm2px(Vec(right, laneTopY)), module, TheReelPeetUndertow::ENABLE_PARAM + i));
      addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(right, laneTopY)), module, TheReelPeetUndertow::ENABLE_LIGHT + i));
      addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(laneX[i], weightY)), module, TheReelPeetUndertow::WEIGHT_PARAM + i));
      addInput(createInputCentered<PJ301MPort>(mm2px(Vec(left, inY)), module, TheReelPeetUndertow::AUDIO_INPUT + i));
      addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(right, outY)), module, TheReelPeetUndertow::AUDIO_OUTPUT + i));
      addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(right, envY)), module, TheReelPeetUndertow::ENV_OUTPUT + i));
    }

    // Labels are static and deliberately not gated on `module`, so the
    // library/browser previews (module == nullptr) still show them.
    // y is the label's vertical center. onPlate labels are white, for the
    // dark output plates in the SVG.
    auto addLabel = [&](const std::string &text, float x, float y, float w,
                         float fontSize, bool bold = false, bool leftAlign = false,
                         bool onPlate = false) {
      auto *label = new UndertowStaticLabel;
      label->text = text;
      label->fontSize = fontSize;
      label->bold = bold;
      label->leftAlign = leftAlign;
      if (onPlate)
        label->color = nvgRGB(0xff, 0xff, 0xff);
      label->box.pos = mm2px(Vec(leftAlign ? x : x - w * 0.5f, y - 2.5f));
      label->box.size = mm2px(Vec(w, 5.f));
      addChild(label);
    };

    // "THEREELPEET" wordmark is path art in the SVG; the module name sits under it.
    addLabel("UNDERTOW", 30.48f, 9.3f, 40.f, 11.f, true);

    addLabel("Pace", paceX, knobY + 8.5f, 12.f, 8.f);
    addLabel("Entropy", entropyX, knobY + 8.5f, 12.f, 8.f);
    addLabel("Rest", 30.48f, restHeadingY, 12.f, 8.f, true);
    addLabel("Silent", restX + 2.5f, restRowY[0], 12.f, 7.f, false, true);
    addLabel("Bed", restX + 2.5f, restRowY[1], 12.f, 7.f, false, true);
    addLabel("One", restX + 2.5f, restRowY[2], 12.f, 7.f, false, true);

    addLabel("Clock", globalRowX[0], globalJackY + jackLabelDY, 10.f, 8.f);
    addLabel("Recall", globalRowX[1], globalJackY + jackLabelDY, 10.f, 8.f);
    addLabel("Drift", globalRowX[2], globalJackY + jackLabelDY, 10.f, 8.f);
    addLabel("Mix", globalRowX[3], globalJackY + jackLabelDY, 10.f, 8.f, false, false, true);

    for (int i = 0; i < TheReelPeetUndertow::kLanes; i++) {
      float left = laneX[i] - subDX;
      float right = laneX[i] + subDX;
      addLabel(std::to_string(i + 1), left, laneTopY, 8.f, 11.f, true);
      addLabel("Weight", laneX[i], weightY + 8.f, 12.f, 8.f);
      addLabel("In", left, inY + jackLabelDY, 10.f, 8.f);
      addLabel("Out", right, outY + jackLabelDY, 10.f, 8.f, false, false, true);
      addLabel("Env", right, envY + jackLabelDY, 10.f, 8.f, false, false, true);
    }
  }

  void appendContextMenu(Menu *menu) override {
    auto *module = dynamic_cast<TheReelPeetUndertow *>(this->module);
    if (!module) return;
    menu->addChild(new MenuSeparator);
    menu->addChild(createMenuItem("Generate new form", "", [=]() {
      module->newFormRequested = true;
    }));
  }
};
#pragma clang optimize on

Model *modelTheReelPeetUndertow =
    createModel<TheReelPeetUndertow, TheReelPeetUndertowWidget>("thereelpeet-undertow");
