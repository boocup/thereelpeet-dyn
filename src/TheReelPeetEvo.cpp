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
//
// Scale quantization is applied only at playback (see heldCV in process()),
// never to master/pool/poolGenesis themselves - genetics keep evolving in
// continuous voltage space (the "genotype"); only the note actually played
// each step (the "phenotype") gets snapped to whichever notes are active.
// That way toggling notes on/off can't disturb the mutation/crossover math,
// and changing the selection live just re-quantizes whatever's already
// there.
//
// Rather than a preset scale-name list, this is 12 individual per-semitone
// toggles (Intellijel Scales-style) - the user builds "C Major" or
// "Bb Pentatonic Minor" etc. by lighting up the notes themselves, same as
// they already do on that hardware. There's deliberately no key/root knob:
// the toggles ARE the key, visibly, since C is always semitone 0.
static const char *kNoteNames[12] = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B",
};

struct TheReelPeetEvo : Module {
  enum ParamId {
    RUN_PARAM,
    LENGTH_PARAM,
    JITTER_PARAM,   // was DYNAMICS_PARAM - Dynamics dropped, this knob repurposed to drive Jitter instead
    RISE_PARAM,
    FALL_PARAM,
    ACTIVE_PARAM,                        // ACTIVE_PARAM + 0..3
    ENTROPY_PARAM = ACTIVE_PARAM + 4,    // ENTROPY_PARAM + 0..3
    // Two independent quantizers, matching the pool grid's left/right
    // columns: L governs phrases 1&3 (poolColLeft[0]), R governs
    // phrases 2&4 (poolColLeft[1]) - see quantSet selection in process().
    NOTE_PARAM_L = ENTROPY_PARAM + 4,     // NOTE_PARAM_L + 0..11, one per semitone (C..B)
    NOTE_PARAM_R = NOTE_PARAM_L + 12,     // NOTE_PARAM_R + 0..11
    PARAMS_LEN = NOTE_PARAM_R + 12
  };
  enum InputId { CLOCK_INPUT, INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, ENV_OUTPUT, OUTPUTS_LEN };
  enum LightId {
    RUN_LIGHT,
    ACTIVE_LIGHT,                      // ACTIVE_LIGHT + 0..3
    RECALL_LIGHT = ACTIVE_LIGHT + 4,   // RECALL_LIGHT + 0..3
    DRIFT_LIGHT = RECALL_LIGHT + 4,    // DRIFT_LIGHT + 0..3
    NOTE_LIGHT_L = DRIFT_LIGHT + 4,    // NOTE_LIGHT_L + 0..11
    NOTE_LIGHT_R = NOTE_LIGHT_L + 12,  // NOTE_LIGHT_R + 0..11
    LIGHTS_LEN = NOTE_LIGHT_R + 12
  };
  enum EnvPhase { ENV_IDLE, ENV_ATTACK, ENV_DECAY };

  bool running = false;
  int step = 0;
  float trigTimer = 0.f;
  float heldCV = 0.f;
  float envLevel = 0.f;
  int envPhase = ENV_IDLE;
  int len = 8;
  int prevLen = 8;  // tracks Length changes so closeLoopSeam re-runs when the wrap point itself moves
  // Which of the 12 semitones (C..B) each quantizer treats as in-scale -
  // see NOTE_PARAM_L/R. Both default to all off (quantizer disabled, CV
  // passed through unquantized - see quantizeToActiveNotes) so adding this
  // feature doesn't change any existing patch's sound until notes are
  // actually toggled on. Which phrases each set governs is dynamic (see
  // quantSet in process()): normally L governs phrases 1&3 and R governs
  // 2&4, but if only one side has any notes active, that side alone
  // governs all 4 phrases and the empty side is ignored entirely.
  bool noteActiveL[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
  bool noteActiveR[12] = {false, false, false, false, false, false, false, false, false, false, false, false};

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
  // Flashes the just-played note's light on top of its steady "enabled"
  // brightness, decaying like mutationFlash above - lets you see which
  // in-scale note actually got picked each step, not just which notes are
  // toggled on.
  float playFlashL[12] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  float playFlashR[12] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  float clockPeriod = 0.5f;    // measured time between the last two Clock In pulses, drives Phrase Duration
  float timeSinceClock = 0.f;  // time since the last Clock In pulse; becomes the next clockPeriod
  // Both false right after genesis() until real Clock In pulses of this
  // run establish a genuine measurement. Without this, phraseTimer (which
  // accumulates every sample regardless of whether any clock pulses have
  // arrived) can exceed a phraseDuration computed from a stale/default
  // clockPeriod almost immediately after pressing Run - firing a spurious
  // premature phrase transition (a real Recall roll) and, with few
  // phrases active, even evolveAllPhrases() (Drift) - both well before
  // the actual incoming clock has sent a single pulse. Reported on real
  // hardware as "Recall and Drift lit up on the same phrase, immediately."
  // Needs two edges, not one: the first edge after Run is pressed only
  // measures "time since genesis" (arbitrary, depends on the clock's
  // phase when Run happened to be pressed), not a real inter-pulse gap -
  // sawFirstEdge marks that starting point; clockMeasured only goes true
  // once a second edge gives an actual period between two real pulses.
  bool sawFirstEdge = false;
  bool clockMeasured = false;

  dsp::SchmittTrigger onTrig;
  dsp::SchmittTrigger activeTrig[4];
  dsp::SchmittTrigger noteTrigL[12];
  dsp::SchmittTrigger noteTrigR[12];
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

    // Note toggles: momentary buttons, same pattern as ACTIVE_PARAM below -
    // the button param itself is just an edge source (see noteTrigL/R in
    // process()); the actual on/off state lives in noteActiveL/R.
    for (int i = 0; i < 12; i++) {
      configParam(NOTE_PARAM_L + i, 0.f, 1.f, 0.f,
                  std::string("Note ") + kNoteNames[i] +
                      " (Left quantizer, optional: phrases 1&3, or all 4 if Right is off)");
      configLight(NOTE_LIGHT_L + i, std::string("Note ") + kNoteNames[i] + " active (Left)");
      configParam(NOTE_PARAM_R + i, 0.f, 1.f, 0.f,
                  std::string("Note ") + kNoteNames[i] +
                      " (Right quantizer, optional: phrases 2&4, or all 4 if Left is off)");
      configLight(NOTE_LIGHT_R + i, std::string("Note ") + kNoteNames[i] + " active (Right)");
    }

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

  // Genesis: master[] is now a random walk (each step = previous step +
  // a small musicalStep delta) rather than 16 independent uniform-random
  // values, so the shared "DNA" every phrase derives from has melodic
  // contour from the start instead of octave-jumping noise. The first
  // step still has no predecessor to walk from, so it stays a free
  // uniform pick. Every one of the 4 backing pool slots is then derived
  // from master (drift-gated per-step variation, using each phrase's own
  // drift) — this is the one point where all phrases share common DNA.
  static constexpr float kGenesisSigma = 0.3f;  // ~3.6 semitones std dev
  void genesis() {
    master[0] = random::uniform() * 5.f;
    for (int i = 1; i < 16; i++)
      master[i] = musicalStep(master[i - 1], kGenesisSigma);
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
    // selectPhrase() rolls the normal 15% Recall chance, but pool[] and
    // poolGenesis[] are identical right after reseedPhrase() above -
    // nothing has evolved yet to make "frozen snapshot" differ from
    // "current pool" - so a lit Recall light here would be technically
    // true but audibly meaningless, and just confusing. Force it off for
    // this first selection only; later phrase transitions still roll
    // normally once there's actually something to recall from.
    usingGenesisNow = false;
    // mutationFlash[] isn't reset anywhere else - without this, a Drift
    // light still mid-decay from before a stop/restart would carry over
    // and appear to indicate a mutation that hasn't actually happened yet
    // in this fresh run.
    for (int p = 0; p < 4; p++)
      mutationFlash[p] = 0.f;
    phraseTimer = 0.f;
    phrasesPlayedThisRound = 0;
    // clockPeriod isn't reset here (its correct value depends on the
    // incoming clock, which we haven't measured yet this run) - instead
    // phrase-transition timing is held off entirely until it's been
    // freshly measured at least once. See clockMeasured's declaration.
    sawFirstEdge = false;
    clockMeasured = false;
  }

  // Snaps volts (1V/oct) to the nearest note active in the given 12-entry
  // set (noteActiveL or noteActiveR), searching one octave above/below too
  // so notes near an octave boundary round to whichever neighboring active
  // note is actually closest (a same-octave-only search would wrongly
  // favor a distant low note over a near high one just across the
  // boundary, or vice versa). If nothing is toggled on (all notes off),
  // passes volts through unquantized rather than picking an arbitrary note
  // or dividing by zero degrees. outSemitoneClass, if given, receives the
  // 0-11 semitone actually picked (or -1 when nothing was active/no
  // quantization happened) - used to flash that note's light.
  float quantizeToActiveNotes(float volts, const bool *active, int *outSemitoneClass = nullptr) {
    float semis = volts * 12.f;
    float baseOct = std::floor(semis / 12.f) * 12.f;
    float best = 0.f;
    float bestDist = 1e9f;
    bool anyActive = false;
    for (int oct = -12; oct <= 12; oct += 12) {
      for (int d = 0; d < 12; d++) {
        if (!active[d]) continue;
        anyActive = true;
        float candidate = baseOct + oct + d;
        float dist = std::abs(semis - candidate);
        if (dist < bestDist) {
          bestDist = dist;
          best = candidate;
        }
      }
    }
    if (outSemitoneClass)
      *outSemitoneClass = anyActive ? (((int)best % 12) + 12) % 12 : -1;
    return anyActive ? best / 12.f : volts;
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

  // The one "musical mutation" primitive, used everywhere a step deviates
  // from a reference value (random-walk genesis, phrase reseed, and
  // ongoing evolution) - a small gaussian-distributed delta around the
  // reference, clamped back into the module's 0-5V pitch range, instead
  // of discarding the reference for an unrelated fresh random value.
  // sigma is in volts (1V/oct, so e.g. 0.25f is a 3-semitone std dev).
  // random::normal() has unbounded tails, so without a hard cap a rare
  // 3+ sigma roll could still land as a jarring jump ("bounded/gaussian"
  // in the design notes means both - gaussian shape, hard-capped tails,
  // not gaussian alone) - reported as "crazy octave jumps" on real
  // hardware before this cap was added.
  static constexpr float kMaxDeltaSigmas = 2.f;
  float musicalStep(float value, float sigma) {
    float delta = clamp(random::normal() * sigma, -sigma * kMaxDeltaSigmas, sigma * kMaxDeltaSigmas);
    return clamp(value + delta, 0.f, 5.f);
  }

  // pool[]/poolGenesis[] are always a fixed 16-slot chain (each step built
  // from the previous), but playback treats them as a closed loop of the
  // current, user-adjustable Length - step len-1 wraps straight back to
  // step 0 every time through. Those two slots have no continuity
  // relationship in the chain (reported on real hardware as "a wild
  // octave jump" at every loop repeat, not just once). This re-derives
  // the wrap step from step 0 using the same musicalStep sizing as any
  // other adjacent-step deviation, so the seam reads like a normal
  // transition instead of a bigger jump than the rest of the melody.
  // Must be re-run any time len changes (the wrap point itself moves) or
  // the array's content changes (reseed, mutation) - not just once at
  // genesis.
  void closeLoopSeam(float *arr) {
    if (len <= 1) return;  // no wrap to smooth for a 1-step "loop"
    arr[len - 1] = musicalStep(arr[0], kMutationSigma);
  }

  // Anchor steps (the downbeat and its halfway point) mutate less often
  // than the steps between them, so evolving phrases keep a stable
  // rhythmic/harmonic backbone instead of drifting unrecognizably even at
  // high Entropy. Scaled to the current Length (len/2) rather than a
  // hardcoded step 8, since Length is user-adjustable (2-16) and a fixed
  // step 8 would be meaningless - or not even played - at shorter lengths.
  static constexpr float kAnchorMutationScale = 0.35f;
  bool isAnchorStep(int i) {
    return i == 0 || i == len / 2;
  }

  // Re-derives a single phrase from the current master (not a fresh one),
  // gated by that phrase's own entropy (drift half). Used both by genesis()
  // and when a phrase's Active toggle goes off->on, so phrases activated
  // later still share the same lineage as those already playing. The
  // result is also saved as that phrase's Recall snapshot.
  static constexpr float kMutationSigma = 0.15f;  // ~1.8 semitones std dev
  void reseedPhrase(int p) {
    float driftP = entropyOf(p);
    for (int i = 0; i < 16; i++) {
      float stepDriftP = isAnchorStep(i) ? driftP * kAnchorMutationScale : driftP;
      pool[p][i] = (random::uniform() < stepDriftP) ? musicalStep(master[i], kMutationSigma) : master[i];
      poolGenesis[p][i] = pool[p][i];
    }
    closeLoopSeam(pool[p]);
    poolGenesis[p][len - 1] = pool[p][len - 1];  // keep them identical, same as every other slot above
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
        // Perturbs whatever crossover already produced for this step by a
        // small musicalStep delta, instead of discarding it for an
        // unrelated fresh random value - keeps mutations sounding like
        // variations on the existing melodic material. Anchor steps
        // mutate less often (see isAnchorStep).
        float stepDriftP = isAnchorStep(i) ? driftP * kAnchorMutationScale : driftP;
        if (random::uniform() < stepDriftP)
          candidates[p][i] = musicalStep(candidates[p][i], kMutationSigma);
      }
    }
    // Below the threshold, the persist check can still pass (structurally
    // "replaced") even though crossover+mutation happened to land back on
    // essentially the same values - e.g. low Entropy, or a crossover
    // partner that was already very similar. Flashing Drift for that
    // would tell the user something changed when nothing audible did.
    const float kAudibleChangeThreshold = 0.01f;  // ~0.12 semitones
    for (int k = 0; k < n; k++) {
      int p = activeIdx[k];
      float persistP = 1.f - entropyRoll[p];
      if (random::uniform() >= persistP) {
        bool audiblyChanged = false;
        for (int i = 0; i < 16; i++) {
          if (std::abs(candidates[p][i] - pool[p][i]) > kAudibleChangeThreshold) {
            audiblyChanged = true;
            break;
          }
        }
        for (int i = 0; i < 16; i++)
          pool[p][i] = candidates[p][i];
        closeLoopSeam(pool[p]);
        if (audiblyChanged)
          mutationFlash[p] = 1.f;
      }
    }
  }

  void process(const ProcessArgs &args) override {
    len = clamp((int)std::round(params[LENGTH_PARAM].getValue()), 1, 16);
    if (len != prevLen) {
      prevLen = len;
      // The wrap point moved - re-close every phrase's seam at the new
      // len-1, not just whichever phrase happens to reseed/mutate next.
      for (int p = 0; p < 4; p++) {
        closeLoopSeam(pool[p]);
        closeLoopSeam(poolGenesis[p]);
      }
    }

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

    // Light brightness itself is set later (with the mutationFlash-style
    // decay loop below), once this step's played note (if any) is known -
    // just track on/off state and the anyActive flags here.
    bool anyActiveL = false, anyActiveR = false;
    for (int i = 0; i < 12; i++) {
      if (noteTrigL[i].process(params[NOTE_PARAM_L + i].getValue()))
        noteActiveL[i] = !noteActiveL[i];
      anyActiveL = anyActiveL || noteActiveL[i];

      if (noteTrigR[i].process(params[NOTE_PARAM_R + i].getValue()))
        noteActiveR[i] = !noteActiveR[i];
      anyActiveR = anyActiveR || noteActiveR[i];
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
      if (sawFirstEdge) {
        // timeSinceClock is now a genuine gap between two real pulses.
        clockPeriod = timeSinceClock;
        clockMeasured = true;
      } else {
        // This edge only establishes a starting point - timeSinceClock
        // up to here was measured from genesis() (when Run was pressed),
        // not from a previous pulse, so it isn't a real period.
        sawFirstEdge = true;
      }
      timeSinceClock = 0.f;
    }

    if (running) {
      int activeCount = 0;
      for (int p = 0; p < 4; p++)
        if (phraseActive[p]) activeCount++;

      if (activeCount > 0 && clockMeasured) {
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
          // Both quantizers are optional (all lights off = ignored, CV
          // passes through unquantized - see quantizeToActiveNotes). When
          // only one side has any notes active, that lone side governs all
          // 4 phrases rather than just its own half of the pool grid -
          // only when BOTH sides are actually in use does the normal
          // left/right split (matching poolColLeft[p % 2] in the widget:
          // phrases 1&3 vs 2&4) apply.
          const bool *quantSet;
          bool quantIsLeft;
          if (anyActiveL && !anyActiveR) {
            quantSet = noteActiveL;
            quantIsLeft = true;
          } else if (anyActiveR && !anyActiveL) {
            quantSet = noteActiveR;
            quantIsLeft = false;
          } else {
            quantIsLeft = (currentPhraseIdx % 2 == 0);
            quantSet = quantIsLeft ? noteActiveL : noteActiveR;
          }
          int playedSemitone = -1;
          heldCV = quantizeToActiveNotes(playingSeq[step], quantSet, &playedSemitone);
          if (playedSemitone >= 0)
            (quantIsLeft ? playFlashL : playFlashR)[playedSemitone] = 1.f;
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
      // Stretched from 4s to 7s (plus the MediumLight bump above) after
      // real-hardware testing confirmed a 4s SmallLight flash was still
      // easy to miss.
      const float mutationDecayTime = 7.f;
      mutationFlash[p] -= args.sampleTime / mutationDecayTime;
      if (mutationFlash[p] < 0.f) mutationFlash[p] = 0.f;
      // Hidden while Recall is showing for this same phrase - "the pool
      // just mutated" is confusing/contradictory information to display
      // at the exact moment you're actually hearing the untouched frozen
      // original instead of that mutation. mutationFlash itself keeps
      // decaying underneath, so if Recall lets go before it's fully
      // decayed, Drift can still reappear and finish its flash normally.
      bool recallShowing = (p == currentPhraseIdx && usingGenesisNow);
      lights[DRIFT_LIGHT + p].setBrightness(recallShowing ? 0.f : mutationFlash[p]);
    }

    // Note lights: dim while merely toggled on, full-bright and decaying
    // (same peak/clip LED idea as mutationFlash above, but much shorter -
    // notes can play every step, not every few minutes) the instant that
    // note is actually played, so "enabled" and "currently sounding" read
    // as visually distinct.
    const float noteFlashDecayTime = 0.25f;
    const float noteBaseBrightness = 0.25f;
    for (int i = 0; i < 12; i++) {
      playFlashL[i] -= args.sampleTime / noteFlashDecayTime;
      if (playFlashL[i] < 0.f) playFlashL[i] = 0.f;
      lights[NOTE_LIGHT_L + i].setBrightness(std::max(noteActiveL[i] ? noteBaseBrightness : 0.f, playFlashL[i]));

      playFlashR[i] -= args.sampleTime / noteFlashDecayTime;
      if (playFlashR[i] < 0.f) playFlashR[i] = 0.f;
      lights[NOTE_LIGHT_R + i].setBrightness(std::max(noteActiveR[i] ? noteBaseBrightness : 0.f, playFlashR[i]));
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

  // Persists the quantizer note toggles across patch save/load - these
  // live outside the auto-saved params (NOTE_PARAM_L/R are just momentary
  // edge sources, not the actual on/off state, same as ACTIVE_PARAM/
  // phraseActive) so without this they'd silently reset to all-off every
  // time a patch reopens.
  json_t *dataToJson() override {
    json_t *rootJ = json_object();
    json_t *noteLJ = json_array();
    json_t *noteRJ = json_array();
    for (int i = 0; i < 12; i++) {
      json_array_append_new(noteLJ, json_boolean(noteActiveL[i]));
      json_array_append_new(noteRJ, json_boolean(noteActiveR[i]));
    }
    json_object_set_new(rootJ, "noteActiveL", noteLJ);
    json_object_set_new(rootJ, "noteActiveR", noteRJ);

    json_t *phraseJ = json_array();
    for (int p = 0; p < 4; p++)
      json_array_append_new(phraseJ, json_boolean(phraseActive[p]));
    json_object_set_new(rootJ, "phraseActive", phraseJ);

    return rootJ;
  }

  void dataFromJson(json_t *rootJ) override {
    json_t *noteLJ = json_object_get(rootJ, "noteActiveL");
    if (noteLJ) {
      for (int i = 0; i < 12 && i < (int)json_array_size(noteLJ); i++)
        noteActiveL[i] = json_boolean_value(json_array_get(noteLJ, i));
    }
    json_t *noteRJ = json_object_get(rootJ, "noteActiveR");
    if (noteRJ) {
      for (int i = 0; i < 12 && i < (int)json_array_size(noteRJ); i++)
        noteActiveR[i] = json_boolean_value(json_array_get(noteRJ, i));
    }

    json_t *phraseJ = json_object_get(rootJ, "phraseActive");
    if (phraseJ) {
      for (int p = 0; p < 4 && p < (int)json_array_size(phraseJ); p++)
        phraseActive[p] = json_boolean_value(json_array_get(phraseJ, p));
    }
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
    // A real bold weight, not the blur-based fake-bold this file uses
    // elsewhere (EvoStaticLabel) - that trick just softens edges and reads
    // as fuzzy, not genuinely thick, at this size. Nunito-Bold ships with
    // Rack itself (and with the MetaModule SDK, firmware v2.0+) as a
    // system font, so no font file needs to be bundled with this plugin.
    // loadFont() caches by path internally, so calling it every draw() is
    // the normal Rack idiom, not a per-frame disk read. (A DSEG7 seven-
    // segment digital-display font was tried here too - reverted, it read
    // as a calculator-clock look rather than "thick".)
    std::shared_ptr<Font> boldFont = APP->window->loadFont(asset::system("res/fonts/Nunito-Bold.ttf"));
    nvgFontFaceId(args.vg, boldFont ? boldFont->handle : APP->window->uiFont->handle);
    // Bumped up from 0.72 - digit glyphs are narrower than the box is
    // tall, so there's real headroom before width becomes the limit.
    float fontSize = box.size.y * 0.85f;
    nvgFontSize(args.vg, fontSize);
    float cx = box.size.x * 0.5f;
    float cy = box.size.y * 0.5f;

#ifdef METAMODULE
    // MetaModule's firmware nvgTextBounds()/nvgTextMetrics() are stubbed
    // (always return zero - the real fontstash-based measurement code is
    // #if(0)'d out in its nanovg.c), so the glyph-bounds-based centering
    // below can't work there - confirmed on hardware: a hand-picked fixed
    // offset (fontSize * 0.34) undershot and still overlapped the knobs
    // above. NVG_ALIGN_MIDDLE is handled inside nvgText()'s own draw path
    // (which IS fully implemented on MetaModule, unlike the separate
    // measurement APIs), using the font's real ascender/descender metrics
    // rather than a guessed constant - less precise per-glyph than the
    // desktop approach below (the original reason MIDDLE was dropped: "1"
    // and "3" sat at very slightly different heights), but reliable
    // without hardware-tuned magic numbers.
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    float baselineY = cy;
#else
    // NVG_ALIGN_MIDDLE centers on the font's ascender/descender metrics,
    // not this specific glyph's own visual shape - digits like "1" render
    // with a different apparent top/bottom than "3" or "4" as a result, so
    // each pool's numeral looked inconsistently positioned against the
    // others. Measuring this glyph's actual bounding box and centering on
    // THAT instead makes every digit's visual top/bottom land in the same
    // place across all 4 boxes.
    nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
    float bounds[4];
    nvgTextBounds(args.vg, cx, 0.f, text.c_str(), nullptr, bounds);
    float glyphMidY = (bounds[1] + bounds[3]) * 0.5f;
    float baselineY = cy - glyphMidY;
#endif

    // Outline temporarily disabled to preview the numeral without it -
    // re-enable this block to restore the light-gray stamped outline.
#if 0
    const float outlineWidth = fontSize * 0.02f;
    const int steps = 16;
    nvgFillColor(args.vg, nvgRGB(0xaa, 0xaa, 0xaa));
    for (int i = 0; i < steps; i++) {
      float angle = 6.28318530718f * (float)i / (float)steps;
      float ox = cx + std::cos(angle) * outlineWidth;
      float oy = baselineY + std::sin(angle) * outlineWidth;
      nvgText(args.vg, ox, oy, text.c_str(), nullptr);
    }
#endif

    // Beyond Nunito-Bold's own weight, stamp the same solid fill at several
    // tiny offsets around center before the final centered draw - this
    // dilates the strokes further (a standard faux-extra-bold technique)
    // while staying crisp, unlike nvgFontBlur which just softens edges.
    nvgFillColor(args.vg, color);
    const float dilate = fontSize * 0.015f;
    const int dilateSteps = 8;
    for (int i = 0; i < dilateSteps; i++) {
      float angle = 6.28318530718f * (float)i / (float)dilateSteps;
      float ox = cx + std::cos(angle) * dilate;
      float oy = baselineY + std::sin(angle) * dilate;
      nvgText(args.vg, ox, oy, text.c_str(), nullptr);
    }
    nvgText(args.vg, cx, baselineY, text.c_str(), nullptr);
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
    // Neutral black (was a reddish #280b0b) so labels read consistently
    // sitting on top of all 4 differently-hued pools, rather than quietly
    // favoring pool1's red family - matches EvoLengthDisplay's "8 Steps"
    // text, which already used plain black.
    nvgFillColor(args.vg, nvgRGB(0x00, 0x00, 0x00));
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

    // Left lane = shared/global playback controls, horizontally centered
    // in the lane box (raw x=7 to 91, center raw x=49 -> 16.596mm) - was
    // left off-center at 11.629mm from when the lane was still only
    // 56 raw px wide (before it was widened to 84 to match the pool
    // columns).
    const float laneXL = 16.596f;
    const float cvDX   = 4.5f;

    const float runY      = 15.5f;
    const float lengthY   = 33.f;
    const float dispY     = 37.f;
    // BPM knob is gone - the module is externally clocked now (see Clock
    // In below). Jitter and Rise/Fall were evenly spaced (23mm steps)
    // between Length and the output row; Jitter (and its label, which
    // follows it) then nudged down 4mm further per feedback.
    const float jitterY   = 60.f;
    const float riseFallY = 79.f;
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
    // Panel widened from 258 to 286 raw px to let the red lane match the
    // pool columns' own width (see laneLeft in the SVG) - both columns
    // shifted +28 raw px right to make room (old raw x=73/167 -> 101/195).
    const float poolColLeft[2] = {34.206f, 66.033f};      // left edge of each pool column (raw x=101, 195)
    // Row 2 (pool3/4) is lifted 6 raw px (2.032mm) closer to row 1, closing
    // up the gap under it to make room for the quantizer's own grey square
    // below - see the matching pool3/pool4 rect y in the SVG (126 -> 120).
    const float row2Lift = 2.032f;
    const float rowAY[2] = {runY, runY + 32.f - row2Lift};      // Active+Entropy row, per pool-grid-row
    const float rowBY[2] = {rowAY[0] + 16.f, rowAY[1] + 16.f};  // Recall+Drift row, per pool-grid-row

    // Pool box bounds (matching res/TheReelPeetEvo.svg's pool1-4 rects,
    // raw px / 2.9528) for the background numerals below.
    const float poolRowTop[2] = {8.467f, 42.667f - row2Lift};  // top edge of each pool row (raw y=25, 120)
    const float poolBoxW = 28.451f;                 // raw width 84
    const float poolBoxH = 31.6f;                   // raw height ~93-94

    // Large background numerals, added before the per-phrase controls
    // below so they render behind the knobs/lights, not on top of them.
    // Solid, fully opaque pastel tints - each column's own color (blue for
    // 1&3, orange for 2&4 - see poolColL/poolColR in the SVG) blended
    // toward white. Blue nudged from 25% to 35% now that the numerals are
    // bigger/bolder and read heavier at the same color; orange left at
    // 50% since that one wasn't flagged.
    NVGcolor poolColors[4] = {
      nvgRGB(0x89, 0xa7, 0xe6), nvgRGB(0xec, 0xc1, 0xa5),
      nvgRGB(0x89, 0xa7, 0xe6), nvgRGB(0xec, 0xc1, 0xa5),
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

    // Quantizer: two independent 12-per-semitone toggle grids, one per pool
    // grid column (poolColLeft[0]/[1]) - left governs phrases 1&3, right
    // governs phrases 2&4 (see quantSet in process()). Each is laid out
    // like a piano keyboard turned on its side - 7 "white key" buttons
    // (naturals) in a vertical column, C at the bottom rising to B at the
    // top (mirroring low-to-high pitch reading bottom-to-top), with the 5
    // "black key" buttons (sharps) offset to the left, each sitting at
    // the vertical midpoint between the two naturals it falls between
    // (none between E/F or B/C, same as a real keyboard). No text labels -
    // the black/white key positions are the identification, same as on
    // the hardware this is modeled after (Intellijel Scales).
    // Nudged up (raw y=223 -> 216, i.e. 80mm -> 77.63mm here) to sit closer
    // under pools 3/4, while keeping the same clearance from the
    // background rect's top edge (see quantizerBgL/R in the SVG) so the
    // topmost button (B) still doesn't poke above it.
    const float whiteTopY = 77.63f;   // y of the topmost row (B); C lands at the bottom
    const float whitePitchY = 6.85f;
    auto addPianoColumn = [&](float centerX, int paramBase, int lightBase) {
      float whiteX = centerX + 4.f;
      float blackX = whiteX - 8.f;
      const int whiteSemis[7] = {0, 2, 4, 5, 7, 9, 11};        // C D E F G A B
      const int blackSemis[5] = {1, 3, 6, 8, 10};              // C# D# F# G# A#
      const int blackAfterWhite[5] = {0, 1, 3, 4, 5};          // black[k] sits between white[idx] and white[idx+1]
      float whiteRowY[7];
      for (int i = 0; i < 7; i++) {
        whiteRowY[i] = whiteTopY + (6 - i) * whitePitchY;
        addParam(createParamCentered<LEDButton>(mm2px(Vec(whiteX, whiteRowY[i])), module, paramBase + whiteSemis[i]));
        addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(whiteX, whiteRowY[i])), module, lightBase + whiteSemis[i]));
      }
      for (int i = 0; i < 5; i++) {
        float by = (whiteRowY[blackAfterWhite[i]] + whiteRowY[blackAfterWhite[i] + 1]) * 0.5f;
        addParam(createParamCentered<LEDButton>(mm2px(Vec(blackX, by)), module, paramBase + blackSemis[i]));
        addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(blackX, by)), module, lightBase + blackSemis[i]));
      }
    };
    addPianoColumn(poolColLeft[0] + 14.226f, TheReelPeetEvo::NOTE_PARAM_L, TheReelPeetEvo::NOTE_LIGHT_L);
    addPianoColumn(poolColLeft[1] + 14.226f, TheReelPeetEvo::NOTE_PARAM_R, TheReelPeetEvo::NOTE_LIGHT_R);

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
      // Bumped from SmallLight - both are meaningful events (Recall: this
      // phrase is playing its frozen snapshot; Drift: it just mutated),
      // easy to miss as small dots.
      addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(ctrlX0, rowBY[row])), module, TheReelPeetEvo::RECALL_LIGHT + p));
      addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(ctrlX1, rowBY[row])), module, TheReelPeetEvo::DRIFT_LIGHT + p));
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
        // Bumped from 7pt to match Entropy's size, alongside the light
        // size bump above - Recall/Drift are meaningful events, not
        // low-priority sub-labels.
        addLabel("Recall", ctrlX0, rowBY[row] + 2.5f, dispW, 8.f);
        addLabel("Drift", ctrlX1, rowBY[row] + 2.5f, dispW, 8.f);
      }

      auto *lenDisplay = new EvoLengthDisplay;
      lenDisplay->box.pos = mm2px(Vec(laneXL - dispW * 0.5f, dispY));
      lenDisplay->box.size = mm2px(Vec(dispW, 11.f));
      lenDisplay->value = &module->len;
      addChild(lenDisplay);

    }
  }
};
#pragma clang optimize on

Model *modelTheReelPeetEvo =
    createModel<TheReelPeetEvo, TheReelPeetEvoWidget>("thereelpeet-evo");
