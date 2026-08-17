#include "plugin.hpp"
#include "EvoGruWeights.h"

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
// module briefly had was dropped entirely (its knob was repurposed for
// Jitter, which has since also been removed - see entropyOf).
//
// No internal BPM knob or free-running clock - the module is externally
// clocked via Clock In. Each incoming pulse advances one step; with
// nothing patched in, steps simply don't advance (same as any other
// clocked Eurorack sequencer). Phrase Duration is likewise driven by
// actual clock edges (counting step-wraps, see loopsCompleted), not a
// measured clock period - guarantees phrase transitions always land
// exactly on a phrase boundary regardless of clock jitter.
//
// Phrase-cycling design: pool[4][16] holds up to 4 phrases, each with its
// own Active toggle and Entropy.
//
// Entropy is a single per-phrase knob spanning drift (mutation rate) and
// persist (replacement resistance) as one inverse-coupled axis: CW = more
// drift + less persist (changes readily), CCW = less drift + more persist
// (nearly frozen). drift = entropy, persist = 1 - entropy.
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
    ENGINE_PARAM,   // was JITTER_PARAM - Jitter dropped, this slot repurposed for mutation-engine select
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
  enum InputId { CLOCK_INPUT, LINK_INPUT, INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, ENV_OUTPUT, OUTPUTS_LEN };
  enum LightId {
    RUN_LIGHT,
    ACTIVE_LIGHT,                      // ACTIVE_LIGHT + 0..3
    RECALL_LIGHT = ACTIVE_LIGHT + 4,   // RECALL_LIGHT + 0..3
    DRIFT_LIGHT = RECALL_LIGHT + 4,    // DRIFT_LIGHT + 0..3
    ENGINE_LIGHT = DRIFT_LIGHT + 4,    // ENGINE_LIGHT + 0..3, one per Engine value
    NOTE_LIGHT_L = ENGINE_LIGHT + 4,   // NOTE_LIGHT_L + 0..11
    NOTE_LIGHT_R = NOTE_LIGHT_L + 12,  // NOTE_LIGHT_R + 0..11
    LIGHTS_LEN = NOTE_LIGHT_R + 12
  };
  enum EnvPhase { ENV_IDLE, ENV_ATTACK, ENV_DECAY };
  // Global mutation engine, selectable via ENGINE_PARAM's cycling button -
  // affects only the mutation-sigma-scale deviation in reseedPhrase(),
  // evolveAllPhrases(), and closeLoopSeam(); genesis()'s own master[] walk
  // always stays Gaussian regardless (see mutateStep()). ENGINE_GRU is a
  // real trained model (see EvoGruWeights.h/gruStep) - named for the
  // technique, not branded "AI", same as Markov isn't either.
  enum Engine { ENGINE_GAUSSIAN, ENGINE_MARKOV, ENGINE_INTERVAL, ENGINE_GRU, NUM_ENGINES };

  bool running = false;
  int step = 0;
  float trigTimer = 0.f;
  float heldCV = 0.f;
  float envLevel = 0.f;
  int envPhase = ENV_IDLE;
  // A tie must pause the envelope ramp itself, not just skip the retrigger -
  // envLevel advances every sample purely off envPhase, so without this a
  // tie landing after a short Fall time is inaudible (decay already
  // finished by then, so "don't retrigger" just continues silence).
  bool envFrozen = false;
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
  // Zero-initialized so a reseedPhrase() call that reads pool[p] mid-loop
  // (markovStep looks at the whole array while still filling it in) never
  // reads indeterminate memory for slots this pass hasn't written yet.
  float pool[4][16] = {};
  float poolGenesis[4][16] = {};  // frozen snapshot of each phrase at genesis/reseed time, for Recall
  // Rest/tie phrasing, baked into the pool as genetic material - crosses
  // over and mutates alongside pitch instead of being a fresh dice roll
  // every playthrough (see reseedPhrase/evolveAllPhrases). master[] has
  // no type array of its own: the shared lineage is always implicitly
  // STEP_NORMAL (pure pitch, no rests/ties), which only emerge through a
  // phrase's own entropy-driven drift, same as pitch deviation does.
  enum StepType { STEP_NORMAL, STEP_REST, STEP_TIE };
  int poolType[4][16] = {};
  int poolGenesisType[4][16] = {};
  bool phraseActive[4] = {true, true, true, true};
  // User choice, not per-session state - deliberately not reset by
  // genesis() the way heldCV/mutationFlash/etc are, so it persists across
  // Run presses like phraseActive does.
  int mutationEngine = ENGINE_GAUSSIAN;
  float blinkPhase = 0.f;  // drives the currently-playing light's pulse; independent of Phrase Duration
  int currentPhraseIdx = 0;
  bool usingGenesisNow = false;  // this playthrough's Recall roll for currentPhraseIdx
  // Counts actual step-wraps (whole passes through the sequence) rather
  // than elapsed wall-clock time, so a phrase transition always lands
  // exactly on a phrase boundary (step 0) regardless of clock jitter -
  // see the clockEdge block in process() for why a time-based version
  // (the old phraseTimer/phraseDuration/clockPeriod approach) couldn't
  // guarantee that.
  int loopsCompleted = 0;
  int phrasesPlayedThisRound = 0;
  float mutationFlash[4] = {0.f, 0.f, 0.f, 0.f};  // decays after a phrase's pool actually gets replaced
  // evolveAllPhrases() runs once per full round, mutating every active
  // phrase's pool at once regardless of which one is currently playing -
  // flashing mutationFlash immediately there meant a phrase's Drift light
  // could light up while a totally different phrase was audibly playing,
  // with no correlation to what's on screen. pendingDrift records "this
  // phrase changed since it last played" and selectPhrase() converts that
  // into an actual flash only once that phrase becomes current - so the
  // light always means "what you're about to hear differs from last time."
  bool pendingDrift[4] = {false, false, false, false};
  // Flashes the just-played note's light on top of its steady "enabled"
  // brightness, decaying like mutationFlash above - lets you see which
  // in-scale note actually got picked each step, not just which notes are
  // toggled on.
  float playFlashL[12] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  float playFlashR[12] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

  dsp::SchmittTrigger onTrig;
  dsp::SchmittTrigger activeTrig[4];
  dsp::SchmittTrigger engineTrig;
  dsp::SchmittTrigger noteTrigL[12];
  dsp::SchmittTrigger noteTrigR[12];
  dsp::SchmittTrigger clockTrig;

  TheReelPeetEvo() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

    configParam(RUN_PARAM, 0.f, 1.f, 0.f, "Run toggle");
    configParam(LENGTH_PARAM, 2.f, 16.f, 8.f, "Length (2-16 steps)");
    // Momentary button, same pattern as ACTIVE_PARAM - the param itself is
    // just an edge source (see engineTrig in process()), the actual choice
    // lives in mutationEngine and cycles Gaussian -> Markov -> Interval.
    configParam(ENGINE_PARAM, 0.f, 1.f, 0.f, "Cycle mutation engine (Gaussian/Markov/Interval/AI)");
    // Rise's range used to be half of Fall's (0-2s vs 0-4s) - same knob
    // position meant very different times between the two, confusing.
    // Matched to Fall's 0-4s range so the pair reads consistently.
    configParam(RISE_PARAM, 0.f, 4.f, 0.f, "Rise time", " s");
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
                  "Phrase " + n + " entropy (CW: more drift/less persist/more rests+ties, CCW: less drift/more persist/fewer rests+ties)",
                  "%", 0.f, 100.f);
      configLight(ACTIVE_LIGHT + p, "Phrase " + n + " active");
      configLight(RECALL_LIGHT + p, "Phrase " + n + " recall active");
      configLight(DRIFT_LIGHT + p, "Phrase " + n + " drift active");
    }

    configInput(CLOCK_INPUT, "Clock");
    configInput(LINK_INPUT, "Link CV (1V/oct external pitch to gently bias this module's mutations toward)");
    configOutput(OUT_OUTPUT, "Pitch CV (1V/Oct)");
    configOutput(ENV_OUTPUT, "Envelope CV (0-10V)");
    configLight(RUN_LIGHT, "Running");
    configLight(ENGINE_LIGHT + (int)ENGINE_GAUSSIAN, "Gaussian engine active");
    configLight(ENGINE_LIGHT + (int)ENGINE_MARKOV, "Markov engine active");
    configLight(ENGINE_LIGHT + (int)ENGINE_INTERVAL, "Interval-locked engine active");
    configLight(ENGINE_LIGHT + (int)ENGINE_GRU, "AI (experimental) engine active - a small trained GRU model");

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
  // Bumped from 0.3 - a smooth walk keeps adjacent raw steps close
  // together by design (the whole point, to avoid octave-jumping noise),
  // but against a sparse quantizer note selection that closeness let
  // multiple adjacent steps collapse onto the same nearest active note
  // (reported on real hardware as 6 identical notes out of 8). More raw
  // spread makes that collapse less likely without reintroducing "crazy"
  // jumps, since the hard 2-sigma delta cap (see musicalStep) still holds.
  static constexpr float kGenesisSigma = 0.45f;  // ~5.4 semitones std dev, ~10.8 semitone cap
  void genesis() {
    // A full 0-5V uniform draw (5 octaves) had no bias toward a "home"
    // register - every Run press could just as easily start at the very
    // top or bottom of the pitch range as the middle, and the rest of the
    // walk stays clustered near wherever step 0 landed (reported as
    // "crazy high" on one Run, "crazy low" after a restart). Narrowed to a
    // centered 2-octave window instead - still varies run to run, just not
    // at the literal extremes.
    master[0] = 1.5f + random::uniform() * 2.f;
    // applyHomeGravity and applyLink here too (not just in mutateStep) so
    // genesis's own walk stays less likely to wander to an extreme by its
    // own end, and a linked module's very first melody already leans
    // toward whatever's on Link In, rather than only picking up either
    // effect once mutations start happening.
    for (int i = 1; i < 16; i++)
      master[i] = applyLink(applyHomeGravity(musicalStep(master[i - 1], kGenesisSigma)));
    for (int p = 0; p < 4; p++)
      reseedPhrase(p);

    int firstActive = 0;
    for (int p = 0; p < 4; p++) {
      if (phraseActive[p]) {
        firstActive = p;
        break;
      }
    }
    // Reset before selectPhrase() below, which checks pendingDrift[idx] -
    // a stale true left over from a previous run could otherwise trigger
    // an unearned Drift flash on the very first phrase of a fresh start.
    for (int p = 0; p < 4; p++)
      pendingDrift[p] = false;
    selectPhrase(firstActive);
    // mutationFlash[] isn't reset anywhere else - without this, a Drift
    // light still mid-decay from before a stop/restart would carry over
    // and appear to indicate a mutation that hasn't actually happened yet
    // in this fresh run.
    for (int p = 0; p < 4; p++)
      mutationFlash[p] = 0.f;
    // heldCV isn't reset by the "stopped" branch of process() either (only
    // envPhase/envLevel/step are) - without this, a stale pitch from
    // before this run (e.g. wherever a previous session left off) could
    // bleed through audibly if the very first step happens to roll a tie,
    // since a tie deliberately leaves heldCV untouched rather than reading
    // a fresh value.
    heldCV = 0.f;
    envFrozen = false;
    loopsCompleted = 0;
    phrasesPlayedThisRound = 0;
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

  float entropyOf(int p) {
    return params[ENTROPY_PARAM + p].getValue();
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
    float result = value + delta;
    // Reflects off the 0-5V boundaries instead of hard-clamping. A plain
    // clamp() piles up repeated near-boundary overshoots at exactly 0 or
    // 5, since every excursion past the edge maps to the same clamped
    // value regardless of how far it overshot - a classic artifact of
    // clamping a bounded random walk, and worse the bigger sigma is.
    // Reported on real hardware as genesis getting stuck on a high pitch
    // right from the start once kGenesisSigma was increased. Reflecting
    // preserves the overshoot's magnitude as movement back into range
    // instead of truncating it away.
    if (result > 5.f) result = 5.f - (result - 5.f);
    if (result < 0.f) result = -result;
    // Defensive fallback only - with kMaxDeltaSigmas capping delta well
    // under the full 0-5 range, a single reflection can't overshoot the
    // opposite boundary too, but clamp anyway rather than trust that math
    // silently forever.
    return clamp(result, 0.f, 5.f);
  }

  // 0-11 pitch class of a raw 1V/oct value, wrapping negatives correctly
  // (plain % can return negative results in C++ for negative operands).
  int semitoneClass(float volts) {
    int semis = (int)std::round(volts * 12.f);
    return ((semis % 12) + 12) % 12;
  }

  // Places targetClass at whichever octave keeps it closest to value -
  // same nearest-candidate search as quantizeToActiveNotes, but for one
  // fixed class instead of a whole active-note set. Used by markovStep so
  // a chosen class lands as a small jump from the current value instead of
  // a random octave, matching musicalStep's own "small deviation" feel.
  float nearestOctaveForClass(float value, int targetClass) {
    float semis = value * 12.f;
    float baseOct = std::floor(semis / 12.f) * 12.f;
    float best = 0.f;
    float bestDist = 1e9f;
    for (int oct = -12; oct <= 12; oct += 12) {
      float candidate = baseOct + oct + targetClass;
      float dist = std::abs(semis - candidate);
      if (dist < bestDist) {
        bestDist = dist;
        best = candidate;
      }
    }
    return clamp(best / 12.f, 0.f, 5.f);
  }

  // Built live from phrase p's own current 16-slot pool, no persistent
  // table and no external corpus - counts observed class-to-class
  // transitions and samples the next class from that distribution, so
  // drift leans toward whatever intervals this specific melody already
  // favors instead of symmetric randomness. With only 16 steps of data
  // this is necessarily sparse (closer to "locally weighted flavor" than
  // a trained model) - if the current class has no observed transitions
  // yet (e.g. right after genesis), falls back to Gaussian.
  float markovStep(float value, int p) {
    int fromClass = semitoneClass(value);
    int counts[12] = {0};
    int total = 0;
    for (int i = 0; i < 15; i++) {
      if (semitoneClass(pool[p][i]) == fromClass) {
        counts[semitoneClass(pool[p][i + 1])]++;
        total++;
      }
    }
    if (total == 0)
      return musicalStep(value, kMutationSigma);
    int pick = (int)(random::uniform() * total);
    int toClass = 0;
    for (int c = 0; c < 12; c++) {
      if (pick < counts[c]) { toClass = c; break; }
      pick -= counts[c];
    }
    return nearestOctaveForClass(value, toClass);
  }

  // Which quantizer set governs phrase p's playback - factored out of
  // process()'s note-read logic so intervalStep can use the exact same
  // left/right selection rule instead of a second copy of it. Mirrors
  // quantizeToActiveNotes' "unquantized if nothing active" fallback: when
  // only one side has any notes active, that lone side governs all 4
  // phrases; only when both are in use does the normal left/right split
  // (phrases 1&3 vs 2&4) apply.
  const bool *quantSetForPhrase(int p) {
    bool anyActiveL = false, anyActiveR = false;
    for (int i = 0; i < 12; i++) {
      anyActiveL = anyActiveL || noteActiveL[i];
      anyActiveR = anyActiveR || noteActiveR[i];
    }
    if (anyActiveL && !anyActiveR) return noteActiveL;
    if (anyActiveR && !anyActiveL) return noteActiveR;
    return (p % 2 == 0) ? noteActiveL : noteActiveR;
  }

  // Mutates by whole scale-degree steps within phrase p's active quantizer
  // notes instead of a continuous voltage delta, so drift stays
  // diatonic-sounding by construction even at high Entropy. Falls back to
  // Gaussian when nothing's toggled on for this phrase's side (nothing to
  // lock to).
  float intervalStep(float value, int p) {
    const bool *active = quantSetForPhrase(p);
    // Build a sorted lattice of absolute semitone positions (spanning a
    // couple octaves either side of value) for every active class, so
    // "step by one scale degree" is just an index move in this list.
    float lattice[36];
    int n = 0;
    float baseOct = std::floor(value * 12.f / 12.f) * 12.f;
    for (int oct = -24; oct <= 24; oct += 12) {
      for (int d = 0; d < 12; d++) {
        if (!active[d]) continue;
        lattice[n++] = baseOct + oct + d;
      }
    }
    if (n == 0)
      return musicalStep(value, kMutationSigma);
    float semis = value * 12.f;
    int nearest = 0;
    float bestDist = 1e9f;
    for (int i = 0; i < n; i++) {
      float dist = std::abs(semis - lattice[i]);
      if (dist < bestDist) {
        bestDist = dist;
        nearest = i;
      }
    }
    // Small step in scale-degree space, mostly +-1, occasionally +-2 -
    // same shape as musicalStep's sigma/cap relationship, just denominated
    // in lattice positions instead of volts.
    int step = (random::uniform() < 0.8f) ? 1 : 2;
    if (random::uniform() < 0.5f) step = -step;
    int target = clamp(nearest + step, 0, n - 1);
    return clamp(lattice[target] / 12.f, 0.f, 5.f);
  }

  // A real trained model (a small GRU recurrent network), unlike Markov's
  // live-built-from-nothing table - trained offline on the Nottingham
  // Database (1034 public-domain folk tunes), predicting the next
  // semitone class from the kGruContext (8) preceding classes. Weights
  // are compiled in (EvoGruWeights.h) - no runtime loading, no ML
  // framework, just the hand-written forward pass below. Context is read
  // from phrase p's own live pool[] (same choice markovStep already
  // makes - the phrase's current content, not the in-progress candidate
  // array evolveAllPhrases() may be mid-building), the kGruContext steps
  // immediately preceding index i, wrapping around the 16-slot array.
  // Hidden state always starts at zero and replays the full context each
  // call, matching exactly how each training example was constructed -
  // there's no persistent memory carried between mutation events.
  float gruStep(float value, int p, int i) {
    float h[kGruHidden] = {};
    for (int t = 0; t < kGruContext; t++) {
      int idx = ((i - kGruContext + t) % 16 + 16) % 16;
      int cls = semitoneClass(pool[p][idx]);
      float x[kGruVocab] = {};
      x[cls] = 1.f;
      float z[kGruHidden], r[kGruHidden], hTilde[kGruHidden];
      for (int u = 0; u < kGruHidden; u++) {
        float zsum = kGruBz[u], rsum = kGruBr[u];
        for (int vIn = 0; vIn < kGruVocab; vIn++) {
          zsum += x[vIn] * kGruWz[vIn][u];
          rsum += x[vIn] * kGruWr[vIn][u];
        }
        for (int vHid = 0; vHid < kGruHidden; vHid++) {
          zsum += h[vHid] * kGruUz[vHid][u];
          rsum += h[vHid] * kGruUr[vHid][u];
        }
        z[u] = 1.f / (1.f + std::exp(-zsum));
        r[u] = 1.f / (1.f + std::exp(-rsum));
      }
      for (int u = 0; u < kGruHidden; u++) {
        float hsum = kGruBh[u];
        for (int vIn = 0; vIn < kGruVocab; vIn++)
          hsum += x[vIn] * kGruWh[vIn][u];
        for (int vHid = 0; vHid < kGruHidden; vHid++)
          hsum += (r[vHid] * h[vHid]) * kGruUh[vHid][u];
        hTilde[u] = std::tanh(hsum);
      }
      for (int u = 0; u < kGruHidden; u++)
        h[u] = (1.f - z[u]) * h[u] + z[u] * hTilde[u];
    }
    float logits[kGruVocab];
    for (int c = 0; c < kGruVocab; c++) {
      float sum = kGruBy[c];
      for (int u = 0; u < kGruHidden; u++)
        sum += h[u] * kGruWy[u][c];
      logits[c] = sum;
    }
    float maxLogit = logits[0];
    for (int c = 1; c < kGruVocab; c++)
      maxLogit = std::max(maxLogit, logits[c]);
    float probs[kGruVocab];
    float total = 0.f;
    for (int c = 0; c < kGruVocab; c++) {
      probs[c] = std::exp(logits[c] - maxLogit);
      total += probs[c];
    }
    float pick = random::uniform() * total;
    int toClass = 0;
    for (int c = 0; c < kGruVocab; c++) {
      if (pick < probs[c]) { toClass = c; break; }
      pick -= probs[c];
    }
    return nearestOctaveForClass(value, toClass);
  }

  // Dispatches mutation-sigma-scale deviations to whichever engine the
  // user has selected. Only affects the mutation calls in reseedPhrase(),
  // evolveAllPhrases(), and closeLoopSeam() - genesis()'s own master[]
  // walk always calls musicalStep() directly, staying Gaussian regardless
  // (it's the one shared DNA every phrase draws from before any per-phrase
  // engine character applies, and there's no pool history yet to build a
  // Markov table or feed the GRU a real context at that exact moment).
  // applyLink is layered on top regardless of engine - see applyLink's
  // own comment. i is this step's index within the 16-slot pool array,
  // needed only by gruStep for positional context.
  float mutateStep(float value, int p, int i) {
    float result;
    switch (mutationEngine) {
      case ENGINE_MARKOV:   result = markovStep(value, p); break;
      case ENGINE_INTERVAL: result = intervalStep(value, p); break;
      case ENGINE_GRU:       result = gruStep(value, p, i); break;
      default:               result = musicalStep(value, kMutationSigma); break;
    }
    return applyLink(applyHomeGravity(result));
  }

  // Genesis starts every phrase in a centered 2-octave window, but nothing
  // afterward pulls values back toward that home register - ongoing
  // mutation is an unbiased walk across the full 0-5V range, reflected at
  // the edges but with no restoring force. Over a long session that's
  // enough steps for the walk to occasionally wander into an extreme
  // register purely by chance (reported on real hardware as a rare,
  // unexpected high note with no other cause - not a bug, just an
  // unbiased random walk eventually visiting its extremes given enough
  // steps). A weak, constant pull back toward the same center genesis
  // already uses (2.5V, the middle of its 1.5-3.5V starting window) makes
  // that a much rarer outlier without noticeably taming the walk's
  // character on any single step - the pull only becomes meaningful once
  // a value has already drifted well away from center.
  static constexpr float kHomeRegister = 2.5f;
  static constexpr float kHomeGravity = 0.05f;
  float applyHomeGravity(float value) {
    return clamp(value + kHomeGravity * (kHomeRegister - value), 0.f, 5.f);
  }

  // Lets one module's current pitch gently bias another's random walk
  // toward it when LINK_INPUT is patched (typically from another
  // instance's own OUT_OUTPUT) - a small pull each time, not a hard
  // override, so two linked modules feel related without becoming exact
  // copies of each other (which would just sound like unison doubling).
  // Deliberately orthogonal to Model/engine selection - Link answers "is
  // this related to another module," engine answers "which algorithm
  // shapes drift," and they should compose regardless of which engine is
  // active. Unpatched behavior is completely unchanged.
  static constexpr float kLinkPull = 0.2f;
  float applyLink(float value) {
    if (!inputs[LINK_INPUT].isConnected())
      return value;
    float target = inputs[LINK_INPUT].getVoltage();
    return clamp(value + kLinkPull * (target - value), 0.f, 5.f);
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
  // genesis. typeArr, if given, is forced to STEP_NORMAL at the same
  // slot - a freshly-regenerated pitch value there is only meaningful if
  // that step actually plays it; leaving it marked rest/tie would waste
  // the smoothing on a step that stays silent/held regardless.
  void closeLoopSeam(float *arr, int p, int *typeArr = nullptr) {
    if (len <= 1) return;  // no wrap to smooth for a 1-step "loop"
    // A tie is always immediately followed by a rest (see reseedPhrase) -
    // if the step right before this wrap point is a tie, forcing this one
    // back to NORMAL would silently break that guarantee right at the
    // seam. Honor it here instead of overwriting it.
    if (typeArr && len >= 2 && typeArr[len - 2] == STEP_TIE) {
      typeArr[len - 1] = STEP_REST;
      return;
    }
    arr[len - 1] = mutateStep(arr[0], p, len - 1);
    if (typeArr) typeArr[len - 1] = STEP_NORMAL;
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
  // Rest/tie phrasing rides the same per-step deviation roll as pitch
  // mutation (stepDriftP above) rather than a separate probability - a
  // step that "deviates" becomes a rest, a tie, or a pitch shift, with
  // these two fractions splitting that outcome (the remainder, ~60% at
  // these settings, is a normal pitch shift, still the common case).
  // Inherits stepDriftP's anchor-step reduction and entropy-scaling for
  // free: a phrase's Entropy knob controls both "how much do notes shift"
  // and "how often does the rhythm get interrupted" together, matching
  // the design note's "entropy-gated" ask without a separate knob.
  static constexpr float kRestFraction = 0.25f;
  static constexpr float kTieFraction = 0.15f;
  void reseedPhrase(int p) {
    float driftP = entropyOf(p);
    for (int i = 0; i < 16; i++) {
      // A tie is always immediately followed by a rest - caps a hold at
      // exactly one extra step instead of letting ties chain indefinitely,
      // and gives the note a clear release before the next retrigger
      // instead of bleeding straight into it. Forced regardless of the
      // deviation roll below.
      if (i > 0 && poolType[p][i - 1] == STEP_TIE) {
        poolType[p][i] = STEP_REST;
        pool[p][i] = master[i];
        poolGenesis[p][i] = pool[p][i];
        poolGenesisType[p][i] = poolType[p][i];
        continue;
      }
      float stepDriftP = isAnchorStep(i) ? driftP * kAnchorMutationScale : driftP;
      if (random::uniform() < stepDriftP) {
        float sub = random::uniform();
        if (sub < kRestFraction) {
          poolType[p][i] = STEP_REST;
          pool[p][i] = master[i];  // pitch kept as reference, in case a later mutation reverts this step to normal
        } else if (sub < kRestFraction + kTieFraction) {
          poolType[p][i] = STEP_TIE;
          pool[p][i] = master[i];
        } else {
          poolType[p][i] = STEP_NORMAL;
          pool[p][i] = mutateStep(master[i], p, i);
        }
      } else {
        poolType[p][i] = STEP_NORMAL;
        pool[p][i] = master[i];
      }
      poolGenesis[p][i] = pool[p][i];
      poolGenesisType[p][i] = poolType[p][i];
    }
    closeLoopSeam(pool[p], p, poolType[p]);
    poolGenesis[p][len - 1] = pool[p][len - 1];  // keep them identical, same as every other slot above
    poolGenesisType[p][len - 1] = poolType[p][len - 1];
  }

  // Makes idx the currently-playing phrase and rolls its Recall check for
  // this playthrough: a fixed chance (kRecallChance, not user-adjustable -
  // Recall is deliberately always "Sometimes") of picking the frozen
  // genesis snapshot instead of the current evolved content for this one
  // playthrough (evolution keeps happening in the background regardless).
  // Does NOT touch loopsCompleted - every caller resets that itself
  // (unlike the old time-based phraseTimer, an integer loop count has no
  // fractional remainder worth preserving, so a hard reset to 0 is always
  // correct here).
  static constexpr float kRecallChance = 0.25f;
  // A phrase's genesis snapshot is only worth recalling once its live pool
  // has actually diverged from it - reseedPhrase() sets pool[]==poolGenesis[]
  // exactly, and evolveAllPhrases() is the only thing that ever writes pool[]
  // afterward, so a straight comparison reliably detects "nothing to recall
  // from yet" (true right after genesis, and again for any phrase reseeded
  // by reactivating its Active toggle). Only checks the first len steps -
  // the backing arrays are always 16 slots regardless of Length, but a
  // phrase diverging only in steps beyond the currently audible range
  // wouldn't actually sound any different if recalled.
  bool phraseHasDiverged(int p) {
    for (int i = 0; i < len; i++) {
      if (poolType[p][i] != poolGenesisType[p][i]) return true;
      if (pool[p][i] != poolGenesis[p][i]) return true;
    }
    return false;
  }
  void selectPhrase(int idx) {
    currentPhraseIdx = idx;
    usingGenesisNow = phraseHasDiverged(idx) && (random::uniform() < kRecallChance);
    // Converts a mutation that happened while this phrase wasn't playing
    // into an actual Drift flash now, exactly as its own turn begins - see
    // pendingDrift's declaration.
    if (pendingDrift[idx]) {
      mutationFlash[idx] = 1.f;
      pendingDrift[idx] = false;
    }
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
  // both halves, so drift/persist always move together within a single
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
    int candidateType[4][16];
    for (int k = 0; k < n; k++) {
      int p = activeIdx[k];
      float driftP = entropyRoll[p];
      int partnerK = (int)(random::uniform() * n);
      if (partnerK >= n) partnerK = n - 1;
      int partner = activeIdx[partnerK];
      int split = 1 + (int)(random::uniform() * 14.f);  // 1..14
      for (int i = 0; i < 16; i++) {
        candidates[p][i] = (i < split) ? pool[p][i] : pool[partner][i];
        // Same rule as reseedPhrase: a tie is always immediately followed
        // by a rest. Checked against the just-resolved candidateType[i-1]
        // rather than the pre-crossover pool, so it also repairs the case
        // where the crossover split point itself lands between an
        // inherited tie and its rest (each side of the splice satisfied
        // the invariant in its own source pool, but splicing between them
        // doesn't guarantee it holds across the seam).
        if (i > 0 && candidateType[p][i - 1] == STEP_TIE) {
          candidateType[p][i] = STEP_REST;
          continue;
        }
        candidateType[p][i] = (i < split) ? poolType[p][i] : poolType[partner][i];
        // Perturbs whatever crossover already produced for this step by a
        // small musicalStep delta, instead of discarding it for an
        // unrelated fresh random value - keeps mutations sounding like
        // variations on the existing melodic material. Anchor steps
        // mutate less often (see isAnchorStep). Same deviation roll also
        // decides rest/tie (see kRestFraction/kTieFraction).
        float stepDriftP = isAnchorStep(i) ? driftP * kAnchorMutationScale : driftP;
        if (random::uniform() < stepDriftP) {
          float sub = random::uniform();
          if (sub < kRestFraction) {
            candidateType[p][i] = STEP_REST;
          } else if (sub < kRestFraction + kTieFraction) {
            candidateType[p][i] = STEP_TIE;
          } else {
            candidates[p][i] = mutateStep(candidates[p][i], p, i);
            candidateType[p][i] = STEP_NORMAL;
          }
        }
      }
    }
    // Below the threshold, the persist check can still pass (structurally
    // "replaced") even though crossover+mutation happened to land back on
    // essentially the same values - e.g. low Entropy, or a crossover
    // partner that was already very similar. Flashing Drift for that
    // would tell the user something changed when nothing audible did.
    // Same reasoning applies to scope: only the first len steps are
    // checked, since a change confined to steps beyond the currently
    // audible Length wouldn't be heard either (the full 16-slot backing
    // store is still updated below regardless - those hidden steps keep
    // evolving even while unheard, just shouldn't trigger the light).
    const float kAudibleChangeThreshold = 0.01f;  // ~0.12 semitones
    for (int k = 0; k < n; k++) {
      int p = activeIdx[k];
      float persistP = 1.f - entropyRoll[p];
      if (random::uniform() >= persistP) {
        bool audiblyChanged = false;
        for (int i = 0; i < len; i++) {
          if (candidateType[p][i] != poolType[p][i]) {
            audiblyChanged = true;
            break;
          }
          if (candidateType[p][i] == STEP_NORMAL && std::abs(candidates[p][i] - pool[p][i]) > kAudibleChangeThreshold) {
            audiblyChanged = true;
            break;
          }
        }
        for (int i = 0; i < 16; i++) {
          pool[p][i] = candidates[p][i];
          poolType[p][i] = candidateType[p][i];
        }
        closeLoopSeam(pool[p], p, poolType[p]);
        if (audiblyChanged)
          pendingDrift[p] = true;
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
        closeLoopSeam(pool[p], p, poolType[p]);
        closeLoopSeam(poolGenesis[p], p, poolGenesisType[p]);
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

    if (engineTrig.process(params[ENGINE_PARAM].getValue()))
      mutationEngine = (mutationEngine + 1) % NUM_ENGINES;

    // Light brightness itself is set later (with the mutationFlash-style
    // decay loop below), once this step's played note (if any) is known -
    // just track on/off state here (quantSetForPhrase re-derives the
    // anyActive flags itself when a note is actually read below).
    for (int i = 0; i < 12; i++) {
      if (noteTrigL[i].process(params[NOTE_PARAM_L + i].getValue()))
        noteActiveL[i] = !noteActiveL[i];
      if (noteTrigR[i].process(params[NOTE_PARAM_R + i].getValue()))
        noteActiveR[i] = !noteActiveR[i];
    }

    if (justStarted)
      genesis();

    if (!phraseActive[currentPhraseIdx]) {
      int next = findNextActivePhrase(currentPhraseIdx);
      if (next != currentPhraseIdx) {
        selectPhrase(next);
        loopsCompleted = 0;
        // Forces the next clock edge's step=(step+1)%len to land exactly
        // on 0 - this is an interrupted jump (the old phrase got
        // deactivated mid-sequence), not a natural phrase-boundary
        // transition, but the new phrase should still always start on
        // its own first step, same as every other transition.
        step = len - 1;
      }
    }

    float riseTime = params[RISE_PARAM].getValue();
    float fallTime = params[FALL_PARAM].getValue();

    float outCV = 0.f;

    if (trigTimer > 0.f) {
      trigTimer -= args.sampleTime;
      if (trigTimer < 0.f) trigTimer = 0.f;
    }

    // Steps only advance on an actual incoming pulse (there's no
    // free-running internal clock) - measured regardless of Run state to
    // avoid a stuck-trigger edge case if a pulse arrives while stopped,
    // but only acted on below while running.
    bool clockEdge = clockTrig.process(inputs[CLOCK_INPUT].getVoltage());

    if (running) {
      int activeCount = 0;
      for (int p = 0; p < 4; p++)
        if (phraseActive[p]) activeCount++;

      if (clockEdge) {
        step = (step + 1) % len;
        trigTimer = 0.f;

        // Phrase transitions are counted in step-wraps (whole passes
        // through the sequence) rather than elapsed wall-clock time -
        // guarantees a transition always lands exactly on a phrase
        // boundary (step 0), regardless of clock jitter. A time-based
        // approach (phraseTimer accumulating args.sampleTime, compared
        // against a phraseDuration computed from a separately-measured
        // clockPeriod) could let real elapsed time and actual clock
        // edges drift out of sync, firing a transition mid-sequence
        // instead of exactly at a phrase boundary. Since this check runs
        // inside the same clockEdge block, right before the note-read
        // logic below, a transition here is guaranteed to be immediately
        // followed by reading step 0 of the NEW phrase on this same
        // clock edge - no gap, no stale step from the old phrase.
        if (step == 0 && activeCount > 0) {
          loopsCompleted++;
          const int phraseLoops = 4;  // fixed, not user-adjustable
          if (loopsCompleted >= phraseLoops) {
            loopsCompleted = 0;
            selectPhrase(findNextActivePhrase(currentPhraseIdx));
            phrasesPlayedThisRound++;
            if (phrasesPlayedThisRound >= activeCount) {
              phrasesPlayedThisRound = 0;
              evolveAllPhrases();
            }
          }
        }

        // Rest/tie is baked into the pool as genetic material (see
        // poolType's declaration) rather than rolled fresh here - just
        // read whichever step is about to play.
        const int *playingType = usingGenesisNow ? poolGenesisType[currentPhraseIdx] : poolType[currentPhraseIdx];
        int stepType = playingType[step];
        // A short Fall time reaches idle well before the *next* clock edge
        // - freezing only during the tie step itself froze a level that
        // had already decayed to zero during the note before it. Freezing
        // one step early too (whenever the step about to play is a tie)
        // stops that decay before it happens, so there's still something
        // to hold by the time the tie lands. Rests never freeze - nothing
        // to hold, they should stay silent regardless of what follows.
        bool nextIsTie = playingType[(step + 1) % len] == STEP_TIE;
        envFrozen = (stepType == STEP_TIE) || (stepType != STEP_REST && nextIsTie);

        if (stepType == STEP_REST) {
          // Forces an early decay - using the existing Fall-time ramp, so
          // it doesn't click the way an instant envLevel=0 jump would -
          // instead of just skipping the retrigger and letting a long
          // Fall time bleed through into what's supposed to be silence.
          // Left alone if already idle (nothing playing to cut off).
          if (envPhase != ENV_IDLE)
            envPhase = ENV_DECAY;
        } else if (stepType == STEP_TIE) {
          // heldCV/envPhase carry straight through untouched (no
          // retrigger), and envFrozen (set above) pauses the ramp itself
          // for this step's duration - without freezing the ramp too, a
          // short Fall time would already have decayed to silence by the
          // time a tie lands, making it indistinguishable from a rest.
        } else if (envPhase == ENV_IDLE) {
          // A-166-style gate, restored per feedback: a new note never
          // fires until the current envelope has fully completed, same as
          // TheReelPeet (dyn). This was briefly changed to always-retrigger
          // to fix a tie-chain bug (the real note after a tie could get
          // silently dropped while envFrozen held envPhase non-idle) - but
          // that's structurally the same situation as any ordinary long
          // Rise/Fall outlasting the clock period, which this gate has
          // always been able to skip past too. Restoring it trades "every
          // step is guaranteed to retrigger" for "long envelopes always
          // ring out fully," matching this module's sibling and the
          // explicit preference here.
          const float *playingSeq = usingGenesisNow ? poolGenesis[currentPhraseIdx] : pool[currentPhraseIdx];
          trigTimer = 0.01f;
          // Both quantizers are optional (all lights off = ignored, CV
          // passes through unquantized - see quantizeToActiveNotes). See
          // quantSetForPhrase for the left/right selection rule.
          const bool *quantSet = quantSetForPhrase(currentPhraseIdx);
          bool quantIsLeft = (quantSet == noteActiveL);
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
      envFrozen = false;
      loopsCompleted = 0;
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

    // Attack always progresses, even on a step envFrozen is holding for an
    // upcoming tie - freezing it too would leave a note that's about to be
    // tied into stuck silent at envLevel 0 for the whole step (never even
    // got to rise), which a later real step would then just overwrite
    // before it was ever heard. Only decay needs to pause - that's what
    // actually preserves something worth holding across a tie.
    if (envPhase == ENV_ATTACK) {
      if (riseTime < 0.001f) {
        envLevel = 10.f;
      } else {
        envLevel += (10.f / riseTime) * args.sampleTime;
        envLevel = std::min(envLevel, 10.f);
      }
      if (envLevel >= 10.f)
        envPhase = ENV_DECAY;
    } else if (envPhase == ENV_DECAY && !envFrozen) {
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
    for (int e = 0; e < NUM_ENGINES; e++)
      lights[ENGINE_LIGHT + e].setBrightness(e == mutationEngine ? 1.f : 0.f);
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

    json_object_set_new(rootJ, "mutationEngine", json_integer(mutationEngine));

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

    json_t *engineJ = json_object_get(rootJ, "mutationEngine");
    if (engineJ) {
      int v = (int)json_integer_value(engineJ);
      if (v >= 0 && v < NUM_ENGINES)
        mutationEngine = v;
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
  bool leftAlign = false;  // Model group's list reads better left-justified than centered

  void draw(const DrawArgs &args) override {
    nvgFontFaceId(args.vg, APP->window->uiFont->handle);
    // Neutral black (was a reddish #280b0b) so labels read consistently
    // sitting on top of all 4 differently-hued pools, rather than quietly
    // favoring pool1's red family - matches EvoLengthDisplay's "8 Steps"
    // text, which already used plain black.
    nvgFillColor(args.vg, nvgRGB(0x00, 0x00, 0x00));
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

// A translucent rounded-rect fill (no stroke) - sets the Model select
// group (button + 3 engine rows) apart from the rest of the global lane.
// Dark shade of the lane's own red (#d94a4a, see laneLeft in
// res/TheReelPeetEvo.svg) at the same 18% opacity the pool boxes already
// use for their own accent tints, so it reads as belonging to this panel's
// existing visual language rather than a bolted-on outline.
struct EvoGroupBox : TransparentWidget {
  NVGcolor color = nvgRGBA(0x82, 0x2c, 0x2c, 46);

  void draw(const DrawArgs &args) override {
    float r = mm2px(Vec(2.4f, 0.f)).x;
    nvgBeginPath(args.vg);
    nvgRoundedRect(args.vg, 0.5f, 0.5f, box.size.x - 1.f, box.size.y - 1.f, r);
    nvgFillColor(args.vg, color);
    nvgFill(args.vg);
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
    // In below). Jitter's old raw-y=60 slot is now the "Model" mutation-
    // engine select: a cycling button + label on top, a 4-row list below
    // it (light + full name per engine, see mutateStep/Engine), the whole
    // group boxed off with EvoGroupBox to set it apart from the rest of
    // the global lane. Rise/Fall pushed further down to make room for the
    // 4th row (GRU) added after Gaussian/Markov/Interval.
    const float engineY = 55.f;
    // Shared left/right columns for the whole Model group - button and
    // engine lights share the left column, "Model" and the engine names
    // share the right column, so everything lines up vertically.
    // Shifted further left per feedback ("Experimental" was creeping past
    // the box's right edge) - the box itself is already close to the
    // lane's own left edge, so most of the extra room comes from
    // tightening the light-to-text gap rather than moving the box more.
    const float modelColL = laneXL - 11.f;
    // Left-justified list reads better than centered - modelColR is now
    // the text's left edge, not its center.
    const float modelColR = laneXL - 6.f;
    const float modelLabelW = 26.f;
    const float engineRowY[4] = {engineY + 7.f, engineY + 13.f, engineY + 19.f, engineY + 25.f};
    // Was 94, crowding the Rise/Fall labels right up against the output
    // jacks below with no breathing room - pulled back up closer to the
    // (now taller, 4-row) Model box, and the CV rows nudged down to
    // compensate instead of everything getting cramped into the same gap.
    const float riseFallY = 92.f;
    const float outY      = 103.f;
    const float clockInY  = 114.f;

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

    // Group box first so it renders behind the button/lights/labels, not
    // on top of them - sized to enclose the button+label row and all 3
    // engine rows with a small margin.
    auto *modelBox = new EvoGroupBox;
    modelBox->box.pos = mm2px(Vec(laneXL - 14.f, engineY - 5.f));
    modelBox->box.size = mm2px(Vec(30.f, 35.f));
    addChild(modelBox);

    addParam(createParamCentered<LEDButton>(mm2px(Vec(modelColL, engineY)), module, TheReelPeetEvo::ENGINE_PARAM));
    // One row per engine (light + full name, see labels below) instead of
    // a horizontal row of abbreviations - there's room for it now that
    // Rise/Fall moved down. Button and lights share modelColL, "Model"
    // and the engine names share modelColR, so the whole group lines up.
    addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(modelColL, engineRowY[TheReelPeetEvo::ENGINE_GAUSSIAN])), module, TheReelPeetEvo::ENGINE_LIGHT + (int)TheReelPeetEvo::ENGINE_GAUSSIAN));
    addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(modelColL, engineRowY[TheReelPeetEvo::ENGINE_MARKOV])), module, TheReelPeetEvo::ENGINE_LIGHT + (int)TheReelPeetEvo::ENGINE_MARKOV));
    addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(modelColL, engineRowY[TheReelPeetEvo::ENGINE_INTERVAL])), module, TheReelPeetEvo::ENGINE_LIGHT + (int)TheReelPeetEvo::ENGINE_INTERVAL));
    addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(modelColL, engineRowY[TheReelPeetEvo::ENGINE_GRU])), module, TheReelPeetEvo::ENGINE_LIGHT + (int)TheReelPeetEvo::ENGINE_GRU));

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

    // Paired with Link at this row, same convention as Out/Gate pairing
    // one row up - Clock moves off-center to make room.
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(laneXL - cvDX, clockInY)), module, TheReelPeetEvo::CLOCK_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(laneXL + cvDX, clockInY)), module, TheReelPeetEvo::LINK_INPUT));

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

      // x is the label's horizontal center, unless leftAlign is set, in
      // which case x is the left edge the text itself starts at instead
      // (the box still spans the same width either way, just the text's
      // anchor point within it changes).
      auto addLabel = [&](const std::string &text, float x, float y, float w,
                           float fontSize, bool bold = false, bool leftAlign = false) {
        auto *label = new EvoStaticLabel;
        label->text = text;
        label->fontSize = fontSize;
        label->bold = bold;
        label->leftAlign = leftAlign;
        label->box.pos = mm2px(Vec(leftAlign ? x : x - w * 0.5f, y));
        label->box.size = mm2px(Vec(w, 5.f));
        addChild(label);
      };

      // "THEREELPEET" wordmark is the real path art in the SVG, recentered
      // on the new panel width — see res/TheReelPeetEvo.svg.

      addLabel("Run", laneXL, runY + 4.f, dispW, 9.f);
      // "Model" sits beside its button, vertically centered on it (same
      // -2.5f box-top offset trick as the engine rows below, so the 5mm
      // label box centers its text on the button's own y). Engine names
      // follow the same pattern, one per row, sharing modelColR. All
      // left-justified (reads as a list, not centered captions), and
      // "Model" bumped to 11pt (was 9pt) so it reads clearly as this
      // group's header rather than just another row.
      addLabel("Model", modelColR, engineY - 2.5f, modelLabelW, 11.f, true, true);
      addLabel("Gaussian", modelColR, engineRowY[TheReelPeetEvo::ENGINE_GAUSSIAN] - 2.5f, modelLabelW, 8.f, false, true);
      addLabel("Markov", modelColR, engineRowY[TheReelPeetEvo::ENGINE_MARKOV] - 2.5f, modelLabelW, 8.f, false, true);
      addLabel("Interval", modelColR, engineRowY[TheReelPeetEvo::ENGINE_INTERVAL] - 2.5f, modelLabelW, 8.f, false, true);
      // "AI (Experimental)" - unlike Markov, this genuinely is a trained
      // model (a small GRU, see gruStep/EvoGruWeights.h), so the label
      // earns the term rather than overselling it. Spelled out in full
      // (was "AI(exp)") now that the group's shifted left for the room;
      // bolded since "AI" alone read ambiguously as "Al" at 8pt regular
      // weight (capital I and lowercase l are near-identical strokes in
      // this font at small sizes) - bolding is the cheapest attempt at
      // disambiguating it before resorting to a different word entirely.
      addLabel("AI (Experimental)", modelColR, engineRowY[TheReelPeetEvo::ENGINE_GRU] - 2.5f, modelLabelW, 8.f, true, true);
      addLabel("Rise", laneXL - cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("Fall", laneXL + cvDX, riseFallY + 3.f, dispW2, 8.f);
      addLabel("1v/O", laneXL - cvDX, outY + 3.5f, dispW2, 8.f);
      addLabel("Gate", laneXL + cvDX, outY + 3.5f, dispW2, 8.f);
      addLabel("Clock", laneXL - cvDX, clockInY + 3.5f, dispW2, 8.f);
      addLabel("Link", laneXL + cvDX, clockInY + 3.5f, dispW2, 8.f);

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
