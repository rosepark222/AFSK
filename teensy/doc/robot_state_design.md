# Robot State Design: Q&A Session

Topic: designing subsystem, superstructure, and sequencer states for a robot with a speaker, 2 mics, and a chassis (Teensy).

---

## Q1: Initial question

> I have a robot having a speaker, 2 mics and chassis as three subsystems. Can you suggest subsystem and superstructures states in my robot?
>
> - robot can send a chirp and AFSK messages
> - robot can detect a chirp
> - robot can calculate the angle of the chirp from the source using TDOA
> - robot can decode AFSK and do what the message asks
> - robot can turn left, turn right, go straight, turn 180 degrees
> - robot send command such as stop chirp, turn off LED, send me an ACK immediately etc.
>
> robot does not activate two subsystems at the same time. It does not drive and send a chirp, or send a chirp and listen.
> robot does not have encoders for motors, it uses timer or gyro to determine the angle of rotation.
>
> What more info do you need to design states of subsystem and superstructures?

### A1

Here's a first-pass design, followed by the questions that would let me tighten it.

#### Subsystem states

**Speaker subsystem**
- `IDLE`: silent
- `SENDING_CHIRP`: emitting a chirp (single or repeated)
- `SENDING_AFSK`: modulating and transmitting a message frame
- `DONE`: transmission complete (momentary state, then back to IDLE)

**Mic subsystem (2 mics)**
- `IDLE`: not sampling
- `LISTENING`: sampling both mics, running chirp detection
- `CHIRP_DETECTED`: chirp found, holding the synchronized sample buffer
- `COMPUTING_TDOA`: cross-correlating the two channels and computing the angle
- `RECEIVING_AFSK`: demodulating a message frame
- `DECODING`: validating the frame (preamble, CRC) and parsing the command

Note that the mic subsystem is really two sub-activities, chirp detection and AFSK decoding. You could split it into two state groups or keep it as one subsystem with a mode.

**Chassis subsystem**
- `STOPPED`
- `DRIVING_STRAIGHT`: with a timer or duration
- `TURNING_LEFT`: with target angle and a timer or gyro
- `TURNING_RIGHT`
- `TURNING_180`
- `FAULT`: gyro timeout, etc. (optional)

For turns, each state has an internal condition: `elapsed_time >= T` (timer-based) or `|gyro_angle - target| < tolerance` (gyro-based). If you use the gyro, add a timeout as a safety exit.

#### Superstructure states

Since only one subsystem can be active at a time, the superstructure is essentially an arbiter that owns a single "who has the floor" state. Suggested states:

| Superstructure state | Active subsystem | Purpose |
|---|---|---|
| `IDLE` | none | Waiting for a trigger |
| `LISTENING` | Mic | Waiting for chirp or AFSK |
| `LOCALIZING` | Mic (compute) | TDOA calculation on the captured chirp |
| `DECODING_MESSAGE` | Mic | AFSK demod and parse |
| `EXECUTING_COMMAND` | Depends on command | Dispatch to a sub-sequence |
| `MOVING` | Chassis | Drive or turn |
| `TRANSMITTING_CHIRP` | Speaker | Send chirp |
| `TRANSMITTING_MSG` | Speaker | Send AFSK (command, ACK, etc.) |
| `SENDING_ACK` | Speaker | Priority transmit of an ACK (can be a variant of TRANSMITTING_MSG) |
| `ERROR` | none | Safe stop, recover to IDLE or LISTENING |

**Typical transitions**
- `IDLE → LISTENING` (start, or after any action completes)
- `LISTENING → LOCALIZING` (chirp detected) `→ MOVING` (turn toward source) `→ LISTENING`
- `LISTENING → DECODING_MESSAGE` (AFSK carrier detected) `→ EXECUTING_COMMAND`
- `EXECUTING_COMMAND → MOVING | TRANSMITTING_CHIRP | SENDING_ACK | (LED toggle, no subsystem)`
- Anything with a command that asks for an ACK: `→ SENDING_ACK → LISTENING`

**Mutual exclusion rule:** the superstructure only allows a transition into a state that activates a subsystem if the previously active subsystem has returned to IDLE first. Put that in the transition guards, not in the subsystems.

#### Design decisions worth making explicitly

1. **Persistent behaviors:** "stop chirp" implies chirping can be continuous. Since the robot can't listen while chirping, a repeating chirp should probably be a `CHIRP_BURST → LISTEN_WINDOW` duty cycle, or "stop chirp" can only be received between bursts.
2. **Command queue:** if a message contains multiple commands, or arrives during a move, do you drop it, queue it, or ignore it?
3. **LED:** it's not in the exclusion rule, so I'd treat it as a trivial subsystem or a side effect, not a superstructure state.

#### Information I need to refine this

**About behavior**
1. What is the robot's goal? (Follow a chirp source? Obey remote commands only? Both?)
2. Does the robot chirp periodically, or only on command?
3. After TDOA gives an angle, does it automatically turn toward the source, or just report the angle (via AFSK)?
4. Are commands ever chained (e.g., "turn left, then go straight 2 s")? Or one command per message?
5. Priority: if a command arrives mid-move, should it interrupt, queue, or be ignored? Does "stop" always preempt?

**About the protocol**
6. Is there a defined message format (preamble, command ID, parameters, CRC)? What are the command set and parameters (duration, angle)?
7. Is an ACK automatic for every message, or only when requested?
8. How is a chirp distinguished from AFSK (frequency band, duration)?

**About hardware and timing**
9. How long do chirp, AFSK transmission, and TDOA computation take? (Determines whether states are blocking.)
10. Is the speaker audible to the robot's own mics, i.e., do you need a deadtime after transmitting before listening?
11. Gyro vs. timer: is the gyro always present, or is the timer a fallback?
12. Does driving noise (motors) prevent listening? (This is probably why exclusion exists, and it implies you need a settle time after stopping.)

**About software architecture**
13. Is this a periodic loop (like a command-based or FSM tick) or event-driven? What microcontroller and RTOS, if any?
14. Do you want a hierarchical state machine where the superstructure commands subsystems by "requested state" and the subsystems report "at goal," or a flat one?

If you answer 1, 3, 5, 6, and 10, I can give you a concrete state diagram with exact transition conditions. The rest I can handle with reasonable defaults.

---

## Q2: Adding a sequence of tasks

> Let's say I want to add a sequence of tasks, what would be modified? For example, it listens to a chirp, gets TDOA, then turns 90 degrees, listens to the chirp and gets TDOA, then determines the direction, then moves 1 foot toward the source, then sends AFSK to turn off the chirp, etc.

### A2

Your subsystems barely change. The main addition is a new layer above the superstructure that runs ordered steps.

#### What changes

| Layer | Change |
|---|---|
| **Subsystems** (Speaker, Mic, Chassis) | Almost none. Each needs to report a result when it finishes: `SUCCESS`, `TIMEOUT`, or `FAIL` (e.g. no chirp heard in 5 s). |
| **Superstructure** | Stays the arbiter that enforces mutual exclusion. Its states become reusable primitives that take parameters (`TURN(angle)`, `DRIVE(distance)`, `LISTEN_FOR_CHIRP(timeout)`) and return a result. |
| **New: Sequencer** | Holds an ordered list of steps, an index, and shared data. It asks the superstructure to run one primitive, waits for the result, then advances, retries, or aborts. |

The hierarchy becomes **Sequencer → Superstructure → Subsystems**. The sequencer never touches subsystems directly.

#### Your example as a sequence

| # | Step | Primitive (active subsystem) | Output / next |
|---|---|---|---|
| 1 | Wait for chirp | `LISTEN_CHIRP` (Mic) | on timeout → abort |
| 2 | Localize | `COMPUTE_TDOA` (Mic) | store `θ1` |
| 3 | Turn 90° | `TURN(90)` (Chassis) | |
| 4 | Settle | `WAIT(t)` (none) | lets motor noise and vibration die down |
| 5 | Wait for chirp | `LISTEN_CHIRP` (Mic) | |
| 6 | Localize | `COMPUTE_TDOA` (Mic) | store `θ2` |
| 7 | Decide direction | `COMPUTE_BEARING` (none, pure math) | store `bearing` |
| 8 | Face source | `TURN(bearing)` (Chassis) | |
| 9 | Drive 1 ft | `DRIVE(1 ft)` (Chassis) | |
| 10 | Stop the chirp | `SEND_AFSK(STOP_CHIRP)` (Speaker) | |
| 11 | Confirm | `LISTEN_ACK(timeout)` (Mic) | on timeout → retry step 10 up to N times |
| 12 | Done | | → `IDLE` |

Step 7 is why the 90° turn is useful: two mics give only an angle relative to the mic baseline, with a front/back mirror ambiguity. Two measurements from different headings leave only one consistent candidate. Because that turn is gyro- or timer-based, its error feeds into the result, so use the gyro here if you can.

#### Specific modifications

1. **Primitives take goals and return results.** Instead of `TURNING_LEFT`, use `TURN(target_deg, timeout)`. The result is what lets the sequencer branch.
2. **Shared context (a "blackboard").** Steps need to pass data forward: `θ1`, `θ2`, `bearing`, retry counters, and the heading at each measurement. Keep this in the sequencer, not in the subsystems.
3. **Steps with no subsystem.** `WAIT` (settle time) and `COMPUTE_BEARING` occupy the superstructure but activate nothing. Add a settle step after any chassis motion and after any speaker output so the robot doesn't hear itself or its motors.
4. **Failure policy per step:** `abort`, `retry(n)`, or `skip`. For example, a missed chirp might retry twice before aborting, while a missed ACK retries the AFSK send.
5. **Abort and preemption.** An incoming `STOP`, or a sequence-cancel command, must be able to interrupt the sequence and force the chassis to `STOPPED`. Decide which commands are allowed mid-sequence. Since the robot can't listen while driving, "stop" can only be heard between steps, so say so explicitly.
6. **Sequences as data.** If an AFSK command can be "run homing sequence", the decoder just loads a sequence ID. You can define sequences as a table of steps, so adding new ones doesn't require new state-machine code.

#### Minimal structure

```
Sequence = [Step, Step, ...]
Step = { primitive, params, on_success, on_fail: ABORT|RETRY(n)|SKIP }

loop:
  step = seq[i]
  superstructure.request(step.primitive, step.params, ctx)
  wait until superstructure reports result
  if SUCCESS: i++
  else: apply step.on_fail
```

The sequencer's own states are just `IDLE`, `RUNNING(step i)`, `ABORTING`, `DONE`, and `FAILED`.

#### Questions that affect this

1. When a step in the middle of the sequence fails, should the robot stop where it is or return to a safe state?
2. Is "1 foot" timer-based? That needs calibration of speed against battery voltage and surface.
3. After the stop-chirp AFSK, must the robot confirm that the chirp actually stopped (by listening again), or is the ACK enough?

If you tell me whether sequences are hard-coded or loaded from AFSK commands, I can sketch the actual state diagram and message format for `RUN_SEQUENCE`.

---

## Q3: Why have a superstructure if there is a sequencer?

> Sequencer → Superstructure → Subsystems is the hierarchy. If I have a sequencer, what's the point of having a superstructure? Show a brief code layout showing how the Superstructure acts as a safety wrapper around a Sequencer step command to ensure your "no simultaneous subsystems" rule is never broken.

### A3

The sequencer answers "what comes next?" The superstructure answers "is it safe to do that right now?" They are different jobs, and the second one has to hold no matter who is asking.

The sequencer is not the only thing that wants to use subsystems. An incoming AFSK `STOP`, a one-off `SEND_ACK`, a chirp-triggered reaction, or a second sequence can all ask for the speaker or chassis. If the exclusion rule lives in the sequencer, every one of those paths has to remember to enforce it. If it lives in the superstructure, there is one choke point, and the subsystems are private, so nobody can bypass it.

If you only ever have one sequence runner and no other command sources, you can merge the two layers. Once you have more than one source of commands, you want the wrapper.

#### Code layout

```cpp
enum class Owner  { NONE, SPEAKER, MIC, CHASSIS };
enum class Result { RUNNING, SUCCESS, TIMEOUT, FAIL, REJECTED, ABORTED };

struct Primitive { Owner needs; /* type, params, timeout */ };

class Superstructure {
public:
    // The ONLY entry point for starting work. Sequencer, AFSK handler, etc. all call this.
    bool request(const Primitive& p) {
        if (owner_ != Owner::NONE)          return false;  // someone is active
        if (settling())                     return false;  // quiet period after last owner
        if (p.needs != Owner::NONE && !subsystem(p.needs).isIdle()) return false;

        owner_ = p.needs;
        current_ = p;
        if (owner_ != Owner::NONE) subsystem(owner_).start(p);
        return true;
    }

    // STOP bypasses the queue but still goes through the same release path.
    void emergencyStop() {
        if (owner_ != Owner::NONE) subsystem(owner_).abort();
        chassis_.stop();                     // always force motors off
        release(Result::ABORTED);
    }

    // Called every control tick.
    void update() {
        if (owner_ == Owner::NONE) return;
        Result r = subsystem(owner_).update();
        if (r != Result::RUNNING) release(r);
    }

    Result takeResult() { auto r = lastResult_; lastResult_ = Result::RUNNING; return r; }

private:
    void release(Result r) {
        // Don't free the floor until the subsystem is truly idle.
        subsystem(owner_).forceIdle();
        settleUntil_ = now() + settleTimeFor(owner_);  // motor noise / speaker ring-down
        owner_ = Owner::NONE;
        lastResult_ = r;
    }
    bool settling() { return now() < settleUntil_; }

    // Subsystems are private: no one outside can call chassis_.drive() directly.
    Speaker speaker_;  Mic mic_;  Chassis chassis_;
    Owner owner_ = Owner::NONE;
    Primitive current_;
    Result lastResult_ = Result::RUNNING;
    uint32_t settleUntil_ = 0;
};
```

```cpp
class Sequencer {
public:
    void update() {
        superstructure_.update();
        switch (state_) {
        case RUNNING_REQUEST:
            // May be refused; just retry next tick. No safety logic here.
            if (superstructure_.request(steps_[i_].primitive)) state_ = RUNNING_WAIT;
            break;
        case RUNNING_WAIT:
            Result r = superstructure_.takeResult();
            if (r == Result::RUNNING) break;
            applyFailurePolicy(r);   // advance, retry(n), or abort
            break;
        }
    }
};

// Other command sources use the same gate:
void onAfskCommand(Cmd c) {
    if (c == STOP) superstructure_.emergencyStop();
    else if (!superstructure_.request(toPrimitive(c))) queueOrDrop(c);
}
```

#### Why this enforces the rule

1. **Single owner variable.** `owner_` can only be set inside `request()`, which refuses if it is not `NONE`. Two active subsystems is structurally impossible, not just "avoided by convention."
2. **Subsystems are private.** There is no other path to `chassis_.drive()` or `speaker_.send()`.
3. **Release goes through one function.** `release()` forces the subsystem idle and starts the settle timer, so the robot never starts listening while motors or the speaker are still ringing.
4. **Emergency stop reuses the same path.** Preemption can't leave the system in a half-owned state.
5. **The sequencer stays simple.** A refused request is just "try again next tick." You can write a buggy sequence and the hardware rules still hold.

The same structure also gives you one place to add other safeties later, such as a maximum drive duration, a gyro-fault lockout, or a watchdog that releases an owner that has been active too long.

---

## Q4: Robot A and Robot B, Teensy code

> A robot does the sequence:
>
> **Robot A:** listens to a chirp, gets TDOA, then turns 90 degrees, listens to the chirp and gets TDOA, then determines the direction, then moves 1 foot toward the source, then sends AFSK to turn off the chirp.
>
> **Robot B:** sends a chirp for 0.7 sec, listens for AFSK for 2.3 sec, and if nothing is detected repeats the chirp-listen. If detected, does what the AFSK says and transitions to DONE.
>
> For robot A and B, are the only differences the sequence? Can you give me Teensy code for robot A and robot B? It does not have to be complete, I just want to see the setup and loop structure.

### A4

Mostly yes. The subsystems, superstructure, and sequencer engine can be identical firmware on both robots, and only the step table differs. Robot B needs two small additions to the engine:

1. **Branching on failure.** For A, a timeout means abort. For B, a timeout on `LISTEN_AFSK` is the normal path and means "go back to step 0." So the failure policy needs `GOTO(step)` alongside `ABORT`, `RETRY`, and `SKIP`.
2. **A step whose action comes from data.** B's "do what the AFSK says" isn't known at compile time. Add a primitive like `EXEC_RECEIVED_CMD` that reads the decoded command from the shared context and turns it into a real primitive at run time.

#### Shared code (same on both robots)

```cpp
// common.h
enum class Prim { WAIT, LISTEN_CHIRP, COMPUTE_TDOA, COMPUTE_BEARING,
                  TURN, DRIVE, SEND_CHIRP, SEND_AFSK, LISTEN_AFSK,
                  EXEC_RECEIVED_CMD };
enum class Result { RUNNING, SUCCESS, TIMEOUT, FAIL, REJECTED, ABORTED };
enum class OnFail { ABORT, RETRY, SKIP, GOTO };

struct Step {
  Prim     prim;
  float    param;       // angle, distance, or cmd id
  uint32_t timeout_ms;  // or duration for SEND_CHIRP / WAIT
  OnFail   onTimeout;
  uint8_t  gotoIdx;     // used if onTimeout == GOTO
};

struct Context {        // the "blackboard"
  float theta1, theta2, bearing;
  uint8_t rxCmd;        // decoded AFSK command
  uint8_t retries;
};
```

```cpp
// main.ino
#include "common.h"
#include "Superstructure.h"   // owns Speaker, Mic, Chassis (private)
#include "Sequencer.h"

#define ROBOT_A 1
#define ROBOT_B 2
#define ROBOT_ROLE ROBOT_A    // or read a jumper pin at boot

Superstructure sup;
Sequencer      seq(sup);
Context        ctx;

#if ROBOT_ROLE == ROBOT_A
const Step SEQ[] = {
  { Prim::LISTEN_CHIRP,     0,    10000, OnFail::ABORT, 0 },  // 0
  { Prim::COMPUTE_TDOA,     1,    200,   OnFail::ABORT, 0 },  // 1 -> theta1
  { Prim::TURN,             90,   3000,  OnFail::ABORT, 0 },  // 2
  { Prim::WAIT,             0,    300,   OnFail::ABORT, 0 },  // 3 settle
  { Prim::LISTEN_CHIRP,     0,    10000, OnFail::ABORT, 0 },  // 4
  { Prim::COMPUTE_TDOA,     2,    200,   OnFail::ABORT, 0 },  // 5 -> theta2
  { Prim::COMPUTE_BEARING,  0,    10,    OnFail::ABORT, 0 },  // 6
  { Prim::TURN,             NAN,  3000,  OnFail::ABORT, 0 },  // 7 NAN = use ctx.bearing
  { Prim::DRIVE,            1.0,  4000,  OnFail::ABORT, 0 },  // 8 feet
  { Prim::SEND_AFSK,        CMD_STOP_CHIRP, 1500, OnFail::RETRY, 0 }, // 9
};
#else  // ROBOT_B
const Step SEQ[] = {
  { Prim::SEND_CHIRP,       0, 700,  OnFail::ABORT, 0 },  // 0
  { Prim::LISTEN_AFSK,      0, 2300, OnFail::GOTO,  0 },  // 1 timeout -> step 0
  { Prim::EXEC_RECEIVED_CMD,0, 5000, OnFail::ABORT, 0 },  // 2 then DONE
};
#endif

elapsedMicros tick;
const uint32_t TICK_US = 1000;   // 1 kHz control loop

void setup() {
  Serial.begin(115200);
  sup.begin();                   // init pins, gyro, speaker DAC, mic ADC/DMA
  seq.load(SEQ, sizeof(SEQ) / sizeof(SEQ[0]), &ctx);
  seq.start();
}

void loop() {
  if (tick >= TICK_US) {
    tick -= TICK_US;
    sup.update();                // subsystems advance, settle timer, results
    seq.update();                // decides the next step
  }
  // AFSK/serial command sources go through the same gate:
  // if (rxFrameReady()) onAfskCommand(decode());   // -> sup.request() / sup.emergencyStop()
}
```

#### Sequencer core, showing the GOTO and DONE handling

```cpp
void Sequencer::update() {
  switch (state_) {
    case REQUEST:
      if (sup_.request(steps_[i_], ctx_)) state_ = WAIT;   // may be refused, so retry next tick
      break;

    case WAIT: {
      Result r = sup_.takeResult();
      if (r == Result::RUNNING) break;
      if (r == Result::SUCCESS) { advance(); break; }

      const Step& s = steps_[i_];
      switch (s.onTimeout) {
        case OnFail::GOTO:  i_ = s.gotoIdx; state_ = REQUEST; break;
        case OnFail::RETRY: if (++ctx_->retries < 3) state_ = REQUEST; else state_ = FAILED; break;
        case OnFail::SKIP:  advance(); break;
        default:            state_ = FAILED; break;
      }
      break;
    }
    case DONE: case FAILED: break;   // wait for a new sequence or reset
  }
}

void Sequencer::advance() {
  ctx_->retries = 0;
  if (++i_ >= n_) state_ = DONE; else state_ = REQUEST;
}
```

#### Notes for the Teensy

- **Sampling is not gated, results are.** Run mic sampling with DMA or the Audio library in the background. The superstructure only lets `LISTEN_*` steps consume those buffers, and flushes the buffer when a `LISTEN` step starts, so stale audio is never decoded.
- **B's timing.** The 0.7 s chirp plus 2.3 s listen gives a 3 s cycle. Decide whether the speaker ring-down settle time comes out of the 2.3 s listen window or adds to it. As written, the superstructure's settle delay is added between steps, which stretches the cycle slightly.
- **Robot B can still move.** Since `EXEC_RECEIVED_CMD` can turn into `TURN` or `DRIVE`, B keeps the chassis subsystem. If B will never move, you can compile it out.
- **Role selection.** A compile flag is simplest. A jumper pin read in `setup()` lets one binary serve both robots.

If you want, I can write the `Superstructure::request()` and `update()` bodies for the Teensy, including the settle timer and the gyro-based `TURN`.

---

## Q5: LISTEN_CHIRP as one state or three?

> LISTEN_CHIRP is made up of 3 steps (rough correlation, mix-FFT, find correlation). In this case do you recommend to separate them into three states or keep them under one state? Answer in one paragraph.

### A5

Keep them as one primitive (`LISTEN_CHIRP`) from the superstructure's and sequencer's point of view, but give the Mic subsystem an internal sub-state machine with three states. The three stages are a single pipeline on one data buffer and one subsystem, so nothing else can legally run between them, and exposing them upward would only force the sequencer to carry details it can't act on (it has no meaningful way to branch on "rough correlation passed"). Inside the Mic subsystem, though, separate states (`ROUGH_CORR → MIX_FFT → FINE_CORR`) pay off: each stage has a different cost, so a cheap rough correlation can reject noise early and skip the expensive FFT, each can be time-sliced across ticks so the 1 kHz loop never blocks, and each can have its own timeout and debug counters. The rule of thumb is that you should split states only when something outside the subsystem needs to observe or interrupt at that boundary. If you later need that, for example to abort mid-pipeline on a STOP or to log which stage rejected a candidate, expose a read-only `progress()` or `stage()` for diagnostics rather than promoting the stages to superstructure states. The primitive still reports a single result: `SUCCESS` with the detected chirp's timing and both channels' data, or `TIMEOUT` if no chirp is confirmed within the window.

---

## Summary of the architecture

```
Sequencer        what comes next? (step table, context, failure policy)
    |
Superstructure   is it safe right now? (single owner, settle timer, e-stop)
    |
Subsystems       Speaker / Mic / Chassis (private, report SUCCESS/TIMEOUT/FAIL)
                 Mic internal sub-states: ROUGH_CORR -> MIX_FFT -> FINE_CORR
```


---

## Addendum: `Superstructure::request()` and `update()` implementations 
---> This is more for FRC SW architecture, rather than Teensy. 


> Write the `Superstructure::request()` and `update()` bodies for the Teensy, including the settle timer and the gyro-based `TURN`.

This section covers `request()` and `update()` plus the helpers they depend on (`finish()`, `ownerFor()`, `settleMsAfter()`). The full source, including the gyro-based `Chassis::TURN`, is in `Superstructure.h` and `Superstructure.cpp`.

### Helpers used by `request()` and `update()`

```cpp
namespace cfg {
  constexpr uint32_t SETTLE_CHASSIS_MS   = 300;  // motor noise + vibration
  constexpr uint32_t SETTLE_SPEAKER_MS   = 150;  // speaker ring-down
  constexpr uint32_t SEND_CHIRP_GRACE_MS = 200;  // watchdog margin for SEND_CHIRP
}

static inline int idx(Owner o) { return (int)o; }

// Which subsystem does a primitive need? NONE = superstructure-only work.
static Owner ownerFor(Prim p) {
  switch (p) {
    case Prim::SEND_CHIRP:
    case Prim::SEND_AFSK:        return Owner::SPEAKER;
    case Prim::LISTEN_CHIRP:
    case Prim::COMPUTE_TDOA:
    case Prim::LISTEN_AFSK:      return Owner::MIC;
    case Prim::TURN:
    case Prim::DRIVE:            return Owner::CHASSIS;
    default:                     return Owner::NONE;   // WAIT, COMPUTE_BEARING, EXEC_RECEIVED_CMD
  }
}

static uint32_t settleMsAfter(Owner o) {
  switch (o) {
    case Owner::CHASSIS: return cfg::SETTLE_CHASSIS_MS;
    case Owner::SPEAKER: return cfg::SETTLE_SPEAKER_MS;
    default:             return 0;
  }
}

bool Superstructure::allSubsystemsIdle() const {
  for (int i = 1; i < idx(Owner::COUNT); i++)
    if (sub_[i] && !sub_[i]->isIdle()) return false;
  return true;
}
```

### `request()`

The only way to start work. Returns `false` meaning "not now, ask again next tick". Returns `true` meaning accepted; the outcome arrives through `takeResult()`.

```cpp
bool Superstructure::request(const Step& in, Context* ctx) {
  if (busy_) return false;                       // covers owner-less steps (WAIT etc.) too

  Step s = in;
  if (!resolve(s, ctx)) {                        // e.g. unknown received command
    ctx_ = ctx; active_ = in; owner_ = Owner::NONE; busy_ = true;
    finish(Result::FAIL);
    return true;                                 // accepted; result = FAIL
  }

  Owner need = ownerFor(s.prim);
  if (need != Owner::NONE) {
    // Quiet period: block a *different* subsystem from starting until the last one settles.
    if (settling() && need != lastOwner_) return false;

    // Invariant check: nobody may be active. If this ever fails, we have a bug elsewhere.
    if (!allSubsystemsIdle()) return false;
  }

  active_  = s;
  ctx_     = ctx;
  owner_   = need;
  busy_    = true;
  startMs_ = millis();
  uint32_t grace = (s.prim == Prim::SEND_CHIRP) ? cfg::SEND_CHIRP_GRACE_MS : 0;
  deadlineMs_  = startMs_ + s.timeout_ms + grace;
  lastResult_  = Result::RUNNING;

  if (need != Owner::NONE) sub_[idx(need)]->start(s, ctx);
  return true;
}
```

**How the settle timer works in `request()`**

- `finish()` sets `settleUntil_` when an owner releases, and `lastOwner_` records who it was.
- A request for a different subsystem is refused while `settling()` is true. For example, `LISTEN_CHIRP` cannot start until 300 ms after the chassis stops.
- A request for the same subsystem starts immediately, so a `TURN` followed by a `DRIVE` has no gap.
- `settling()` uses a signed subtraction, `(int32_t)(millis() - settleUntil_) < 0`, so it stays correct when `millis()` wraps.

### `update()`

Called every control tick (1 kHz). Never blocks.

```cpp
void Superstructure::update() {
  // Heading integration always runs. The gyro is a sensor, not an "active" subsystem.
  chassis_.tickHeading(micros());

  if (!busy_) return;
  uint32_t now = millis();
  Result r = Result::RUNNING;

  switch (active_.prim) {
    case Prim::WAIT:             // timeout_ms is the wait duration
      r = ((int32_t)(now - (startMs_ + active_.timeout_ms)) >= 0) ? Result::SUCCESS : Result::RUNNING;
      break;

    case Prim::COMPUTE_BEARING:  // pure math, finishes immediately
      r = computeBearing(*ctx_) ? Result::SUCCESS : Result::FAIL;
      break;

    default:                     // subsystem-owned primitive
      r = sub_[idx(owner_)]->update();
      if (r == Result::RUNNING && (int32_t)(now - deadlineMs_) >= 0)
        r = Result::TIMEOUT;     // watchdog: no primitive may run past its deadline
      break;
  }

  if (r == Result::SUCCESS && active_.prim == Prim::COMPUTE_TDOA) {
    // Mic stored theta1/theta2. Stamp the heading it was measured at.
    if ((int)active_.param == 1) ctx_->heading1 = chassis_.yawDeg();
    else                         ctx_->heading2 = chassis_.yawDeg();
  }

  if (r != Result::RUNNING) finish(r);
}
```

### `finish()`: the single release path

Success, timeout, failure, and emergency stop all end here, so the settle timer always starts and the hardware is always left safe.

```cpp
void Superstructure::finish(Result r) {
  if (owner_ != Owner::NONE) {
    Subsystem* sub = sub_[idx(owner_)];
    if (r != Result::SUCCESS) sub->abort();      // make hardware safe on any abnormal end
    sub->forceIdle();                            // never free the floor until truly idle
    settleUntil_ = millis() + settleMsAfter(owner_);   // start quiet period
    lastOwner_   = owner_;
  }
  owner_      = Owner::NONE;
  busy_       = false;
  lastResult_ = r;
}

void Superstructure::emergencyStop() {
  chassis_.forceIdle();                          // motors off no matter what
  if (busy_) finish(Result::ABORTED);
}

Result Superstructure::takeResult() {
  Result r = lastResult_;
  lastResult_ = Result::RUNNING;                 // one-shot
  return r;
}
```

### Design notes

1. **`busy_` vs `owner_`.** `request()` checks `busy_`, not `owner_`. Steps like `WAIT` and `COMPUTE_BEARING` have no subsystem (`owner_ == NONE`) but still occupy the superstructure. Checking only `owner_`, as in the earlier sketch, would let a second request start during a `WAIT`.
2. **Belt-and-braces invariant.** `allSubsystemsIdle()` re-checks every subsystem even though `busy_` should already guarantee it. If a subsystem ever fails to return to idle, the request is refused instead of two subsystems running together.
3. **Watchdog.** Every subsystem-owned primitive has a deadline of `timeout_ms`, plus a small grace for `SEND_CHIRP` (where `timeout_ms` is the chirp duration). A hung subsystem becomes `TIMEOUT`, which `finish()` turns into `abort()` and `forceIdle()`.
4. **Timing semantics.** For `WAIT`, `timeout_ms` is the duration, and the primitive reports `SUCCESS` when it elapses. For `LISTEN_*`, `timeout_ms` is a limit, and reaching it reports `TIMEOUT`. This is why Robot B's `LISTEN_AFSK` step uses `OnFail::GOTO` for its timeout.
5. **Heading stamp.** After a successful `COMPUTE_TDOA`, the superstructure records the chassis yaw in the context. `computeBearing()` uses these headings to combine the two TDOA angles and resolve the front/back ambiguity.
6. **Settle time and Robot B's cycle.** The settle delay adds to Robot B's 0.7 s chirp plus 2.3 s listen cycle. If you need exactly 3 s, shorten `SETTLE_SPEAKER_MS` or subtract it from the listen window.

