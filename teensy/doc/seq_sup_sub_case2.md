## Things to improve for seq_sup_sub_case2.cpp which is a sup/sub/seq assuming dedicated DSP -- handle heavy work and main cpu can poll its state. 

## Q: Can a subsystem change its own state, instead of the superstructure changing it?

> so far super changes sub state. Is there a way sub changes its state itself?

### A

Yes, and it is the more common arrangement, especially in FRC. In the current code the superstructure writes every subsystem state, but a subsystem can own its own.

#### Who writes what today

In `seq_sup_sub_case2.cpp`, the superstructure does all of this:

| Change | Done by | Where |
|---|---|---|
| `IDLE` → `ACTIVE` | superstructure | `requestGroup()` sets `sl.state` |
| `ACTIVE` → `IDLE` | superstructure | `finish()` sets `sl.state`, calls `forceIdle()` |
| `IDLE` → `SETTLING` (holdoff) | superstructure | `finish()` of a conflicting peer |
| `SETTLING` → `IDLE` | superstructure | `update()` checks the timer |

The subsystems only answer `update()` with a `Result`. They also hold a private state of their own (`busy_`, `mode_`), so each subsystem's real state is already duplicated in the superstructure's slot, and the two copies have to be kept in agreement by `forceIdle()`.

#### The pattern: separate "wanted" from "current"

Each variable gets one writer:

- **Wanted state:** written by the superstructure through requests such as `request(step)` and `requestAbort()`.
- **Current state:** written only by the subsystem, inside its own `update()`.

This is the usual FRC arrangement. The superstructure sets a goal and reads where the subsystem is. It never assigns the subsystem's state.

A base class can own all the transitions, so every subsystem moves through the same states. The subclasses only fill in what is specific to them:

```cpp
enum class SubState : uint8_t { IDLE, ACTIVE, DONE, HOLDOFF };

class Subsystem {
public:
  // read by anyone
  SubState state() const { return state_; }

  // requests from the superstructure: the subsystem decides what to do
  bool request(const Step& s, Context* ctx) {
    if (state_ != SubState::IDLE) return false;          // refuses by itself
    deadlineMs_ = millis() + s.timeout_ms + graceMs(s);
    doStart(s, ctx);
    state_ = SubState::ACTIVE;
    return true;
  }
  void requestAbort() { if (state_ == SubState::ACTIVE) finishWith(Result::ABORTED); }

  Result takeResult() {                                  // acknowledge: DONE -> IDLE or HOLDOFF
    if (state_ != SubState::DONE) return Result::RUNNING;
    Result r = result_;
    state_ = holdoffRemaining() ? SubState::HOLDOFF : SubState::IDLE;
    return r;
  }
  void onPeerFinished(Sub who) { uint32_t ms = holdoffAfter(who); if (ms) startHoldoff(ms); }

  // the ONLY place state_ changes by itself, called every loop pass
  void update() {
    if (state_ == SubState::ACTIVE) {
      Result r = doStep();
      if (r == Result::RUNNING && deadlinePassed()) r = Result::TIMEOUT;   // own deadline
      if (r != Result::RUNNING) finishWith(r);
    } else if (state_ == SubState::HOLDOFF && !holdoffRemaining()) {
      state_ = SubState::IDLE;
    }
  }

protected:
  virtual void     doStart(const Step&, Context*) = 0;
  virtual Result   doStep() = 0;                          // RUNNING / SUCCESS / FAIL
  virtual void     doStop() = 0;                          // exit action: leave hardware safe
  virtual uint32_t holdoffAfter(Sub who) const { return 0; }

private:
  void finishWith(Result r) { doStop(); result_ = r; state_ = SubState::DONE; }
  SubState state_ = SubState::IDLE; Result result_ = Result::RUNNING; uint32_t deadlineMs_ = 0;
  // holdoff timer helpers omitted
};
```

#### The holdoff, where it belongs

This fixes the confusion from the earlier question about the Mic entering `SETTLING`. The Mic knows how long it must stay deaf after each peer, so that knowledge goes in the Mic, and the Mic changes its own state:

```cpp
class Mic : public Subsystem {
  uint32_t holdoffAfter(Sub who) const override {
    if (who == SUB_SPK) return 150;                       // speaker ring-down
    if (who == SUB_CHS && MOTOR_NOISE_MASKS_CHIRP) return 300;
    return 0;
  }
  ...
};
```

The superstructure only forwards the news: when a subsystem's result is taken, it calls `onPeerFinished(who)` on the others. It no longer needs the settle-time table or any holdoff timer.

#### What the superstructure still does

- **Admission control.** It decides whether a request may start, using the conflict table. A subsystem cannot decide this, because the rule is about other subsystems.
- **Group atomicity.** A group must start all branches or none. It checks `state() == IDLE` on every branch, then calls `request()` on each. This is safe on one thread. If you ever start work from an interrupt, put a lock around the check and the start.
- **Forwarding results** to the sequencer and **forwarding peer events** for holdoffs.
- **Emergency stop.** It calls `requestAbort()` on everything. Each subsystem still stops itself immediately.

`finish()` in the superstructure shrinks to "collect the result and notify peers", and the `forceIdle()` call and the slot state array go away.

#### Benefits and costs

**Benefits**
- **One copy of the state.** There is no slot state to keep in sync with the subsystem's internal state, and no way for `forceIdle()` to be forgotten.
- **Safety with the hardware.** The exit action `doStop()` runs on every path out of `ACTIVE`, so the chassis motors go off whether the step succeeded, timed out, or was aborted.
- **Local deadlines.** Each subsystem enforces its own deadline, so the superstructure's backstop check is optional.
- **A smaller superstructure** that depends only on the interface.

**Costs**
- **A `DONE` state to acknowledge.** The subsystem stays in `DONE` until `takeResult()`, so a sequencer that forgets to poll leaves it stuck. The e-stop path should clear it.
- **More machinery in the base class.** It is written once, but it is subtle, and it has to treat a blocking `doStep()` (Case 1) the same as a polling one.
- **Harder to see the whole picture.** Transitions are spread across subsystems, so keep `dumpStates()` reading each `state()`.

I'd use this design for new code. If you'd like, I can rewrite `seq_sup_sub_case2.cpp` with the base class, per-subsystem holdoff, and a slimmer superstructure.

---

## Q: Which is easier to read for beginners: slot state or state in the subsystems?

> which one is easier to read for beginners? keeping slot state or keep state in subs ?

### A

For a beginner, keeping the slot state in the superstructure is easier to read. State in the subsystems is the better design as the code grows, but it asks the reader to hold more ideas at once.

#### A readability test

Take the question you just asked: "when does the Mic stop accepting requests, and why?"

- **Slot state in the superstructure:** read one function, `finish()`. Everything is there: the result is recorded, the subsystem is stopped, and the conflicting subsystems are told to wait. You can follow it top to bottom.
- **State in the subsystems:** you have to read the base class `update()`, `takeResult()`, `onPeerFinished()`, the Mic's `holdoffAfter()`, and the superstructure code that forwards the peer notification. That is five places in at least two classes, connected by virtual functions.

#### Comparison

| | Slot state in superstructure | State in subsystems |
|---|---|---|
| Where state changes | one place | the base class, triggered from several places |
| Subsystem code | small and simple: `start`, `update`, `abort` | each subclass fills in hooks (`doStart`, `doStep`, `doStop`, `holdoffAfter`) |
| C++ concepts needed | classes, one interface | inheritance, protected virtual hooks, a base class that calls its subclass |
| Extra state to learn | none | a `DONE` state that must be acknowledged |
| Debugging | print the slot array | print each subsystem's state, then trace who changed it |
| Duplicate state | yes: slot state and the subsystem's own flag can disagree | no |
| Matches FRC habits | partly | closer: goal in, status out |
| Scales to many subsystems | harder: the superstructure grows | easier |

The "duplicate state" row is the real weakness of the central version. Its cost is a bug that a beginner can hit: forgetting to reset one of the two copies. Its benefit is that everything is visible in one file.

#### A simpler middle path

You can remove most of the duplication without the base class. The superstructure stops storing `ACTIVE` and `IDLE` itself and asks the subsystem instead, and it keeps only the holdoff as a timestamp:

```cpp
bool isActive(Sub s)  { return !sub_[slotOf(s)]->isIdle(); }     // ask the subsystem
bool isBlocked(Sub s) { return (int32_t)(millis() - blockedUntil_[slotOf(s)]) < 0; }
```

Then there is no `SlotState` enum, no `SETTLING` state to explain, and a single source of truth for "is it running". The Mic is `IDLE` and also "blocked until time T", which also answers the earlier question about why a state starts after the conflict ends. A beginner reads "blocked until" more easily than a state name.

#### My recommendation

1. **First version:** keep the central design, rename `SETTLING` to `HOLDOFF`, and consider the timestamp version above. It is the easiest to read, trace, and debug.
2. **Move to state in subsystems when** you add more subsystems, when each one needs richer internal states, or when you hit a bug caused by the two copies disagreeing. At that point the extra machinery pays for itself.

I said earlier that I would use state in the subsystems for new code. That is the stronger design in the long run, but the right choice for a new engineer's first implementation is the one they can trace by reading one function. Doing the central version once also teaches why the second one exists, which is the same advice I gave for blocking versus `request()`/`update()`.

---

## Q: Based on the single source of truth rule, which one is better?

> based on the single source of truth rule, which one is better?

### A

By the single-source-of-truth rule alone, state in the subsystems is better. The rule says each fact is stored in exactly one place, and everything else reads it or derives it. The current slot design breaks this for the most important fact.

#### Where the current code has two copies

In `seq_sup_sub_case2.cpp`, "is the Mic running?" is stored twice:

| Copy | Written by |
|---|---|
| `slots_[i].state == ACTIVE` | the superstructure |
| `busy_` inside `MicProxy` (and `mode_` in `Chassis`, `busy_` in `Speaker`) | the subsystem |

Two copies can disagree. The code already knows this: `requestGroup()` has an "invariant check" that compares the two (`sub_[...]->isIdle()`), and `finish()` calls `forceIdle()` to bring them back into agreement. Code that exists to keep copies in sync is the sign of a violated rule.

#### The three options, ranked by the rule

| Option | Is "running?" stored once? | Is "blocked until" stored once? | Verdict |
|---|---|---|---|
| State in the subsystems | yes, in the subsystem | yes, in the subsystem that must wait | satisfies the rule fully |
| Middle path (superstructure asks `isIdle()`, keeps a `blockedUntil` timestamp) | yes, in the subsystem | yes, in the superstructure | satisfies the rule |
| Slot state in the superstructure | no, two copies | yes | violates the rule |

The middle path also satisfies the rule, because each fact still has exactly one owner. The difference between the first two is only who owns the holdoff.

#### Who should own the holdoff

The Mic. "How long I must stay deaf after a speaker transmission" is a property of the Mic's hardware and the room, and the fact that it applies is a fact about the Mic. If the superstructure owns the timestamp, then the Mic's behavior depends on state stored in another class. With the Mic owning it, a subsystem's whole condition (running, done, holding off) is in one object, and `dumpStates()` is just a loop over `state()`.

Other facts in the current slot also have a second home, and the same rule applies:

- **Deadline:** the slot stores `deadlineMs`. The subsystem needs it to enforce its own timeout, so it should hold it, and the superstructure's backstop check goes away.
- **Result:** the slot stores `result` and `hasResult`. With subsystem-owned state, the result lives in the subsystem until it is acknowledged, so the slot's copy is not needed.

#### What stays in the superstructure

Facts that belong to no single subsystem still have one owner, the superstructure:

- the conflict table, which describes relationships between subsystems,
- the group admission decision, which is an action, not stored state,
- the step and context pointers for each running branch, if the subsystem doesn't need to keep them.

#### The caveat

The rule decides which design is correct, but not which is easiest to read. The previous answer recommended the central version for a first implementation because it can be read in one function. That advice was about readability, not about the rule. If you want the design that cannot get out of sync, choose state in the subsystems. If you want something readable that still follows the rule, choose the middle path, because it removes the duplicated state with only a few lines of change.

If you'd like, I can rewrite `seq_sup_sub_case2.cpp` using subsystem-owned state, including the Mic's own holdoff, and delete the slot state array.
