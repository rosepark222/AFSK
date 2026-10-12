##  
--- 
### Q 

Currently requestUserStep is called by either onUserCommand or serviceUserCommands. 

how about  
1, onUserCommand does not call requestUserStep but queue up immediately
2, serviceUserCommands calls requestUserStep 

In this way, we only have one chain of command : onUserCommand  -- serviceUserCommands  -- requestUserStep 
Also, I can think of serviceUserCommands as an another Sequencer.
 

### A 
Yes, that is a much cleaner architecture! Moving the queuing logic entirely out of onUserCommand() and letting serviceUserCommands() handle the calls to requestGroup() creates a single, predictable linear chain:
onUserCommand() ---> Queue Up Immediately ---> serviceUserCommands() ---> requestGroup()

Why this design is cleaner:
Single Responsibility: onUserCommand() becomes purely an input parser. Its only job is to translate incoming characters (like t or d) into a Step object and drop it into a queue or slot, completely detached from the complexities of locks, states, or whether the subsystem is currently busy.

Decoupled Timing: The background service loop (serviceUserCommands()) acts as the gatekeeper that attempts execution. If the superstructure rejects the command with BUSY or SETTLING, it simply remains in the queue and tries again on the next pass, keeping all retry logic centralized away from the raw hardware interrupt/serial read handlers.


onUserCommand() queues commands; it does not start ordinary motion commands directly.
serviceUserCommands() is the only function that starts or retries manual commands.
BUSY and SETTLING requests remain queued and are retried.
A second motion command is ignored while another command is queued.
x or ! triggers an emergency stop.
The main loop updates the autonomous sequencer before servicing manual commands, so the sequencer can consume its result before manual-command processing.


### Q 
Why the below is needed? 
The main loop updates the autonomous sequencer before servicing manual commands, so the sequencer can consume its result before manual-command processing.
 
### A 
It is not strictly needed, but I recommend keeping seq.update() before serviceUserCommands() for this design.
The important issue is how the autonomous sequencer and manual-command service share the Superstructure.
Why the order matters
Suppose an autonomous action has just finished.
1. sup.update() detects completion and stores the result.
2. seq.update() consumes that result using sup.pollResult() and advances the autonomous sequence.
3. serviceUserCommands() attempts to start a queued manual command.
This order lets the sequencer process its completed action before the manual-command service attempts to acquire resources.
What if we reverse the order?
If serviceUserCommands() runs first, it may start a manual command immediately after sup.update() releases the autonomous action's resources. Then seq.update() processes the autonomous result and might request its next action.
The superstructure's resource checks should prevent conflicting actions from starting simultaneously, so reversing the order is not inherently unsafe. However, it can make the autonomous sequencer encounter BUSY or SETTLING and retry later.

One important distinction: this ordering does not guarantee that manual commands wait until the entire autonomous sequence finishes. It only gives the autonomous sequencer first opportunity to request its next action.
If your intended policy is that a manual command must wait until the entire autonomous sequence finishes, we should add an explicit policy check in serviceUserCommands(), such as checking whether seq.state() is DONE or FAILED. That is a separate design choice from the order of these calls.

---
### Q 
How does Sequencer knows advance to the next primitive when previous one is done? 

### A 
In this codebase, the Sequencer advances to the next primitive by actively polling the progress of the current step (or group of steps) inside its update() method.

--- 
### Q 
sup.requestGroup is called by two requestors -- Sequencer.update and requestUserStep (from onUserCommand or serviceUserCommands for queued up user commands). 
Does the robot code have two sources for the requestor—one from user input and the other from the sequencer?

### A 
Yes, exactly! In this architecture (as implemented in [seq_sup_sub_case2.cpp](https://github.com/rosepark222/AFSK/blob/main/teensy/doc/seq_sup_sub_case2.cpp#L615)), the system has **two distinct command sources** that both feed into the exact same safety gate:

#### 1. The Two Request Sources
* **The Autonomous Sequencer (`Sequencer`):** 
  * Automatically walks through a predefined script table of steps (e.g., listening for chirps, turning, driving, or sending AFSK packets).
  * Calls `sup.requestGroup()` to drive the robot through its programmed routine.
* **Manual User Input (`onUserCommand()` / Serial):** 
  * Listens to incoming characters from the serial monitor (such as pressing `t` to turn, `d` to drive, or `x` for an emergency stop).
  * Wraps the input into a `Step` and calls `requestUserStep()`, which routes through `sup.requestGroup()`.

#### 2. The Unified Safety Gate (`Superstructure`)
Both sources must pass through the **Superstructure's** `requestGroup()` method. This ensures that whether a command comes from the autonomous script or human keyboard input, it is handled with identical safety rules:
* **Atomic Validation:** Checks whether the required subsystems (Speaker, MicProxy, or Chassis) are available all at once.
* **Conflict & Settling Protection:** Enforces acoustic conflict tables and active settle timers to prevent manual commands from overriding or crashing a running routine without proper checks.



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
