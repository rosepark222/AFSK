# Superstructure Loop Design: Long Tasks and FRC vs Teensy DSP

How should the superstructure's loop be organized when a primitive such as `LISTEN_CHIRP` can last 10 seconds or more and includes heavy DSP (rough correlation, mix-FFT, fine correlation)?

This note records the reasoning, so the design choice is explicit rather than inherited from FRC habits.

---

## 1. Summary

- A primitive being **long** does not by itself break a loop design. What matters is **why** it is long.
- There are **two kinds of "long"**: long in *waiting time* and long in *computing time*. They have different costs and different remedies.
- FRC robots use a fast, fixed-rate, non-blocking loop (`request()`/`update()` style) because they must do many unrelated things at once and answer to a field watchdog.
- This robot has an **exclusion rule** (only one subsystem active at a time, no listening while driving). During `LISTEN_CHIRP` nothing else needs to run, so a primitive may run for as long as it needs. The loop does not need a 1 kHz tick for its own sake.
- Whichever style is chosen, a few requirements remain: keep up with the sample stream, enforce deadlines inside the algorithm, and use a hardware watchdog.

---

## 2. Two kinds of "long"

### 2.1 Long in waiting time

The task takes seconds, but almost all of that time is spent waiting for something to happen.

| Example | What each check does |
|---|---|
| Run the intake until a game piece is detected (FRC) | Read one sensor, return |
| Drive until the arm reaches its position (FRC) | Compare position to goal, return |
| Wait for a chirp to arrive (this robot) | Process the newest audio block with a cheap detector, return |

This kind is cheap. A task can be 10 seconds long and still consist of thousands of tiny checks.

### 2.2 Long in computing time

The task needs a large amount of arithmetic once its input is available.

| Example | Typical cost |
|---|---|
| Camera image processing, AprilTag detection (FRC) | tens to hundreds of ms per frame |
| Trajectory generation (FRC) | tens to hundreds of ms |
| Correlation + FFT + fine correlation on a chirp capture (this robot) | depends on length; may be several ms to tens of ms |

This kind is expensive. A single call can stall everything else for as long as the math takes.

### 2.3 `LISTEN_CHIRP` is both

| Phase | Kind | What to do |
|---|---|---|
| Waiting for a chirp | Waiting | Cheap per-block detector, keeps up with the sample stream |
| Rough correlation, mix-FFT, fine correlation | Computing | Optimize, chunk, block briefly, or run in the background (see section 6) |

Treating the two phases separately is the key insight. The waiting phase is cheap no matter how long it lasts. The computing phase is where you must decide how to schedule the work.

---

## 3. How FRC handles long tasks

FRC does have long tasks. It handles them by making the waiting cheap and moving the computing out of the main loop.

1. **Waiting:** a command's `execute()` is called repeatedly (every 20 ms) and each call checks a condition and returns. `isFinished()` ends it.
2. **Heavy computing goes elsewhere:**
   - **Coprocessor:** vision runs on a Limelight, or a Raspberry Pi / Orange Pi running PhotonVision. The robot code reads only the small result (target angle, tag ID, distance).
   - **Offline computation:** autonomous paths are planned ahead of time (PathPlanner, Choreo) and the robot follows precomputed data.
   - **Background threads:** work that must run on the robot controller runs in a separate thread, and the main loop reads the finished result.
3. **The main loop stays small:** whatever remains in the loop is written to finish within a few milliseconds.

The idea behind all of this is that the main loop never waits on heavy computing. It reads the newest result from a producer running on its own schedule.

### Why FRC needs this

- Many unrelated mechanisms run at the same time (drive, arm, shooter, intake, joystick input).
- The field system disables the robot if the code stalls, and a blocked loop produces "loop time overrun" warnings and degraded behavior.
- So WPILib calls every subsystem's `periodic()` and the command scheduler on a fixed period (20 ms by default), and each call does a small slice of work.

---

## 4. Loop architecture comparison

### 4.1 Concept mapping

| FRC (WPILib command-based) | This robot |
|---|---|
| Subsystem `periodic()` | `Subsystem::update()` |
| Superstructure state machine | `Superstructure` |
| `schedule(command)` / set goal | `request()` |
| Command scheduler, 20 ms loop | `loop()` calling `update()` |
| `SequentialCommandGroup` | `Sequencer` |
| Command `initialize()` / `execute()` / `isFinished()` / `end()` | `start()` / `update()` / result != `RUNNING` / `finish()` |

> **Naming caution:** in FRC, `execute()` is the *repeatedly called, non-blocking* method of a command. In the blocking, run-to-completion design below, a function named `execute()` would mean something different. Use a different name (for example `run()` or `runToCompletion()`) to avoid confusion.

### 4.2 Architecture comparison

| Aspect | FRC robot | This robot (Teensy DSP) |
|---|---|---|
| Concurrency needs | Many mechanisms at once | One subsystem at a time (exclusion rule) |
| Loop rate | Fixed, 20 ms | Whatever the active primitive needs |
| External watchdog | Field control system | Optional hardware watchdog (Teensy 4.x) |
| Heavy computing | Coprocessor, offline, or thread | Same core: chunk it, block briefly, or use a background job |
| Operator input during a task | Always possible (joystick) | Not while the mic is busy (cannot hear a command) |
| What the loop protects | Responsiveness and watchdog | Sample throughput and deadlines |
| Biggest risk | Loop overrun starves other mechanisms | Sample-buffer overflow or a hung primitive |

---

## 5. Two loop styles for this robot

### 5.1 Non-blocking: `request()` / `update()`

Starting the task and doing the work are separate. `update()` is called repeatedly, does one step, and returns.

```cpp
sup.request(step, &ctx);          // set the goal, return immediately

void loop() {
  sup.update();                   // advance the active primitive, return
  seq.update();                   // decide the next step
  // other code can run between calls
}
```

- Easy to abort, easy to add concurrent features, matches FRC practice.
- Every primitive must be written as a state machine.

### 5.2 Blocking: run to completion

The superstructure runs one primitive from start to finish inside a single call.

```cpp
Result Superstructure::run(const Step& s, Context* ctx) {
  if (!resolve(s, ctx)) return Result::FAIL;
  Owner need = ownerFor(s.prim);
  waitForSettle(need);                       // a blocking delay is fine here
  Subsystem* sub = sub_[idx(need)];
  sub->start(s, ctx);
  Result r = sub->runToCompletion(s.timeout_ms);   // blocks; the DSP checks its own deadline
  finishRelease(need, r);                    // abort if not SUCCESS, forceIdle, record settle
  return r;
}

void loop() {
  seq.step();                                // run the next step, apply the failure policy
}
```

- Reads like a normal function and is easier to debug.
- Exclusion holds by construction, because only one primitive runs at a time.
- Nothing can interrupt a primitive unless you add an interrupt-driven abort path.

### 5.3 Trade-offs

| | `request()`/`update()` | Blocking run to completion |
|---|---|---|
| Code complexity | Higher: each task is a state machine | Lower |
| Do other things during a task | Yes | No |
| Abort / interrupt a task | Easy | Needs flags or an ISR |
| Add concurrent features later | Easy | Needs rework |
| Debugging | Harder: reason about state between calls | Easier |

### 5.4 When non-blocking earns its cost

Non-blocking is worth the extra complexity when something must run *between* polls:

- Receiving a `STOP` while driving (this conflicts with the exclusion rule, so it may never apply)
- Blinking an LED or reporting status during a long task
- A hardware button that must abort any task
- Logging or telemetry at a steady rate

If none of these apply, blocking is a valid choice.

### 5.5 A reasonable middle path

Keep the `request()`/`update()` API between layers because it is clean and common in FRC. Let each primitive's `update()` do as much work as it needs:

- The Mic can run its long DSP inside one `update()` call.
- The Chassis can run its gyro control loop at a fixed rate inside its own call.

This keeps the structure while allowing individual pieces to become more responsive later.

---

## 6. If one DSP algorithm is slow

Measure first, then choose the cheapest option that works.

| Worst-case time | Approach |
|---|---|
| < 0.5 ms | Run it in `update()` as is |
| 0.5 to ~5 ms | Chunk it, or optimize with CMSIS-DSP and decimation |
| ~5 to 50 ms | Chunk if easy. Otherwise block while the chassis is idle, using a snapshot buffer |
| > 50 ms | Background job with polling and cooperative cancel |

### 6.1 Make the algorithm cheaper

- Use CMSIS-DSP (`arm_cfft_f32`, `arm_correlate_f32`, and similar). Q15 fixed point is faster still if precision allows.
- Correlate with an FFT (multiply spectra, inverse FFT) instead of a direct sliding sum: O(N log N) instead of O(N²).
- Decimate before the expensive stage. After dechirp mixing, the beat tone is low frequency, so low-pass and decimate first.
- Reject early with a cheap energy or rough-correlation test.
- Keep hot code in ITCM and hot data in DTCM. RAM2 (`DMAMEM`) has more wait states for repeated work.

### 6.2 Split into chunks

- Correlation one block of lags at a time
- FFT one stage per call, or one channel per tick
- Overlap-save convolution (chunked by design)

Each call does one chunk and returns `RUNNING`.

### 6.3 Block briefly while the chassis is idle

Acceptable up to roughly 20 to 50 ms, because:

- DMA keeps filling the audio ring buffer (it must be large enough to cover the stall)
- Heading is not changing, because the robot is stationary
- The watchdog fires late by at most the stall length

Add the stall time to the step's `timeout_ms`.

### 6.4 Background job

For long work that cannot be chunked, run it on a frozen snapshot in a worker, and have `Mic::update()` poll for completion:

```cpp
class Mic : public Subsystem {
  volatile enum { J_IDLE, J_RUNNING, J_DONE } job_ = J_IDLE;
  volatile bool cancel_ = false;
  volatile uint32_t gen_ = 0;           // generation id: detects stale jobs

  static void worker(void* p);          // heavy DSP on the frozen snapshot

  Result update() override {
    switch (stage_) {
      case WAIT_CHIRP:
        if (candidateDetected()) {
          freezeSnapshot();             // copy to a buffer the worker owns
          cancel_ = false; job_ = J_RUNNING; gen_++;
          threads.addThread(worker, this, 4096);   // TeensyThreads
          stage_ = PROCESSING;
        }
        break;
      case PROCESSING:                  // cheap poll
        if (job_ == J_DONE) return result_;
        break;
    }
    return Result::RUNNING;
  }

  void abort() override { cancel_ = true; }   // worker checks between blocks
};
```

Rules:

1. The worker owns the snapshot, so there are no locks to get wrong.
2. Cancellation is cooperative: the worker checks `cancel_` between blocks, and a stale generation id means its result is discarded.
3. The superstructure still treats the Mic as busy while the job runs, so exclusion is preserved.
4. DMA and audio interrupts must have higher priority than the worker.

---

## 7. Requirements that do not go away

Dropping the fixed tick does not remove these:

1. **Keep up with the sample stream.** Each audio block must be processed before the next one arrives, or the ring buffer overflows and the chirp can be missed. The real-time constraint is sample throughput, not tick rate.
2. **Enforce the deadline inside the algorithm.** If nothing polls from outside, the detection loop must check `millis()` against its deadline every block and return `TIMEOUT`.
3. **Use a hardware watchdog.** On the Teensy 4.x, feed the watchdog from inside long loops. A hang then resets the robot with the motors off.
4. **Provide an abort path if one is ever needed.** A stop during a long primitive requires an ISR that cuts the motors and sets a cancel flag the DSP checks.
5. **Handle overruns in the loop.** Do not replay a backlog of ticks after a slow call:

```cpp
void loop() {
  if (tick >= TICK_US) {
    tick -= TICK_US;
    if (tick > 5 * TICK_US) tick = 0;   // overrun: discard the backlog
    sup.update();
    seq.update();
  }
}
```

6. **Chassis control still wants timely gyro integration.** Either run a self-contained gyro loop at a fixed rate (200 to 500 Hz) inside `Chassis`, or integrate the gyro in an `IntervalTimer` interrupt.

---

## 8. Recommended starting point

1. Keep the layer structure: Sequencer → Superstructure → Subsystems, with the single `request()` gate and one release path.
2. Let each primitive take the time it needs. Do not force the Mic's long DSP into short slices unless measurements show a problem.
3. Measure worst-case time per primitive with `micros()` and log the maximum.
4. Add deadline checks inside every long loop, plus the hardware watchdog.
5. Move heavy computation to a chunked or background form only when the numbers call for it.
6. Start with a blocking version of one primitive (for example `TURN`), get it working on the robot, then convert it to `request()`/`update()` and compare. Doing it once by hand teaches more than a diagram.





---

## Q: Offloading DSP to a second Teensy: long-in-time becomes long-in-wait

> You have explained the difference in two kinds of "long":
> 2.1 Long in waiting time (long-in-wait)
> 2.2 Long in computing time (long-in-time)
>
> `LISTEN_CHIRP` is a long-in-time task.
> If I have a dedicated DSP machine (a secondary Teensy), I can convert it to long-in-wait.
> Then, I can do true request/update and interleave multiple tasks, such as `LISTEN_CHIRP` and `TURN`.
> In that sense, update is: update the subsystem state which is working on long-in-wait tasks.

### A

Yes. This is the same move FRC makes with a vision coprocessor: moving the computation to another processor turns a long-in-time task into a long-in-wait task from the main Teensy's point of view. Its `update()` then only has to poll for a result, which is cheap.

And `update()` means what you said: advance the state of each subsystem that is working on a long-in-wait task. It never does the heavy work itself. The work happens in the motors, the DMA audio, or the DSP Teensy, and `update()` checks on it.

#### What changes

**1. The Mic becomes a proxy for the DSP Teensy.** The DSP Teensy owns the microphones, and the main Teensy only sends commands and reads results over UART or SPI:

```cpp
Result Mic::update() {
  while (link.available()) parser.feed(link.read());     // cheap byte parsing

  switch (st_) {
    case WAITING:
      if (parser.hasChirpResult()) { fill ctx (timing, tdoa); return Result::SUCCESS; }
      if (!parser.heartbeatOk())   return Result::FAIL;  // DSP board died or went silent
      break;
  }
  return Result::RUNNING;                                // every call is microseconds
}
```

You now need a small protocol: `START_LISTEN_CHIRP`, `ABORT`, `RESULT`, a CRC, and a heartbeat so a dead DSP board is detected.

**2. The exclusion rule splits into two causes.** There were two reasons for "one subsystem at a time", and offloading removes only one:

- **CPU contention:** this goes away with the second Teensy.
- **Acoustic contention:** this stays. The speaker deafens the mics, and motor noise can mask a chirp.

So the single `owner_` becomes a conflict matrix:

| | Speaker | Mic | Chassis |
|---|---|---|---|
| **Speaker** | | conflict | allowed |
| **Mic** | conflict | | conflict, unless motor noise is tolerable |
| **Chassis** | allowed | conflict | |

Under the strict acoustic rule, the only new combination you get is Speaker + Chassis. Your `LISTEN_CHIRP` + `TURN` example works only if the DSP can find the chirp through motor noise. A matched filter has processing gain, so this is plausible, and you can test it.

**3. `request()` checks conflicts, not a single owner.** Replace `owner_` with an `activeMask_`. A request is accepted if the new subsystem doesn't conflict with any active one.

**4. The sequencer needs a way to overlap steps.** One active step at a time can't interleave anything. You need either a parallel group (start `LISTEN_CHIRP` and `TURN` together, finish when both finish, like WPILib's `ParallelCommandGroup`) or separate start and await steps.

**5. Measurements must be timestamped.** If you listen while turning, the TDOA angle is relative to the heading at the moment the chirp arrived, not the heading when the primitive ended. The main Teensy should keep a short history of gyro yaw, and the DSP's detection time must be mapped to the main Teensy's clock. Otherwise the bearing calculation will be wrong.
