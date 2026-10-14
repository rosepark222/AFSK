

 

## summary relationship between tasks and deadline for each sub.

###  speaker: 
if spkStartAfsk takes longer than the endMs_, it pass SUCCESS to sup -- take a much as it needs (no hard deadline). 
if it takes less than endMs_, it tells sup that speaker is still RUNNING.

spkStartChirp is not implemented yet, but should take a much time it needs -- not timed.

--> for speaker task, deadline does not mean anything. Speaker takes a much time it need to send a chirp or fsk. 

### mic:  
if listenChirp   finds a chirp  before deadline, return SUCCESS
if it cannot find a msg before deadline, return TIMEOUT 

if listenAfsk finds a fsk before deadline, return SUCCESS
currently, afskDecodeFrame is blocking and deadlinePassed is not checked. Thus it waits afsk forever. To fix it, deadline needs to be passed down to the afskDecodeFrame (just like the manager pass down task and deadline to us).

--> use timeout from Step and return TIMEOUT within the task in the sub

--> I believe there is no need to send deadlineMs to afskDecodeFrame , if I place afskDecodeFrame  (and other dsp codes) as  member functions because deadlineMs_  is accessible within the class scope. 

### chassis:  
In chassis,  updateTurn and updateDrive are not timed. 

--> driver take a much it needs

---
## Converting an independent test sketch loop into a framework method bound by a hard deadline
### Q:
generally, sub test code for mic has this format:
```cpp
void loop() {
  if (queue1.available()) {
``` 
if I want to convert it to a function in the robotics framework having a hard deadline, I need to 
```cpp
Result listenAFSK() {
  micFlush();
  afsk::reset();  // equivalent to setup for afsk listening
  while( queue1.available() && !deadlinePassed()) { 
``` 
Do you agree?

### A:
Yes, **I completely agree**. 

Converting an independent test sketch loop into a framework method bound by a hard deadline naturally maps to a `while` loop that conditions on both data availability (`queue1.available()`) and your time window (`!deadlinePassed()`).

However, looking at how the underlying Audio library queue works (as seen in your `fsk_packet_rx.ino` implementation), there is one important nuance to keep in mind: **`queue1.available()` only tells you if a block is ready in the queue, but you still have to read and free the buffer inside your loop.**

Here is how that structure translates cleanly into your `Mic` class member function:

```cpp
Result listenAfsk() {
    micFlush();
    afskReset(); // Reset decoder state machine

    while (!deadlinePassed()) {
        feedWatchdog();

        // If a new audio block is available from the Teensy Audio Library queue
        if (queue1.available()) {
            int16_t* block = queue1.readBuffer();
            
            // Process the 128 samples in this block
            for (int i = 0; i < 128; i++) {
                float x = block[i] / 32768.0f;
                
                // Run your demodulation, clock recovery, and frame assembly here...
                // If a valid frame is successfully decoded and saved to ctx_->rxCmd:
                if (decodeSampleAndCheckFrame(x, ctx_->rxCmd)) {
                    queue1.freeBuffer();
                    return Result::SUCCESS;
                }
            }
            
            queue1.freeBuffer(); // Crucial: always free the buffer after reading!
        }
    }

    return Result::TIMEOUT; // Timed out before finding a frame
}
```

Key Considerations for this Approach:
Buffer Management: Whenever you call queue1.readBuffer(), you must call queue1.freeBuffer() afterward, or the Teensy audio system will quickly run out of internal memory buffers.

Granularity: Because a single audio block contains 128 samples, checking !deadlinePassed() at the start of every block (or even per sample inside the inner loop) ensures the deadline is strictly enforced without blocking indefinitely waiting for hardware.

---

<details>
<summary>Click to reveal hidden Q/A which is not needed if I place task in the class</summary>



**Q** 
if listenAfsk finds a fsk before deadline, return SUCCESS. 
currently, afskDecodeFrame is blocking and deadlinePassed is not checked. Thus it waits afsk forever.

What is the way for it check timeout and exit if fsk is not detected ?

**A** 
Looking at your `listenAfsk()` method in [seq_sup_sub_case1.cpp](https://github.com/rosepark222/AFSK/blob/main/teensy/doc/seq_sup_sub_case1.cpp#L287), it actually **already wraps `afskDecodeFrame(cmd)` inside a `while (!deadlinePassed())` loop** that checks `deadlinePassed()` on every iteration and feeds the watchdog. 

However, if `afskDecodeFrame()` itself is written as a heavy blocking function that hangs internally (waiting indefinitely for samples or bytes without checking time), then execution gets trapped *inside* `afskDecodeFrame()`, preventing the outer `while` loop from checking the deadline.

### How to Fix It

To make sure it correctly respects the timeout, `afskDecodeFrame()` needs to take the deadline into account or be broken down into incremental non-blocking checks:

1. **Pass the deadline (or remaining time) into `afskDecodeFrame()`:**
```cpp
   static bool afskDecodeFrame(uint8_t& cmd, uint32_t deadlineMs) {
       while ((int32_t)(millis() - deadlineMs) < 0) {
           feedWatchdog();
           // Read or parse audio samples incrementally here...
           // If a valid frame is found: return true;
           // If no data is available yet: small yield or continue
       }
       return false; // Timed out inside frame decode
   }
```
  
2. **Update your call in listenAfsk():**
```cpp
Result listenAfsk() {
    micFlush();
    // Delegate the timeout checking down into the decoder loop
    uint8_t cmd;
    if (afskDecodeFrame(cmd, deadlineMs_)) {
        ctx_->rxCmd = cmd;
        return Result::SUCCESS;
    }
    return Result::TIMEOUT;
}

This is the hidden content that appears when you click the dropdown.
</details>
```

By pushing the deadline check directly into the decoding logic or feeding samples step-by-step, it will exit cleanly with Result::TIMEOUT rather than waiting forever.
  
