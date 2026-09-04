


---------------------------------------------------------------------------
---------------------------------------------------------------------------
---------------------------------------------------------------------------
---------------------------------------------------------------------------
---------------------------------------------------------------------------
---------------------------------------------------------------------------
9/3
In embedded systems like Teensy, it is often better to allocate one large memory pool up front and manage it yourself if your application has distinct phases that never overlap.

Why:

Teensy has malloc() and free(), but there is no OS memory manager running in the background.
Dynamic allocation can cause heap fragmentation over time.
If Phase 1 allocates and frees many differently-sized blocks, Phase 2 may later be unable to obtain one large contiguous block even though enough total memory exists.
A statically allocated pool gives you deterministic behavior and avoids fragmentation.

cleaner approach is to use a union:
union
{
    struct {
        float fftBuf[16384];      // 64KB
        int16_t samples[32768];   // 64KB
    } phase1;                     // total = 128KB

    struct {
        float spectrum[32768];    // 128KB
    } phase2;                     // total = 128KB

    struct {
        uint8_t packetBuf[131072];// 128KB
    } phase3;                     // total = 128KB
} work;


All three point to the same starting byte of the union storage.
&work.phase1.fftBuf[0]
&work.phase2.spectrum[0]
&work.phase3.packetBuf[0]


---------------------------------------------------------------------------


https://share.google/aimode/6uXjbDO7qFYJuvZ02

rosep@MSI:/mnt/c/Users/rosep/Downloads/Chirp$


# generate chirp.txt  
python3 chirp_gen.py
 
# examine
python3 ../AFSK/spectrogram_analysis.py chirp.txt  44100 6000 8000
dechirp/Screenshot 2026-08-29 222747.png

# dechirp artificial chirp
python3 dechirp.py chirp.txt
dechirp/Screenshot 2026-08-29 184148.png


# dechirp real rx chirp
python3 dechirp.py ../AFSK/rx_chirp.txt
dechirp/Screenshot 2026-08-29 184315.png


During the mix the chirpPos points a specific 4096 block of reference chirp. When next 4096 sample arrives, next block of reference chirp is chosen and mix. On reset, the initial value of chirpPos is 0 and it does not guarantee the reference to be aligned with incoming chirp. That is the reason the rough estimation of the start is needed. Do you agree? Answer in one paragraph.

9:49 PM
Yes — I agree. In your current implementation, each 4096-sample FFT block is mixed with a reference chirp phase indexed by chirpPos = absIdx % CHIRP_SAMPLES, so the reference is effectively sliding in lockstep with the sample index, not with the unknown start of the incoming chirp. That means the reference phase is only aligned if the arriving chirp happens to begin at the same offset as the assumed phase reference, and on reset it starts at zero, which does not guarantee alignment. This is exactly why a rough onset estimate or timing search is needed: without knowing where the incoming chirp begins relative to the local sample clock, the dechirp is not coherent across the full chirp duration, and the accumulated energy will be weak or noisy.



