Firmware Description

This document describes how the Audio_AI firmware works on the Apollo4 Blue Plus KXR Evaluation Board (EVB). It is meant for someone who has just cloned the repo and needs to understand the codebase well enough to modify it.

For setup / build / flash instructions, see the project README and the Quick Start Guide. This document focuses on the firmware itself - what runs where, how data flows, and which file does what.


1. What the firmware does

The board is a Bluetooth Low Energy (BLE) peripheral that records audio from an analog mic, runs a keyword-spotting (KWS) neural net on the live mic feed, and streams compressed audio to a paired phone over BLE.

End-to-end, on a single device boot:

1. Boot - clocks, Floating-Point Unit (FPU), low-power init, then FreeRTOS starts.
2. Listen for keywords - at boot the firmware sits in MODE_LISTENING, runs a TensorFlow Lite for Microcontrollers (TFLM) Depthwise Separable Convolutional Neural Network (DS-CNN) classifier over Mel-Frequency Cepstral Coefficient (MFCC) features extracted from the live mic, and waits for the trigger word.
3. Record - when the trigger word fires (or user presses BUTTON1), the firmware records audio at 16 kHz mono into shared Static Random-Access Memory (SRAM), then encodes it to Opus 32 kbit/s Constant Bit Rate (CBR).
4. Advertise & wait for phone - the BLE stack (Cordio) advertises as EatingAnalytics_AAI and exposes two custom Generic Attribute Profile (GATT) services: a JavaScript Object Notation (JSON) command/notify channel and an Opus byte-stream channel.
5. Stream Opus to phone - when the phone subscribes and writes stream_start, the firmware sends the encoded Opus buffer in a windowed, Acknowledgment (ACK)-driven flow protocol.
6. Play back locally - pressing BUTTON1 a second time pumps the recorded Pulse-Code Modulation (PCM) audio out the Inter-IC Sound (I2S) transmit (TX) line to a MAX98357A speaker amp.

The firmware deliberately keeps the audio path and the BLE path decoupled - capture, encode, and stream are independent stages that pass data through fixed shared-SRAM buffers.


2. Peripherals the firmware drives

At a high level, the firmware talks to:

- Audio Analog-to-Digital Converter (AUDADC) - analog mic input, sampled via Direct Memory Access (DMA).
- I2S0 TX - drives the external speaker amp.
- External BLE radio - controlled over Universal Asynchronous Receiver-Transmitter (UART) Host Controller Interface (HCI).
- Two General-Purpose Input/Output (GPIO) buttons - BUTTON0 triggers a BLE JSON notification; BUTTON1 drives the record/playback state machine.
- Serial Wire Output (SWO) - debug printf via J-Link.

For pinouts, Microcontroller Unit (MCU) revisions, board wiring, and electrical detail, see the dedicated hardware documentation.


3. Memory layout (where things live in Random Access Memory)

Two memory regions matter to this firmware:

- Tightly-Coupled Memory (TCM) - fast, on-core memory. Holds the FreeRTOS heap, task stacks, and ordinary .data/.bss.
- Shared SRAM - slower but much bigger. Holds DMA buffers, the recording/Opus buffers, and the TFLite tensor arena.

Anything that DMA touches or that won't fit in TCM is annotated with AM_SHARED_RW or NS_SRAM_BSS.

Big buffers, all in shared SRAM:

Buffer                       Size         Purpose
g_audadcBufRaw               2 KB         AUDADC ping-pong DMA (raw 32-bit DMA words)
g_txBufRawA / g_txBufRawB    2 KB each    I2S TX ping/pong for speaker playback
g_kwsBuf[2][320]             1.25 KB      KWS 20 ms ping-pong feed
g_mfccArena                  64 KB        neuralSPOT MFCC scratch arena
g_pcmBuf                     320 KB       10 s × 16 kHz × int16 mono recording
g_opusBuf                    40 KB        encoded Opus output (10 s × 50 frames/s × 80 B)
s_arena (TFLM)               48 KB        TFLM tensor arena

The build will fail at link time if any of these can't be placed - that is the first thing to check if the linker complains about overflowing a region.


4. Tasks & threading

FreeRTOS runs four cooperating tasks. Created in src/rtos.c by setup_task() after the scheduler starts:

Task          Priority   Stack       Purpose
SetupTask     3          512 W       Spawns everything else, then vTaskSuspend(NULL) (effectively dies)
RadioTask     3          1024 W      Cordio BLE stack - advertising, GATT, HCI, connection events
MicTask       2          16384 W     Audio state machine: listen → record → encode → playback
ButtonTask    2          256 W       10 ms-period BUTTON0 polling with debounce; calls HelloJsonOnButton0Pressed()

Why MicTask has a 64 KB stack: TFLM AllocateTensors() and the Opus encoder both burn deep stack during graph planning / encoding. ~10 KB is a safe headroom over what's actually used.

Communication between tasks is done with shared volatile flags in mic_task.c (e.g. g_recDone, g_opusLen, g_mode) and direct GATT writes (RadioTask → JSON service → MicTask via callbacks). There are no FreeRTOS queues between MicTask and RadioTask - the BLE side reads MicTask's encoded buffer directly via MicTaskGetOpusData().

Sleep policy: idle currently uses SLEEP_NORMAL (not deep sleep) so J-Link can halt the core for debugging - see the comment in am_freertos_sleep() in src/rtos.c line 107. Switch to SLEEP_DEEP for production.


5. File-by-file guide

This is the essential reading order for someone new to the project.

Top of the tree

src/ble_freertos_fit.c
Entry point. main() enables the FPU, calls am_bsp_low_power_init(), optionally enables Instrumentation Trace Macrocell (ITM)/SWO printing, generates the placeholder Opus payload (FakeOpusAudioInit(), used during BLE bring-up before real recordings exist), and calls run_tasks(). Should not need editing once running.

src/ble_freertos_fit.h
Aggregator header - pulls in Hardware Abstraction Layer (HAL), Board Support Package (BSP), util, and FreeRTOS headers used across the project. Almost every .c file includes this first.

src/rtos.c / src/rtos.h
FreeRTOS glue:
- setup_task() creates RadioTask, ButtonTask, MicTask, then suspends itself.
- ButtonTask() polls BUTTON0 every 10 ms with a 4-sample debounce; calls HelloJsonOnButton0Pressed() on press.
- am_freertos_sleep() / am_freertos_wakeup() - idle hooks.
- Stack-overflow and malloc-failure hooks print to ITM and spin.

This is where you adjust task priorities, stack sizes, or add new tasks.

src/FreeRTOSConfig.h
Standard FreeRTOS config. Notable: ticks at 1 kHz, heap is heap_4, stack overflow detection enabled.

BLE / radio

src/radio_task.c / src/radio_task.h
Owns the Cordio BLE stack:
- RadioTaskSetup() - sets COOPER Interrupt Request (IRQ) priorities. Called before scheduler starts.
- exactle_stack_init() - initializes every Cordio layer: Wireless Software Foundation (WSF) Operating System (OS)/timers, security (Advanced Encryption Standard (AES) / Cipher-based Message Authentication Code (CMAC) / Elliptic Curve Cryptography (ECC)), HCI, Device Manager (DM), Logical Link Control and Adaptation Protocol (L2CAP), Attribute Protocol (ATT), Security Manager Protocol (SMP), FIT app handler, HCI driver. Order matters; do not reshuffle.
- RadioTask() - boots the EM9304 radio (HciDrvRadioBoot(1)), calls FitStart() to start advertising, then loops on wsfOsDispatcher().
- WSF buffer pool sizing is non-obvious (see comments at the top of the file). Pool (400, 2) is sized for AttsCalculateDbHash() walking all 8 GATT services - if you add another service, that pool may need to grow.

The actual GATT service list and advertising payload lives outside this file in third_party/cordio/.../fit/fit_main.c and is patched in via patches/fit_main.patch. See section 7 below.

src/hci_apollo_config.h
Pin assignments for the BLE radio's HCI UART. Don't edit unless you change boards.

src/ble_json_svc.c
Custom JSON service - Universally Unique Identifier (UUID) 12341234-...90AB, value char ...90AC. The phone writes ASCII JSON, the device echoes status notifications.

Three roles:
1. Stream control router - recognizes stream_start, ack, nack keywords in incoming writes and forwards them to OpusStreamOnJsonCmd().
2. Ping/pong - {"cmd":"ping"} → {"reply":"pong"}.
3. Button bridge - HelloJsonOnButton0Pressed() (called from ButtonTask) sends {"device":"Apollo4","message":"hello"} to the phone.

GATT handles are hardcoded at 0x9000–0x9003. Notifications are gated on the Client Characteristic Configuration Descriptor (CCCD) being written by the phone - see HelloJsonCccState().

src/ble_opus_stream_svc.c / src/ble_opus_stream_svc.h
Custom Opus audio stream service - UUID ...90BB, value char ...90BC, notify-only. This is the bulk of the BLE complexity.

Implements the windowed stop-and-wait flow protocol documented in CLAUDE.md: packet header (flags / version / packet_index / total_packets / total_len), START / END / WINDOW_END flags, ACK / Negative Acknowledgment (NACK) handling, selective retransmit, ACK-timeout auto-resend.

Important entry points:
- OpusStreamSvcAdd() - registers the GATT group at boot.
- OpusStreamOnJsonCmd() - JSON service hands off stream_start/ack/nack here.
- OpusStreamSetMtu() - called when the phone negotiates Maximum Transmission Unit (MTU); the per-packet payload size is derived from this.
- OpusStreamTick() - called periodically by the WSF timer pump (set up in fit_main.c); drives inter-packet pacing and ACK timeout.
- OpusStreamSetPumpCallback() - Cordio side wires this to a WSF timer so the service can schedule its own ticks.

The recorded buffer is read via MicTaskGetOpusData() (defined in mic_task.h) - the Opus service does not own the audio buffer; it just iterates over it in 200-byte slices.

src/fake_opus_audio.c / src/fake_opus_audio.h
Generates a deterministic synthetic Opus-shaped byte stream at boot. Used for BLE-side bring-up / debugging when no real recording has been made yet - the JSON service can serve this as a stand-in. Safe to ignore in normal operation.

Audio path

src/mic_task.c - the heart of the firmware
By far the largest file (~930 lines). Owns:

1. AUDADC capture - ping-pong DMA at the native sample rate; Interrupt Service Routine (ISR) am_audadc0_isr runs once per DMA half-buffer.
2. DC blocker - single-pole Infinite Impulse Response (IIR) High-Pass Filter (HPF) that kills mic Direct Current (DC) bias before it pollutes downstream features.
3. Capture-side resampler - linear interpolation from the native rate to 16 kHz storage (Q16.16 fixed-point step counter).
4. State machine - MODE_LISTENING (KWS active), MODE_IDLE, MODE_RECORDING, MODE_PLAYING, MODE_TESTTONE.
5. KWS feed - when in LISTENING, fills 320-sample (20 ms) ping-pong buffers with software-gained int16 PCM for the inference loop.
6. Recording - when in RECORDING, writes int16 samples into g_pcmBuf until full.
7. Opus encoding - when recording finishes, runs Ambiq's libopus wrapper (audio_enc_init / audio_enc_encode_frame from ae_api.h) to produce 50 frames/s × 80 bytes = g_opusBuf.
8. Playback - int16 PCM is upsampled back to the native rate and packed into 24-bit I2S TX words with a soft limiter to prevent clipping. ISR am_dspi2s0_isr services TX DMA completions.
9. Test tone - long-press BUTTON1 generates a 1 kHz square wave for end-to-end audio-path verification.
10. Per-second debug stats - peak/average sample magnitude, ISR rate; gated by g_micDebug.

Key tunables at the top of the file: REC_SECONDS, MIC_RECORD_SOURCE, PREAMP_GAIN_DB, CHANNEL_GAIN_DB, MIC_BIAS_TRIM, PLAY_VOL_SHIFT, PLAY_LIMIT_START, KWS_PEAK_GATE, KWS_SW_GAIN.

The MicTask main loop is mostly a giant switch on g_mode. The interesting work happens inside the ISRs; the task body just orchestrates transitions and runs blocking work (encoding, KWS inference) outside the ISR.

src/mic_task.h
Public surface: MicTask() task entry point and MicTaskGetOpusData() accessor.

src/am_resources.c
HAL resource hooks. Should not need editing.

Keyword spotting

src/kws_inference.cc / src/kws_inference.h
C++ glue around TFLM:
- kws_init() - loads the flatbuffer model, registers the 6 ops the DS-CNN model uses (Conv2D, DepthwiseConv2D, FullyConnected, Reshape, Softmax, AveragePool2D), allocates tensors. Heavy verbose printing - useful for diagnosing arena-size or schema-version issues.
- kws_run() / kws_run_top2() - quantizes the float MFCC features into the int8 input tensor, runs Invoke(), dequantizes the int8 output back to scores, returns argmax.

The model is bundled as a C array in src/kws/kws_model_data.h.

Bare-metal C++ caveats addressed in this file:
- A no-op sized operator delete is provided because libsupc++ isn't linked.
- MicroInterpreter is constructed with placement-new into s_interp_buf to avoid __cxa_atexit registration that bare-metal linkers don't support.
- Tensor arena is in shared SRAM (NS_SRAM_BSS) to keep TCM free.

The "missing __libc_init_array" pitfall - if you ever see TFLM faulting on the first virtual dispatch, check that startup calls __libc_init_array so C++ static constructors run. (See the project memory note for full context.)

src/kws/kws_model_settings.h
Compile-time constants for the model (49 frames × 10 MFCC coeffs × 1 channel = 490 inputs, 12 output classes). Mirrors KWS_* defines in kws_inference.h; the static_assert in the .cc enforces consistency.

src/kws/kws_model_data.h
Big const uint8_t g_kws_model_data[] array - the actual TFLite flatbuffer. Replace this file (and update the size constants) to swap models.

Build / patches

gcc/Makefile
The build. Targets apollo4p, cortex-m4, fpv4-sp-d16, hard FPU. Pulls in 48+ Software Development Kit (SDK) include directories, the AmbiqSuite HAL/BSP libs, Cordio, FreeRTOS, libopus, libcatena (KWS), neuralSPOT, Cortex Microcontroller Software Interface Standard - Digital Signal Processing (CMSIS-DSP), TFLM. Output: gcc/bin/ble_freertos_fit.{axf,bin}.

NEURALSPOT_ROOT and TFLM_ROOT are absolute paths near the top of the Makefile - adjust these if your neuralSPOT clone lives elsewhere.

gcc/linker_script.ld / gcc/startup_gcc.c
Standard Apollo4P linker script and ARM startup. Defines TCM/SRAM regions, vector table, calls main(). The startup file is where C++ static-constructor invocation (__libc_init_array) needs to live - don't strip that call.

gcc/flash.jlink
J-Link CommanderScript that loads bin/ble_freertos_fit.axf, resets, and runs.

Makefile (project root)
Wraps gcc/Makefile and adds the vendor-patch targets (vendor-patch-check, vendor-patch-apply, vendor-patch-revert).

Patches

patches/fit_main.patch
Modifies third_party/cordio/ble-profiles/sources/apps/fit/fit_main.c (lives outside the repo) to:
- Register the JSON and Opus services (HelloJsonSvcAdd, OpusStreamSvcAdd).
- Add CCCD entries for both services.
- Negotiate ATT MTU 247 on connect.
- Set the advertised local name to EatingAnalytics_AAI.
- Wire up the WSF timer pump that drives OpusStreamTick().

Apply with make vendor-patch-apply. Without this patch, the BLE stack will run but neither custom service will be reachable.

patches/hci_vs_cooper.patch
Smaller patch to the EM9304 HCI driver (vendor-specific commands).

scripts/apply_vendor_patches.sh
Idempotent patch applier - handles "already applied" gracefully.


6. End-to-end data flow

Analog mic → AUDADC → DMA → am_audadc0_isr ────┐
                                               │
                                  DC-block + resample (23.4k→16k)
                                               │
                   ┌─────────────────────────   ────────────────────┐
                   │                           │                          │
              MODE_LISTENING            MODE_RECORDING             MODE_PLAYING (reads g_pcmBuf)
                   │                           │                          │
       fill g_kwsBuf[320]              write to g_pcmBuf            resample 16k→23.4k → soft limit
                   │                           │                          │
       MicTask: MFCC + kws_run()       (10 s, 160k samples)          pack i24 into g_txBufRaw*
                   │                           │                          │
       above threshold? log it         encode g_pcmBuf → g_opusBuf   I2S TX DMA → MAX98357A
                                               │
                                       MicTaskGetOpusData()
                                               │
                                       OpusStreamSvc walks bytes,
                                       sends notifications per
                                       windowed protocol → phone

Two control paths cross-cut:

- Phone → device control: BLE write to JSON char → helloWriteCback → OpusStreamOnJsonCmd → state update in opus stream svc.
- Buttons → state changes: ButtonTask polls BUTTON0 and triggers a JSON notification; the BUTTON1 path is polled inside MicTask itself (it owns the audio state machine).


7. Where customizations live

If you need to add a new BLE service:
1. Write the service file (model after src/ble_json_svc.c).
2. Add a *SvcAdd() call in the SDK's fit_main.c - update patches/fit_main.patch.
3. Add a CCCD entry to fitCccSet[] in fit_main.c (also via the patch).
4. Bump WSF buffer pool 6 size in src/radio_task.c if the GATT DB hash buffer overflows.
5. Re-run make vendor-patch-apply and make -C gcc.

If you need to change the audio config (sample rate, recording length, gain): edit the #defines at the top of src/mic_task.c. Be careful - REC_PCM_RATE is fixed at 16 kHz by the Opus encoder library and cannot change without rebuilding libopus with different config. REC_SECONDS is the easiest knob.

If you need to swap the KWS model: replace src/kws/kws_model_data.h, update kws_label_names[] and KWS_TRIGGER_IDX in src/kws_inference.cc, and bump KWS_NUM_FRAMES / KWS_NUM_COEFFS / KWS_NUM_CLASSES if the input/output shape changed. The op list registered via s_resolver.AddXxx() must match the ops your model uses - TFLM will fail at AllocateTensors() if any op is missing.


8. Debugging tips

- Build output sanity check: arm-none-eabi-size gcc/bin/ble_freertos_fit.axf - TCM should have headroom; shared SRAM is what fills up.
- Live logs: JLinkSWOViewer -device AMA4B2KP-KBR -if SWD -speed 4000 -cpufreq 96000000 shows everything from am_util_debug_printf. Build with AM_DEBUG_PRINTF defined to enable.
- BLE on the phone side: nRF Connect (iOS/Android) is the easiest way to discover services and inspect notifications. Filter on advertising name EatingAnalytics_AAI.
- MicTask not advancing past KWS init: usually means TFLM AllocateTensors() failed - check the op list and arena size, and verify __libc_init_array is being called.
- No notifications received: phone must write 0x0001 to the CCCD descriptor for each characteristic before notifications fire. nRF Connect does this automatically when you tap the "subscribe" icon.
- Audio sounds clipped: drop PLAY_VOL_SHIFT or PREAMP_GAIN_DB. The soft limiter helps but won't undo a genuine overload at the analog input.


9. Quick reference - where to look first

You want to...                              Start here
Add / change a BLE service                  src/ble_json_svc.c + patches/fit_main.patch
Tweak record length / mic gain              top of src/mic_task.c
Adjust the streaming flow protocol          src/ble_opus_stream_svc.c
Swap or retrain the KWS model               src/kws/kws_model_data.h, src/kws_inference.cc
Add a FreeRTOS task                         src/rtos.c setup_task()
Change peripheral pin / clock config        src/hci_apollo_config.h, top of src/mic_task.c
Update SDK include paths / link libs        gcc/Makefile
Change advertising name                     patches/fit_main.patch (fitScanDataDisc[])
