This folder included the updated files that need to be placed in other areas my PC.

- LATEST_fit_main.c goes in C:\AmbiqSuite-R4-5-0\AmbiqSuite_R4.5.0\AmbiqSuite_R4.5.0\third_party\cordio\ble-profiles\sources\apps\fit (remove LATEST_ from name)
- The changes for one change are described in fitUpdateCfg-fit_main
- It also includes the patched code from the students

---------------------------------------------------------------------

I rebuilt the opus conversion code in order to make it faster. This code was in NeuralSpot. You can see the original and new versions of them in this folder.
Key changes are:
neuralSPOT lives at /home/hyer/neuralSPOT, is Ambiq's repo, can't be pushed to
Build it with make EXEEXT= — the default .exe invokes the Windows toolchain from WSL and fails
Which files were modified and why: FIXED_POINT + VAR_ARRAYS on, USE_ALLOCA off; silk/float → silk/fixed; opus1.4 added to modules; opus-precomp and octopus removed (header collision); src/src/*.c trimmed to skip multistream/projection
nnse demo fails to build — expected and harmless

The original files are .orig. The new files have no suffix.

make EXEEXT= clean
make EXEEXT= 2>&1 | tee /tmp/nsbuild.log
grep -n "error:\|Error " /tmp/nsbuild.log | head -20

In flutter we made these changes to make things work with the firmware changes:
lib/device_info_page.dart
	_blePayloadSize: 200 → 234
	static const int _blePayloadSize = 234;

	frameSize in _wrapInOggOpus: 80 → 60
	const frameSize = 60;   // bytes per CBR frame at 24 kbit/s, 20 ms
	
_blePayloadSize and the firmware's OPUS_MAX_AUDIO_PAYLOAD must always match, since the app computes offset = packetIndex * _blePayloadSize to place bytes. A mismatch corrupts every packet's position silently — no error, just garbled audio.

Same coupling for frameSize against the firmware's OPUS_FRAME_BYTES. You already hit this one.

Note the two known fragilities

Worth a comment in the code or the README: frameSize is duplicated across two repos with no link between them (bitrate changes silently corrupt playback), and the Flutter TOC byte 0x68 is SILK WB while you're now CELT.

Here is how things changed with the opus improvements.
Ambiq blob: 14,668 µs/frame
opus1.4 fixed-point CELT complexity 4: 14,503 µs/frame
opus1.4 complexity 0 @ 32 kbps: 11,699 µs/frame
opus1.4 complexity 0 @ 24 kbps: 11,175 µs/frame, 90 KB files
Transfer: 5.4 s (was 7.25 s at 120 KB, was 14.47 s at session start)
BLE settings: OPUS_INTER_PKT_DELAY_MS 15, payload 234 B, window 1000

Also note the negative results, so you don't retry them: connection priority high made things worse; SILK mode was 4× slower than CELT; complexity 1–3 gave no benefit over 4.

------------------------------
from 8-7-2026 work
Chewallow 79×24 float32: 5,944 ms/inference, arena 245,280 bytes
.shared is AT>MCU_MRAM — anything declared AM_SHARED_RW is stored byte-for-byte in flash. Use MIC_NOLOAD (.sram_bss) for scratch buffers. Cost 1.12 MB of MRAM until fixed.
.sram_bss is not zero-filled at startup in the current linker script, and nothing here needed it — TFLM arenas and DMA buffers are all written before read.
CMSIS-NN ships inside libtensorflow-microlite-cm4-gcc-release.a; already linked, nothing to configure.
KWS: ~366 ms inference, ~75 ms MFCC (from March)