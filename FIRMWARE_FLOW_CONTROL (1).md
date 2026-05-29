# BLE Audio Stream — Firmware Contract for App-Side Implementation

> **Audience:** This document is the authoritative wire-protocol and behavior spec for the firmware that lives in `src/ble_opus_stream_svc.c` and `src/ble_json_svc.c` of the `Audio_AI_4.5.0` project. The phone/app implementation must match it exactly.
>
> **If the audio you are receiving is noisy, truncated, has gaps, or never arrives at all, the most likely cause is that the app code is built against an older version of this protocol.** Read this whole file before debugging.

---

## What changed recently (BREAKING for older app code)

The transfer is now **device-initiated**. The firmware automatically starts streaming the moment a recording finishes encoding. **The phone must no longer write `{"cmd":"stream_start"}` to begin a transfer under normal operation.**

| Old behavior (deprecated) | New behavior (current) |
|---|---|
| Phone sends `stream_start` to begin every transfer | Device auto-sends as soon as encode completes |
| If no `stream_start`, nothing happens | If phone is not connected, recording is **held** in device RAM and flushed automatically when the phone re-connects and re-enables CCCD |
| `stream_start` is the normal control path | `stream_start` exists only as an optional **manual replay** command (re-send the last recording) |

Implementation in firmware: `OpusStreamStartAuto()` in [src/ble_opus_stream_svc.c](src/ble_opus_stream_svc.c) is invoked from MicTask the instant `g_opusLen = opusBytes` is set in [src/mic_task.c](src/mic_task.c).

**If the app is still writing `stream_start` blindly on connect, it will work** (it triggers the same `opusKickOffStream()` path), but you will get **two streams trying to start in quick succession** when the encode-complete moment and the app's `stream_start` write race each other. Symptoms: duplicate START flags, corrupted in-flight state, dropped packets. **Remove the `stream_start` write from the normal connection-establishment flow.**

---

## Required app-side state machine (correct behavior)

```
   ┌──────────────┐
   │  Disconnected│
   └──────┬───────┘
          │ scan + connect
          ▼
   ┌──────────────┐
   │   Connected  │
   └──────┬───────┘
          │ MTU exchange (request 247)
          ▼
   ┌──────────────────────┐
   │  MTU negotiated      │
   └──────┬───────────────┘
          │ enable CCCD on JSON char (write 0x0001 to its CCCD)
          ▼
   ┌──────────────────────┐
   │ JSON notifications on│
   └──────┬───────────────┘
          │ enable CCCD on Opus char (write 0x0001 to its CCCD)
          ▼
   ┌──────────────────────────────────────────────┐
   │  Listening — DO NOT write stream_start       │
   │                                              │
   │  When the user records on the device (button │
   │  or wake-word), a stream begins on its own.  │
   │  The first packet arrives with flags & 0x01  │
   │  (START) set.                                │
   └──────┬───────────────────────────────────────┘
          │ wait for first Opus notification with START flag
          ▼
   ┌──────────────────────┐
   │  Receiving stream    │
   │                      │
   │  • Buffer payloads   │
   │  • Track received    │
   │    indices per window│
   │  • On WINDOW_END:    │
   │     ACK or NACK      │
   │  • On END: send final│
   │    ACK and reset     │
   └──────┬───────────────┘
          │ END flag + final ACK sent
          ▼
   ┌──────────────────────┐
   │ Back to Listening    │
   │                      │
   │ KEEP THE CONNECTION  │
   │ AND CCCDs ENABLED.   │
   │ The next recording   │
   │ will auto-stream     │
   │ with no further app  │
   │ action.              │
   └──────────────────────┘
```

---

## GATT services and characteristics

All 128-bit UUIDs shown big-endian display form. Reverse byte order if your stack uses little-endian byte arrays.

| Role | UUID |
|---|---|
| JSON service | `12341234-5678-1234-1234-1234567890AB` |
| JSON characteristic (read / write / notify) | `12341234-5678-1234-1234-1234567890AC` |
| Opus audio service | `12341234-5678-1234-1234-1234567890BB` |
| Opus audio characteristic (read / notify) | `12341234-5678-1234-1234-1234567890BC` |

Both characteristics expose a standard CCCD (UUID 0x2902). Write `0x0001` to enable notifications. The device uses **notifications**, never indications.

Advertised local name: **`EatingAnalytics_AAI`** — filter on this if you want to be tolerant to UUID changes.

---

## MTU and payload sizing

The firmware computes per-packet payload as:

```
payload_size = min(ATT_MTU - 3 - 10, 200)
                              │   │   └─ Opus metadata header (OPUS_META_BYTES)
                              │   └─── ATT notification header (OPUS_ATT_HDR_BYTES)
                              └─────── 200 byte cap (OPUS_MAX_AUDIO_PAYLOAD)
```

| MTU | payload_size | Notes |
|---|---|---|
| 23 (default if MTU never negotiated) | 10 | Fallback only. Use 247. |
| 247 | 200 | Normal operation. |

**App must request MTU 247 (or higher) before enabling CCCDs.** The device calls `OpusStreamSetMtu(23)` at connection open and updates on `ATT_MTU_UPDATE_IND`. If the app enables CCCDs without negotiating MTU, the device will recompute payload at 10 bytes per packet — `total_packets` will be ~12 000 instead of 600, and you may exhaust the 16-bit packet index. Always negotiate MTU first.

The device **fixes** `payload_size` for a transfer at the moment of stream-start. Mid-stream MTU changes are ignored until the next stream begins (see `opusRecomputeStream` gating in source).

---

## Opus notification packet format (device → phone)

Every notification on the Opus characteristic has this layout. All multi-byte fields are **little-endian**.

| Byte | Field | Type | Meaning |
|---|---|---|---|
| 0 | `flags` | u8 | bitmask, see below |
| 1 | `metadata_version` | u8 | always `0x01` for this firmware |
| 2–3 | `packet_index` | u16 | 0-based index of this packet within the stream |
| 4–5 | `total_packets` | u16 | total notifications the app should expect |
| 6–9 | `total_len` | u32 | total length of the raw Opus byte stream in bytes |
| 10..end | `payload` | bytes | up to 200 bytes of raw Opus, length = ATT len − 10 |

### Flag bits

| Bit | Constant | Value | Meaning |
|---|---|---|---|
| 0 | `OPUS_FLAG_START` | `0x01` | First packet of the entire stream (`packet_index == 0`). |
| 1 | `OPUS_FLAG_END` | `0x02` | Last packet of the entire stream (`packet_index == total_packets - 1`). |
| 2 | `OPUS_FLAG_WINDOW_END` | `0x04` | Last packet of the current window. **App must respond with ACK or NACK.** |

Bits OR together. Examples you will see:
- `0x01` — first packet of stream, not last of window (e.g. window size > 1).
- `0x04` — last packet of a window in the middle of the stream.
- `0x05` — first packet of the stream **and** also the last of window 0 (only happens if window=1).
- `0x06` — last packet of the stream **and** last of the final window (most common at end).
- `0x07` — entire stream fits in a single 1-packet window (unusual).

The app must check **each bit independently**; do not switch on the byte value.

---

## Typical numbers for a 30-second recording

These are the values the app will see for a normal recording at MTU 247:

| Quantity | Value | Where it comes from |
|---|---|---|
| Recording duration | 30 s | `REC_SECONDS` in [mic_task.c](src/mic_task.c) |
| PCM rate after resample | 16 kHz mono int16 | Opus encoder input |
| Opus bitrate | 32 kbit/s CBR | hardcoded in `audio_enc_*` wrapper |
| Frame duration | 20 ms | encoder config |
| Bytes per Opus frame | **80** | `32000 × 0.020 / 8` |
| Total frames | 1500 | 30 s ÷ 20 ms |
| `total_len` | **120000** bytes | 1500 × 80 |
| `payload_size` | 200 bytes | min(247−3−10, 200) |
| `total_packets` | **600** | ceil(120000 / 200) |
| Default window | 20 packets | `OPUS_DEFAULT_WINDOW` |
| Windows per transfer | 30 | 600 / 20 |

If `total_packets` shows up as something wildly different (e.g. 12000), the MTU was not negotiated up. See [MTU and payload sizing](#mtu-and-payload-sizing).

---

## Timing constants the app must respect

These are hard-coded in [src/ble_opus_stream_svc.c](src/ble_opus_stream_svc.c) and the app must align with them:

| Constant | Value | Meaning |
|---|---|---|
| `OPUS_INTER_PKT_DELAY_MS` | **50 ms** | Spacing between consecutive notifications inside a window. The device schedules the next packet via a WSF timer 50 ms after the current one. This is the wall-clock pacing of incoming packets you'll see on the app side. |
| `OPUS_ACK_TIMEOUT_MS` | **5000 ms** | After sending the packet with `WINDOW_END`, the device waits up to 5 s for an `ack` or `nack`. If neither arrives, it **resends the entire window** automatically (treated as implicit NACK). |
| `OPUS_MAX_MISSING` | **64** | Maximum number of packet indices the app can list in a single `nack`. Going over silently truncates. |
| `OPUS_DEFAULT_WINDOW` | **20** | Used both by auto-send and as the default if `stream_start` omits `window`. |

**Time budget for a 30 s recording:** 600 packets × 50 ms = 30 s of pacing, plus 30 windows × ~one ACK round-trip ≈ ~31–33 s total transfer over a clean link. Plan UI/progress accordingly.

---

## JSON commands the phone writes to the JSON characteristic

All writes must be **valid JSON objects** (start with `{`, end with `}`). The device rejects malformed input with `{"device":"Apollo4","status":"error","reason":"invalid_json"}` notified back on the JSON characteristic.

### `ack` — acknowledge a window

```json
{"cmd":"ack","next":20}
```

- Send after receiving a packet with `WINDOW_END` set, when **all packets in the window were received**.
- `next` = `last_received_index_in_this_window + 1`. The device uses this as the start of the next window.
- If `next >= total_packets`, the device treats this as **transfer complete**: stops the pump timer, clears its pending-auto-start flag, logs `OpusStream: transfer complete`. Send this final ACK before considering the file done.

**Device reply** (on JSON characteristic, as a notification): `{"device":"Apollo4","status":"ok","received_len":N}` where `N` is the byte length of your JSON write.

### `nack` — request selective retransmit

```json
{"cmd":"nack","window_start":20,"missing":[22,27]}
```

- Send after receiving `WINDOW_END` when **gaps were detected** in this window.
- `window_start` is the index of the first packet in the window (informational; the device uses `missing[]` exclusively to drive the resend).
- `missing` is the array of packet indices the app needs resent. Max 64 indices. Must be sorted? No — the device walks the list in order; it sets `OPUS_FLAG_WINDOW_END` on the **last index in the array**, regardless of numeric ordering. Whatever order you send is the order they arrive in.
- Device resends only those packets. After the last resent packet (which has `WINDOW_END`), the device enters `WAITING_ACK` again — the app should examine the resent set and send another `ack` or `nack`.
- App should retry up to **3 times** before giving up on a window and sending `ack` to skip it.

**Device reply:** same `{"status":"ok","received_len":N}` notification.

### `stream_start` — optional manual replay

```json
{"cmd":"stream_start","window":20}
```

- **Do not send this on connect under normal operation.** The device auto-sends.
- Use this to **re-transmit the last recording from packet 0**. Useful for "redownload" UI buttons. Overrides any in-flight stream (resets to STREAM_IDLE first, then kicks off).
- `window` is optional; defaults to 20.
- Clears the device's pending-auto-start flag.
- If no recording has ever been encoded since boot, device replies `{"device":"Apollo4","status":"error","reason":"stream_not_ready"}` and **no notification follows**.

### `ping` — diagnostic

```json
{"cmd":"ping"}
```

Replies with `{"device":"Apollo4","status":"ok","reply":"pong"}` on the JSON characteristic. Useful for verifying the JSON channel is round-trip healthy independent of audio.

### Anything else

Any other valid JSON object will be ack'd with `{"device":"Apollo4","status":"ok","received_len":N}`. This is intentional — it lets you write arbitrary metadata to the device without an error, but the device does not act on it.

---

## Device-initiated notifications on the JSON characteristic (device → phone)

The JSON characteristic also produces **unsolicited** notifications the app must be ready to receive:

| When | Payload | Meaning |
|---|---|---|
| Reply to any JSON write | varies (see above) | Command processed. |
| BUTTON0 (SW1) pressed on device | `{"device":"Apollo4","message":"hello"}` | Diagnostic — used for end-to-end notification check. App can ignore or surface in UI. |

The app must handle reply notifications **at any time**, not just synchronously after its own write. Specifically, during an audio transfer, the app will receive an interleaved stream of:
- Opus notifications on the Opus characteristic (the audio data)
- JSON notifications on the JSON characteristic (replies to the app's ACK/NACK writes)

Do not block reading one channel waiting for the other.

---

## Pending-recording semantics (the "phone disconnects mid-recording" case)

This is the single most important new behavior to get right in the app.

The firmware maintains a `g_pendingAutoStart` flag (bool) with the following lifecycle:

| Event | What happens to `g_pendingAutoStart` |
|---|---|
| Recording finishes encoding, phone not connected | Set TRUE |
| Recording finishes encoding, phone connected + CCC on | Stream begins immediately, flag stays FALSE |
| Phone enables Opus CCCD while flag is TRUE | Stream begins immediately, flag cleared to FALSE |
| App sends `ack` with `next >= total_packets` | Flag cleared to FALSE |
| App sends `stream_start` | Flag cleared to FALSE (manual override) |
| Phone disconnects | Flag is **not** cleared. In-flight stream state resets to IDLE. |

**Implication for the app:**

1. The very first thing the app should do after MTU exchange is enable the JSON CCCD, then the Opus CCCD. **The Opus CCCD write is the trigger that releases a held recording.** If the user produced a recording before opening the app, enabling CCCD is what unlocks the file transfer.

2. If the connection drops mid-stream, simply reconnect, re-negotiate MTU, re-enable both CCCDs, and the device will **restart the stream from packet 0**. You do not need to "resume" — there is no resume mechanism. Clear any partial buffer in the app on reconnect.

3. If the user records a **second** file while a transfer is in flight, the device clobbers `g_opusBuf` with the new encoding. The previous transfer's mid-flight packets become invalid. The firmware mitigates this by resetting stream state to IDLE inside `OpusStreamStartAuto()`, but a window already in-flight on the BLE link may land at the app with valid headers but stale-buffer payload. **App heuristic:** if you see a `START`-flagged packet arrive in the middle of an already-in-progress transfer, drop the old buffer and start a fresh receive against the new `total_packets`/`total_len`.

---

## Stream state machine (device side, for debug visibility)

Internal states the firmware moves through, visible in UART logs:

| State | Entry log | Meaning |
|---|---|---|
| `STREAM_IDLE` | `OpusStream: notify enabled` / `OpusStream: transfer complete` | No active transfer. |
| `STREAM_SENDING_WINDOW` | `OpusStream: start (window=20, packets=600, bytes=120000)` | Sending packets one every 50 ms. |
| `STREAM_WAITING_ACK` | `OpusStream: window sent (pkts 0-19 / 600)` | Last packet of window has been sent. Waiting on app reply. |
| `STREAM_RESENDING` | `OpusStream: nack, resending N packets` | Iterating the missing[] list. |

Other useful log lines:

| Log line | What it tells you |
|---|---|
| `OpusStream: auto-send (recording ready, phone connected)` | Auto-send fired immediately. Happy path. |
| `OpusStream: auto-send pending (waiting for phone)` | Recording held; flag set; waiting for CCCD enable. |
| `OpusStream: flushing pending recording` | App just enabled CCCD; held recording is being released. |
| `OpusStream: ACK timeout, resending window at pkt N` | App did not ACK within 5 s. Device resends entire window. **Frequent appearance = app's ACK writes are not getting through, or app is parsing WINDOW_END wrong.** |
| `OpusStream: stream_start but no recording` | App sent `stream_start` before any recording existed. Replies `stream_not_ready`. |
| `OpusStream: transfer complete` | Final ACK with `next >= total_packets` was received. |

---

## Common app-side bugs that look like "audio is broken"

If audio is noisy, clipped, or never arrives, check these **in order**:

### 1. App is hardcoding `payload_size = 200` instead of computing from MTU

If the app slices the received buffer into 200-byte chunks at decode time but the device negotiated a smaller MTU (e.g. fell back to MTU 23), the slices misalign and Opus decode produces garbage. **Fix:** read the actual notification length and use `att_len - 10` as the per-packet payload size; pull `total_len` from the header for the decode-time buffer split, and divide by `total_packets` if you need a uniform stride.

### 2. App is decoding the Opus byte stream with the wrong frame size

The device sends a **raw concatenation of CBR Opus frames** — no Ogg container, no TOC, no length prefix per frame. Each Opus frame is **exactly 80 bytes** (32 kbit/s × 20 ms ÷ 8). The app must:
- Create an Opus decoder at 16 kHz mono.
- Walk `total_len` bytes in 80-byte slices.
- Feed each 80-byte slice as one decoder packet.
- Each call produces exactly 320 PCM samples (int16 mono).

If you feed the decoder anything other than 80-byte slices (e.g. one-packet-per-notification = 200 bytes), it will decode garbage.

### 3. App is sending `ack` with the wrong `next` value

`next` must be `last_received_index_in_this_window + 1`. Common mistake: using `window_start + window_size` after the device sent a partial final window. The actual final window is shorter than 20 packets (`total_packets % 20`), so `last_index + 1 = total_packets`, not `previous_start + 20`.

### 4. App is still sending `stream_start` on connect

Causes a race with the device's auto-send. See [What changed recently](#what-changed-recently-breaking-for-older-app-code). Remove the write.

### 5. App enables Opus CCCD before JSON CCCD

The device will attempt to stream immediately on Opus CCCD enable if there's a pending recording. But the app cannot reply with `ack`/`nack` until JSON notifications are also enabled (otherwise the reply notifications on the JSON characteristic are silently dropped by the central). **Enable JSON CCCD first, Opus CCCD second.**

### 6. App is not negotiating MTU before enabling CCCDs

See [MTU and payload sizing](#mtu-and-payload-sizing). `total_packets` will balloon and the device's `OPUS_MAX_MISSING=64` cap will make recovery impossible.

### 7. App writes are not reaching the device

The JSON channel uses **write with response** (the device's characteristic permits both, but apps default to write-with-response). If the central uses write-without-response and the link layer drops it, the device sees nothing and the ACK timeout fires. Verify on a sniffer that ATT writes are arriving at the device.

---

## Quick reference: byte-level sanity checks for the app

Print these for every received Opus notification when debugging:

```
[Opus rx] len=210 flags=0x01 idx=0/600 totLen=120000 payload[0..3]=fc 30 c1 d2
[Opus rx] len=210 flags=0x00 idx=1/600 totLen=120000 payload[0..3]=...
...
[Opus rx] len=210 flags=0x04 idx=19/600 totLen=120000 payload[0..3]=...
  → app sends {"cmd":"ack","next":20}
[Opus rx] len=210 flags=0x00 idx=20/600 totLen=120000 payload[0..3]=...
...
[Opus rx] len=210 flags=0x06 idx=599/600 totLen=120000 payload[0..3]=...
  → app sends {"cmd":"ack","next":600}
[JSON rx] {"device":"Apollo4","status":"ok","received_len":24}
```

Expected invariants for every packet on a clean transfer:
- `len` is constant at `payload_size + 10` (210 at MTU 247) for all packets **except** possibly the very last, which is short by `(total_len % payload_size)` if non-zero.
- `idx` is strictly monotonically increasing by 1.
- `totLen` and `total_packets` are constant across the entire stream.
- `metadata_version` (byte 1) is always 0x01.

If any of these invariants break mid-stream, log the raw bytes and the prior state — that's the signal that a second recording started or the device reset.

---

## File / symbol map for cross-referencing

| Symbol | File |
|---|---|
| `OpusStreamStartAuto` | [src/ble_opus_stream_svc.c](src/ble_opus_stream_svc.c) — triggered from MicTask on encode complete |
| `OpusStreamOnJsonCmd` | [src/ble_opus_stream_svc.c](src/ble_opus_stream_svc.c) — dispatch for `ack` / `nack` / `stream_start` |
| `OpusStreamCccState` | [src/ble_opus_stream_svc.c](src/ble_opus_stream_svc.c) — auto-flush on CCCD enable |
| `opusKickOffStream` | [src/ble_opus_stream_svc.c](src/ble_opus_stream_svc.c) — shared stream-start helper |
| `OpusStreamTick` | [src/ble_opus_stream_svc.c](src/ble_opus_stream_svc.c) — pump driven by WSF timer; 50 ms cadence inside a window |
| `helloWriteCback` | [src/ble_json_svc.c](src/ble_json_svc.c) — receives all JSON writes, forwards stream commands |
| `helloSetJsonAckLen` / `helloSetJsonError` / `helloSetJsonAckPong` | [src/ble_json_svc.c](src/ble_json_svc.c) — reply formatters |
| `g_opusBuf` / `MicTaskGetOpusData` | [src/mic_task.c](src/mic_task.c) — the encoded byte buffer the stream service reads from |
| `OpusStreamStartAuto()` call site | [src/mic_task.c](src/mic_task.c) immediately after `g_opusLen = opusBytes` |

---

## TL;DR for the app-side Claude

1. **Stop writing `stream_start` on connect.** The device auto-sends.
2. After connect: negotiate MTU 247 → enable JSON CCCD → enable Opus CCCD → **wait** for a notification with `flags & 0x01` (START).
3. Buffer payloads by `packet_index`. On every notification with `flags & 0x04` (WINDOW_END), send `{"cmd":"ack","next":<last_idx+1>}` or `{"cmd":"nack","window_start":<ws>,"missing":[...]}`.
4. On `flags & 0x02` (END), send the final `ack` with `next == total_packets`.
5. Decode the assembled byte stream in **80-byte Opus frames** at 16 kHz mono.
6. Stay connected. The next recording will auto-arrive the same way.
7. If the user wants to re-download the most recent file, send `{"cmd":"stream_start","window":20}` — that's the manual replay path.
