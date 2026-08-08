---
title: Protocol reference
weight: 30
---

# M0601 protocol & hardware reference

Everything this repo knows about the DFRobot **M0601** direct-drive hub motor,
compiled from the official documentation and three independent implementations,
cross-checked byte-for-byte against this crate's test vectors. Where sources
disagree, the disagreement is recorded (see [Known
contradictions](#known-contradictions)) rather than silently resolved.

This is the byte-level reference. For what these frames *mean* and why the driver
handles them the way it does, see [Concepts]({{< relref "concepts" >}}) — especially
[The bus]({{< relref "concepts/the-bus" >}}) and [Telemetry and echo]({{< relref
"concepts/telemetry-and-echo" >}}).

## Identity

The "DFRobot M0601" is a rebadged **Direct Drive Tech (DDT) M0601C-111**. DFRobot
sells it as two SKUs:

| SKU | Side | DFRobot product |
|---|---|---|
| **FIT1042** | Left  | product-3077 |
| **FIT1038** | Right | product-3076 |

The two are mechanically and electrically identical and speak the same protocol;
only the (directional) tire tread differs. Related DDT models — **M0601C-112**,
M0602C, M1502A, DDSM115 — are *not* protocol-guaranteed identical.

## Electrical & mechanical specs

| Parameter | Value |
|---|---|
| Operating voltage | 18 V DC |
| Rated / stall current | 1.25 A / ≤ 2.7 A |
| No-load / rated / no-load speed | ≤ 0.25 A · 115 rpm · 200 ± 10 rpm |
| Rated / stall torque | 0.96 N·m / 2.0 N·m |
| Torque / speed constant | 0.75 N·m/A / 11.1 rpm/V |
| Encoder resolution | 4096 (relative accuracy 1024) |
| Protection / temp range | IP54 / −20…45 °C |
| Wheel diameter | 102 mm |
| Drive | direct — the wheel is the rotor; no gearbox, no backlash |

Note the stall current (≤ 2.7 A) against the current-loop command range (±32767 ≈
±8 A): the top of the commandable range is far beyond what the motor can draw, and
the 3 A bus-overcurrent protection trips long before it.

## Wiring

Signal cable (4-pin JST):

| Wire | Signal | Notes |
|---|---|---|
| Black | GND | signal ground reference |
| White | RS485 **A (+)** | |
| Orange | RS485 **B (−)** | |
| Brown | RESV | reserved/shield — **must be tied to GND** |

Power cable (2-pin): red = 18 V DC, black = GND.

Gotchas, in the order they actually bite:

1. **A/B polarity**: the motor's A/B labelling is inverted relative to many
   USB-RS485 adapters. No response → swap orange ↔ white.
2. A floating brown wire causes intermittent comms errors.
3. Add a 120 Ω termination resistor across A/B for cable runs over ~1 m.
4. A powered-down motor is silent on the bus, not absent-with-errors.
5. Keep exactly one motor on the bus when assigning IDs.

## Link layer

- RS485, half-duplex, multi-drop; **115200 baud, 8N1**.
- Every frame in both directions is **exactly 10 bytes**.
- Motor addresses: `0x01..=0xFE`. `0x00`/`0xFF` are reserved; `0xC8` is the
  broadcast destination of the ID query, so avoid assigning it.
- Many USB adapters echo their own transmission: RX may open with an exact copy
  of the TX frame. The driver strips it (a genuine reply never byte-equals the TX
  frame — its byte 1 is a mode value, not the command).

**Polling model:** a drive command does not latch. Officially documented max is
**500 Hz**; community/empirical floor is **~50 Hz (≤ every 20 ms)** or the motor
coasts. Power-up defaults: velocity mode, ID as last assigned (stored in flash).

## CRC

Standard host frames carry a checksum over bytes 0–8 in byte 9: **CRC-8/MAXIM**
(Dallas/1-Wire): polynomial x⁸+x⁵+x⁴+1, reflected constant `0x8C`, init `0x00`, no
final XOR. Check value: `crc8("123456789") = 0xA1`.

```text
crc = 0
for byte in data:
    crc ^= byte
    repeat 8: crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
```

**Two host frames carry no CRC:** the mode-switch frame (`0xA0`) puts the mode
value in byte 9, and the set-ID frame sets byte 9 to `0x00`. Replies carry the
same CRC (verified on hardware) but drivers should treat it as advisory, not
grounds for rejection.

## Host → motor frames

### Drive (`0x64`)

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| | ID | `0x64` | value HI | value LO | 0 | 0 | accel | brake | 0 | CRC |

- The 16-bit big-endian value in bytes 2–3 is interpreted **per the active
  mode**. Zero is *not* universally "stop": 0 rpm in velocity, "drive to 0°" in
  position, "zero torque" (coast) in current.
- **Acceleration** (byte 6): 0–255. `0` = motor default; `1` = *fastest* ramp;
  larger = gentler.
- **Brake** (byte 7): `0xFF` engages the electric brake (velocity mode only);
  otherwise `0x00`.

Worked examples (ID `0x01`, accel 0 — from `m0601/tests/vectors.rs`):

```text
+100 rpm      01 64 00 64 00 00 00 00 00 4F
-150 rpm      01 64 FF 6A 00 00 00 00 00 5A
brake         01 64 00 00 00 00 00 FF 00 D1
position 8192 01 64 20 00 00 00 00 00 00 BF   (≈ 90°)
```

### Feedback query (`0x74`)

`ID 74 00 00 00 00 00 00 00 CRC` — for ID `0x01`: `01 74 00…00 04`. The addressed
motor answers with the **query-layout** telemetry frame (the only reply carrying
winding temperature).

### Mode switch (`0xA0`)

`ID A0 00 00 00 00 00 00 00 <mode>` — **byte 9 is the mode value, not a CRC**.
Modes: `0x01` current, `0x02` velocity, `0x03` position. No acknowledgement;
implementations send it **5×**. Switching **into position mode requires < 10 rpm**.

### Set ID (unaddressed)

`AA 55 53 <new_id> 00 00 00 00 00 00` — **no CRC; byte 9 is 0x00.** Sent **5×**;
the new ID persists in flash. **Every motor that hears this frame takes the new
ID** — send it with exactly one motor on the bus.

### Broadcast ID query (unaddressed)

Fixed frame `C8 64 00 00 00 00 00 00 00 DE`. Every motor replies with a
**drive-layout** telemetry frame beginning with its own ID. Replies are
unarbitrated: simultaneous answers collide into garbage.

## Motor → host telemetry

**Two reply layouts, selected by the command that elicited the reply.** Bytes
0–5, 8 and 9 are common; bytes 6–7 differ.

Common fields:

| Byte | Field | Decoding |
|---|---|---|
| 0 | ID | responding motor |
| 1 | mode | `0x01`/`0x02`/`0x03` |
| 2–3 | torque current | i16 BE; **amps = raw × 8 / 32767** |
| 4–5 | speed | i16 BE, rpm directly (signed) |
| 8 | faults | bitmask (below) |
| 9 | CRC-8/MAXIM over bytes 0–8 | advisory |

Reply to a **`0x74` query** — bytes 6–7:

| Byte | Field | Decoding |
|---|---|---|
| 6 | winding temperature | u8, °C directly |
| 7 | position | u8; **deg = raw × 360 / 255** (~1.4° steps) |

Reply to a **`0x64` drive frame or `0xC8` broadcast** — bytes 6–7:

| Byte | Field | Decoding |
|---|---|---|
| 6–7 | position | u16 BE; **deg = raw × 360 / 32767** (~0.011° steps) |

The drive reply carries **no temperature** — a parser that decodes byte 6 as °C
on a drive reply is reading the position high byte. There is no bus-voltage field
in either layout. The u8 position divides by **255**, not 256.

### Fault byte (byte 8)

| Bit | Meaning | Trip | Release |
|---|---|---|---|
| `0x01` | Sensor (hall/encoder) fault | — | auto ~5 s |
| `0x02` | Bus overcurrent | 3 A | auto ~5 s |
| `0x04` | Phase overcurrent | 4.6 A | auto ~5 s |
| `0x08` | Stall | locked > 5 s | auto ~5 s |
| `0x10` | Overheat | winding 80 °C | on cooling to 75 °C |
| `0x20`–`0x80` | reserved | | |

`0x00` means no fault. While a protection is active the motor stops responding to
drive commands and flags the corresponding bit.

## Modes

| Mode | Wire | Setpoint range | Physical meaning |
|---|---|---|---|
| Current | `0x01` | −32767 … +32767 (i16) | ≈ −8 … +8 A |
| Velocity | `0x02` (default) | −330 … +330 (i16) | rpm |
| Position | `0x03` | 0 … 32767 (u16) | 0° … 360° |

Position mode is single-turn absolute (the 4096-line encoder underlies the
0–360° range).

## Known contradictions

Resolved and unresolved disagreements between the sources are documented in full
in [`PROTOCOL.md`](https://github.com/dougcalobrisi/m0601-rs-test/blob/main/PROTOCOL.md#known-contradictions-between-sources)
in the repo. In brief: reply CRC is real but treated as advisory; the accel byte
is byte 6 (not 4); the set-ID frame carries no CRC; the ≥50 Hz floor is
empirical; mode-switch/set-ID are sent 5×; and the accel byte's *unit* (rate vs
time-constant) is unresolved — only its direction (1 = fastest) is relied on.

## Sources

Official: the [DFRobot wiki protocol
reference](https://wiki.dfrobot.com/fit1042/docs/23322) and the FIT1042/FIT1038
product pages. Cross-checked implementations: the [DDT vendor
sample](https://github.com/tech-life-hacking/DDT_M0601C_111) (authoritative where
sources disagree), [navigation_robot](https://github.com/Il1yasviel/navigation_robot)
(ESP32 C driver with test vectors), and
[MotorLink](https://github.com/MukeshSankhla/MotorLink).
