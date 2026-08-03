# M0601 hub motor — protocol & hardware reference

Everything this repo knows about the DFRobot **M0601** direct-drive hub
motor, compiled from the official documentation and three independent
implementations, cross-checked byte-for-byte against this crate's test
vectors. Where sources disagree, the disagreement is recorded (see
[Known contradictions](#known-contradictions-between-sources)) rather than
silently resolved.

## Identity

The "DFRobot M0601" is a rebadged **Direct Drive Tech (DDT) M0601C-111**.
DFRobot sells it as two SKUs:

| SKU | Side | DFRobot product |
|---|---|---|
| **FIT1042** | Left  | product-3077 |
| **FIT1038** | Right | product-3076 |

The two are mechanically and electrically identical and speak the same
protocol; only the (directional) tire tread differs. Either spins either
way — see the library's `mirrored()` handle for sign conventions.

Related DDT models — **M0601C-112**, M0602C, M1502A, DDSM115 — are *not*
protocol-guaranteed identical. Do not port framing from their sample code
without re-verification.

## Electrical & mechanical specifications

From the DFRobot product pages (3076/3077, identical tables):

| Parameter | Value |
|---|---|
| Operating voltage | 18 V DC (MotorLink README: 12 V minimum) |
| Rated current | 1.25 A |
| Stall current | ≤ 2.7 A |
| No-load current | ≤ 0.25 A (README says ≤ 0.2 A) |
| Rated speed | 115 rpm |
| No-load speed | 200 ± 10 rpm |
| Rated torque | 0.96 N·m |
| Stall torque | 2.0 N·m |
| Torque constant | 0.75 N·m/A |
| Speed constant | 11.1 rpm/V |
| Encoder resolution | 4096 (relative accuracy 1024) |
| Protection rating | IP54 |
| Noise | ≤ 50 dB |
| Operating temperature | −20 … 45 °C |
| Wheel diameter | 102 mm |
| Drive | direct — the wheel is the rotor; no gearbox, no backlash |
| Mounting | M2.5 thread (wiki: 5 mm depth; README: 6 mm), ⌀15.2 mm boss with 8 mm flat |

Note the stall current (≤ 2.7 A) against the current-loop command range
(±32767 ≈ ±8 A): the top of the commandable range is far beyond what the
motor can physically draw, and the 3 A bus-overcurrent protection trips
long before it.

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
2. A floating brown wire causes intermittent comms errors, especially on
   longer cables.
3. Add a 120 Ω termination resistor across A/B for cable runs over ~1 m.
4. The 18 V supply is independent of the RS485 transceiver: a powered-down
   motor is silent on the bus, not absent-with-errors.
5. Keep exactly one motor on the bus when assigning IDs (see set-ID below).

DFRobot's recommended adapter is the RainbowLink **TEL0185** (CH343).

## Link layer

- RS485, half-duplex, multi-drop; **115200 baud, 8N1**.
- Every frame in both directions is **exactly 10 bytes**.
- Motor addresses: `0x01..=0xFE`. `0x00`/`0xFF` are reserved; `0xC8` is the
  broadcast destination of the ID query, so avoid assigning it (a motor at
  `0xC8` is indistinguishable from the query itself on an echoing adapter).
- Many USB adapters echo their own transmission: RX may open with an exact
  copy of the TX frame. Strip it before parsing (a genuine reply never
  byte-equals the TX frame — its byte 1 is a mode value, not the command).

### Polling model

This is a **polling protocol**: a drive command does not latch. Officially
documented: a maximum command rate of **500 Hz**. Community consensus
(MotorLink's default, this crate's observation): resend drive frames at
**~50 Hz (≤ every 20 ms)** or the motor does not sustain motion and coasts
to a stop. The 50 Hz floor appears in no official document — treat it as
empirical — but its consequence is the protocol's built-in fail-safe: if
the host dies, the wheel coasts.

Power-up defaults: **velocity mode**, ID as last assigned (stored in
flash).

## CRC

Standard host frames carry a checksum over bytes 0-8 in byte 9:
**CRC-8/MAXIM** (Dallas/1-Wire): polynomial x⁸+x⁵+x⁴+1, reflected
implementation constant `0x8C`, init `0x00`, no final XOR. Check value:
`crc8("123456789") = 0xA1`.

```text
crc = 0
for byte in data:
    crc ^= byte
    repeat 8: crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
```

**Exceptions — two host frames carry no CRC:**

- the mode-switch frame (`0xA0`): byte 9 holds the **mode value**;
- the set-ID frame: byte 9 is `0x00`.

Replies carry the same CRC — verified on real hardware by this repo (see
[Known contradictions](#known-contradictions-between-sources), item 1, for
the capture) — but drivers should treat it as advisory, not grounds for
rejection.

## Host → motor frames

### Drive (command `0x64`)

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| | ID | `0x64` | value HI | value LO | 0 | 0 | accel | brake | 0 | CRC |

- The 16-bit big-endian value in bytes 2–3 is interpreted **per the
  motor's active mode** (see [Modes](#modes)). A zero value therefore does
  *not* universally mean "stop": it is 0 rpm in velocity mode, but "drive
  to 0°" in position mode and "zero torque" (a coast) in current mode.
- **Acceleration** (byte 6): ramp steepness, range 0–255. `0` selects
  the motor default; `1` is the *fastest* ramp, and larger values ramp
  more gently. The wiki also gives a unit, “1 rpm per 0.1 ms”, which
  reads as a rate and so contradicts that direction — see
  [Known contradictions](#known-contradictions-between-sources), item 6.
  Only the direction is relied on by this crate.
- **Brake** (byte 7): `0xFF` engages the electric brake (velocity mode
  only); any other value must be `0x00`.

Worked examples (ID `0x01`, accel 0 — from independent implementations'
verified vectors, reproduced in `m0601/tests/vectors.rs`):

```text
+100 rpm      01 64 00 64 00 00 00 00 00 4F
-150 rpm      01 64 FF 6A 00 00 00 00 00 5A
brake         01 64 00 00 00 00 00 FF 00 D1
position 8192 01 64 20 00 00 00 00 00 00 BF   (≈ 90°)
```

### Feedback query (command `0x74`)

`ID 74 00 00 00 00 00 00 00 CRC` — for ID `0x01`: `01 74 00…00 04`.
The addressed motor answers with the **query-layout** telemetry frame
(the only reply that carries the winding temperature).

### Mode switch (command `0xA0`)

`ID A0 00 00 00 00 00 00 00 <mode>` — **byte 9 is the mode value, not a
CRC**. Modes: `0x01` current, `0x02` velocity, `0x03` position. The motor
sends no acknowledgement; implementations commonly send the frame **5×**
(~20 ms apart) to be sure it lands (the DDT vendor sample sends it once).

Constraint: switching **into position mode requires the wheel to be
turning slower than 10 rpm**.

### Set ID (unaddressed)

`AA 55 53 <new_id> 00 00 00 00 00 00` — **no CRC; byte 9 is 0x00.**
Convention is to send it **5×**; the new ID persists in flash.

**Every motor that hears this frame takes the new ID.** It is unaddressed,
so it must only ever be sent with exactly one motor on the bus — and a
single broadcast scan cannot prove there is only one (simultaneous replies
collide). Poll all 254 IDs first when it matters.

### Broadcast ID query (unaddressed)

Fixed frame `C8 64 00 00 00 00 00 00 00 DE` (`0xDE` is its genuine
CRC-8/MAXIM). Every motor on the bus replies with a **drive-layout**
telemetry frame beginning with its own ID. Replies are unarbitrated:
several motors answering at once collide into garbage belonging to none of
them.

## Motor → host telemetry

**There are two reply layouts, selected by the command that elicited the
reply.** Bytes 0–5, 8 and 9 are common; bytes 6–7 differ:

Common fields:

| Byte | Field | Decoding |
|---|---|---|
| 0 | ID | responding motor |
| 1 | mode | `0x01`/`0x02`/`0x03` |
| 2–3 | torque current | i16 BE; **amps = raw × 8 / 32767** |
| 4–5 | speed | i16 BE, rpm directly (signed) |
| 8 | faults | bitmask, below |
| 9 | CRC-8/MAXIM over bytes 0-8 | verified on hardware; treat as advisory |

Reply to a **`0x74` query**:

| Byte | Field | Decoding |
|---|---|---|
| 6 | winding temperature | u8, °C directly |
| 7 | position | u8; **deg = raw × 360 / 255** (~1.4° steps) |

Reply to a **`0x64` drive frame or the `0xC8` broadcast**:

| Byte | Field | Decoding |
|---|---|---|
| 6–7 | position | u16 BE; **deg = raw × 360 / 32767** (~0.011° steps) |

The drive reply carries **no temperature** — a parser that decodes byte 6
as °C on a drive reply is reading the position high byte. (This crate had
exactly that bug before this document existed; the DDT vendor sample's
`Control_Motor` vs `Get_Motor` parsing settles the layouts.) There is no
bus-voltage field in either layout.

The u8 position divides by **255**, not 256 (`0xFF` ≡ 360° ≡ 0° wrapped);
every known implementation agrees.

### Fault byte (byte 8)

| Bit | Meaning | Trip | Release |
|---|---|---|---|
| `0x01` | Sensor (hall/encoder) fault | — | auto ~5 s |
| `0x02` | Bus overcurrent | 3 A | auto ~5 s |
| `0x04` | Phase overcurrent | 4.6 A | auto ~5 s |
| `0x08` | Stall | locked > 5 s | auto ~5 s |
| `0x10` | **Overheat** | winding 80 °C | on cooling to 75 °C |
| `0x20`–`0x80` | reserved | | |

`0x00` means no fault. Multiple bits can be set at once; while a
protection is active the motor stops responding to drive commands and
flags the corresponding bit.

> Naming note: MotorLink labels `0x10` "Troubleshoot"; the DFRobot wiki's
> protocol image and the navigation_robot C driver both call it the
> overheat/over-temperature fault. This repo follows the wiki.

## Modes

| Mode | Wire | Setpoint range | Physical meaning |
|---|---|---|---|
| Current | `0x01` | −32767 … +32767 (i16) | ≈ −8 … +8 A (`A = raw × 8/32767`) |
| Velocity | `0x02` (default) | −330 … +330 (i16) | rpm |
| Position | `0x03` | 0 … 32767 (u16) | 0° … 360° (`deg = raw × 360/32767`) |

- Rated/no-load speeds (115 / 200 rpm) sit well inside the ±330 command
  range; commanding 330 rpm does not mean reaching it.
- Position mode is single-turn absolute (the 4096-line encoder underlies
  the 0–360° range).

## Known contradictions between sources

1. **Reply byte 9 — resolved by hardware capture.** The wiki's tables
   label it CRC8 and the navigation_robot C driver validates CRC-8/MAXIM
   on replies; this crate's original observation claimed genuine replies
   fail that check, and the DDT vendor sample and MotorLink ignore reply
   checksums entirely. Running `reply_checksum_capture`
   (`m0601/tests/hardware.rs`) against a real unit settled it — **replies
   do carry a valid CRC-8/MAXIM**, in both layouts:

   ```text
   0x74 query reply: 01 02 00 37 00 00 1E BB 00 D5  → CRC(bytes 0-8) = D5, matches
   0x64 drive reply: 01 02 00 39 00 00 5D F1 00 6F  → CRC(bytes 0-8) = 6F, matches
   ```

   (These two frames also confirm the dual layout with live data: the
   query's u8 position `0xBB` → 264.0° and the drive reply's u16 `0x5DF1`
   → 264.2° are the same physical angle through both encodings.)
   The original "replies fail CRC" observation was most likely made
   against echo-contaminated or mis-framed reply bytes. Recommendation
   unchanged: **treat the reply CRC as advisory rather than rejecting on
   it** — two of four reference implementations never check it, and
   firmware revisions may differ.
2. **Acceleration byte position.** Wiki + DDT vendor sample +
   navigation_robot: **byte 6**. MotorLink's Python examples put it at
   byte 4 — a bug that never surfaced because all its captured vectors use
   accel 0. Byte 6 is correct.
3. **Set-ID checksum.** MotorLink's README table shows a CRC (`…00 CB`,
   which *is* the CRC-8/MAXIM of that frame) in byte 9, but the wiki, the
   vendor sample, and MotorLink's own `m0601_set_id.py` all send `0x00`.
   No CRC is correct.
4. **The ≥50 Hz floor** is community/empirical, not official — official
   docs state only the 500 Hz maximum.
5. **Repeat counts.** Mode-switch and set-ID frames: vendor sample sends
   once; MotorLink and navigation_robot send 5×. Sending 5× is harmless
   (the frames are idempotent) and is what this crate does.
6. **Acceleration byte unit vs direction — unresolved.** The wiki states
   the unit as “1 rpm per 0.1 ms”, i.e. a *rate*, under which a larger
   value would ramp *faster*. Every source — the wiki included — also
   says `1` is the fastest ramp and larger values are gentler, which is
   the behaviour of a *time constant*. The two cannot both hold. Not
   yet resolved against hardware; this crate documents only the
   direction, which all sources agree on.
7. Minor spec conflicts: no-load current ≤ 0.25 A (product page) vs
   ≤ 0.2 A (README); mounting thread depth 5 mm (wiki) vs 6 mm (README);
   supply 18 V (wiki/product page) vs a 12 V minimum (MotorLink).
8. **No official PDF datasheet exists.** The wiki's protocol section is
   published as images only.

## Sources

Official:

- DFRobot wiki — protocol reference: <https://wiki.dfrobot.com/fit1042/docs/23322>
- DFRobot wiki — getting started: <https://wiki.dfrobot.com/fit1042/docs/23321>
- DFRobot product page, FIT1042 (left): <https://www.dfrobot.com/product-3077.html>
- DFRobot product page, FIT1038 (right): <https://www.dfrobot.com/product-3076.html>

Implementations (cross-checked):

- DDT vendor sample (authoritative where sources disagree):
  <https://github.com/tech-life-hacking/DDT_M0601C_111>
- navigation_robot ESP32 C driver, with unit-test vectors:
  <https://github.com/Il1yasviel/navigation_robot>
- MotorLink (DFRobot-branded community tool; the outlier in the
  contradictions above): <https://github.com/MukeshSankhla/MotorLink>
