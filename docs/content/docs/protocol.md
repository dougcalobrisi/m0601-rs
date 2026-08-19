---
title: Protocol reference
weight: 80
---

# M0601 protocol & hardware reference

Everything this repo knows about the DFRobot **M0601** direct-drive hub motor,
compiled from the official documentation and three independent implementations,
cross-checked byte-for-byte against this crate's test vectors. Where sources
disagree, the disagreement is recorded (see [Known
contradictions](#known-contradictions-between-sources)) rather than silently resolved.

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

From the DFRobot product pages (3076/3077, identical tables):

| Parameter | Value |
|---|---|
| Operating voltage | 18 V DC (MotorLink's README gives a 12 V minimum) |
| Rated current | 1.25 A |
| Stall current | ≤ 2.7 A |
| No-load current | ≤ 0.25 A (MotorLink's README says ≤ 0.2 A) |
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
| Mounting | M2.5 thread (wiki: 5 mm depth; MotorLink README: 6 mm), ⌀15.2 mm boss with 8 mm flat |

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
- **Acceleration** (byte 6): 0–255. `0` = the motor's own default ramp.
  **Which end of the range is the gentle one is undocumented and unmeasured** —
  see [contradiction 6](#known-contradictions-between-sources) before assuming a bigger number is
  softer.
- **Brake** (byte 7): `0xFF` engages the electric brake (velocity mode only);
  otherwise `0x00`.

Worked examples (ID `0x01`, accel 0) — these are independent implementations' verified
vectors, reproduced as golden tests in `m0601/tests/vectors.rs`:

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
Modes: `0x01` current, `0x02` velocity, `0x03` position. The motor sends no
acknowledgement, so implementations commonly send the frame **5×**, ~20 ms apart, to
be sure it lands (the DDT vendor sample sends it once; the frame is idempotent, so
repeating is harmless). Switching **into position mode requires < 10 rpm**.

### Set ID (unaddressed)

`AA 55 53 <new_id> 00 00 00 00 00 00` — **no CRC; byte 9 is 0x00.** Sent **5×**;
the new ID persists in flash. **Every motor that hears this frame takes the new
ID** — send it with exactly one motor on the bus.

### Broadcast ID query (unaddressed)

Fixed frame `C8 64 00 00 00 00 00 00 00 DE` (`0xDE` is its genuine CRC-8/MAXIM, not
a magic constant). Every motor replies with a **drive-layout** telemetry frame
beginning with its own ID. Replies are unarbitrated: several motors answering at once
collide into garbage belonging to none of them.

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
on a drive reply is reading the position high byte. (This crate had exactly that bug
before this reference existed; the DDT vendor sample's `Control_Motor` vs `Get_Motor`
parsing is what settles the two layouts.) There is no bus-voltage field in either
layout.

The u8 position divides by **255**, not 256 — `0xFF` ≡ 360° ≡ 0° wrapped — and every
known implementation agrees on that.

### Fault byte (byte 8)

| Bit | Meaning | Trip | Release |
|---|---|---|---|
| `0x01` | Sensor (hall/encoder) fault | — | auto ~5 s |
| `0x02` | Bus overcurrent | 3 A | auto ~5 s |
| `0x04` | Phase overcurrent | 4.6 A | auto ~5 s |
| `0x08` | Stall | locked > 5 s | auto ~5 s |
| `0x10` | Overheat | winding 80 °C | on cooling to 75 °C |
| `0x20`–`0x80` | reserved | | |

`0x00` means no fault. Multiple bits can be set at once; while a protection is active
the motor stops responding to drive commands and flags the corresponding bit.

> **Naming note.** MotorLink labels `0x10` "Troubleshoot"; the DFRobot wiki's protocol
> image and the navigation_robot C driver both call it the overheat/over-temperature
> fault. This crate follows the wiki.

## Modes

| Mode | Wire | Setpoint range | Physical meaning |
|---|---|---|---|
| Current | `0x01` | −32767 … +32767 (i16) | ≈ −8 … +8 A |
| Velocity | `0x02` (default) | −330 … +330 (i16) | rpm |
| Position | `0x03` | 0 … 32767 (u16) | 0° … 360° |

- The rated and no-load speeds (115 / 200 rpm) sit well inside the ±330 command
  range, so **commanding 330 rpm does not mean reaching it**.
- Position mode is single-turn absolute (the 4096-line encoder underlies the 0–360°
  range). Unwrapping it into continuous travel is
  [`PositionAccumulator`]({{< relref "library/odometry" >}})'s job.

## Known contradictions between sources

The sources do not all agree. Rather than silently pick a winner, each disagreement
is recorded here with how it was settled — or that it wasn't.

**1. Reply byte 9 — resolved by hardware capture.** The wiki's tables label it CRC8
and the navigation_robot C driver validates CRC-8/MAXIM on replies; this crate's
original observation claimed genuine replies fail that check, and the DDT vendor
sample and MotorLink ignore reply checksums entirely. Running
`reply_checksum_capture` (`m0601/tests/hardware.rs`) against a real unit settled it —
**replies do carry a valid CRC-8/MAXIM**, in both layouts:

```text
0x74 query reply: 01 02 00 37 00 00 1E BB 00 D5  → CRC(bytes 0-8) = D5, matches
0x64 drive reply: 01 02 00 39 00 00 5D F1 00 6F  → CRC(bytes 0-8) = 6F, matches
```

(Those two frames also confirm the dual layout with live data: the query's u8
position `0xBB` → 264.0° and the drive reply's u16 `0x5DF1` → 264.2° are the same
physical angle through both encodings.) The original "replies fail CRC" observation
was most likely made against echo-contaminated or mis-framed reply bytes. The
recommendation is unchanged: **treat the reply CRC as advisory rather than rejecting
on it** — two of four reference implementations never check it, and firmware
revisions may differ. The driver follows that default, with a
[strict opt-in]({{< relref "library/quickstart" >}}) for callers who'd rather drop a
suspect frame.

**2. Acceleration byte position.** Wiki + DDT vendor sample + navigation_robot:
**byte 6**. MotorLink's Python examples put it at byte 4 — a discrepancy that never
surfaces there, because all its captured vectors use accel 0. Byte 6 is correct.

**3. Set-ID checksum.** MotorLink's README table shows a CRC (`…00 CB`, which *is*
the CRC-8/MAXIM of that frame) in byte 9, but the wiki, the vendor sample, and
MotorLink's own `m0601_set_id.py` all send `0x00`. No CRC is correct.

**4. The ≥50 Hz floor** is community/empirical, not official — official docs state
only the 500 Hz maximum.

**5. Repeat counts.** Mode-switch and set-ID frames: the vendor sample sends once;
MotorLink and navigation_robot send 5×. Sending 5× is harmless (the frames are
idempotent) and is what this crate does.

**6. Acceleration byte direction — unstated everywhere, and this crate used to claim
otherwise.** Earlier versions of this page said "every source, the wiki included, says
`1` is the fastest ramp and larger values are gentler." **That claim was wrong: no
source says it.** What the sources actually say:

The **upstream DDT manual** (`M0601C_111 Motor Driver Instructions`,
[PDF](https://d2air1d4eqhwg2.cloudfront.net/media/files/a48110eb-432c-4083-a159-9e0f35913b23.pdf),
p. 10) gives one sentence, and it is only a unit:

> "Acceleration：Valid in velocity loop. unity: RPM/0.1ms. When set to 0, it would be
> the default value"

No direction, no worked example, and — note — no statement that the default equals
`1`. `RPM/0.1ms` is a **rate**, and if the unit is taken at face value a *larger* byte
ramps *harder*, the opposite of what this crate documented.

The **DFRobot wiki** ([FIT1042](https://wiki.dfrobot.com/fit1042/docs/23322), identical
on [FIT1038](https://wiki.dfrobot.com/fit1038/docs/23322)) repeats the sentence with
additions, and the result contradicts itself three ways:

> "Acceleration time: Valid in velocity loop mode. unity: 1 rpm/0.1 ms. When you set to
> 1, acceleration time is 10*0.1 ms = 1 ms each 1 rpm. When set to 0, it would be the
> default value as 1."

1. The field is renamed "acceleration **time**", but the unit given is a **rate**.
2. The worked example re-reads that unit as **time per rpm** — the inverse orientation.
3. The factor of ten is unexplained: setting the byte to `1` supposedly yields
   `10 * 0.1 ms`, not `1 * 0.1 ms`.

The wiki's one durable addition is that the default equals `1`, which is why
`DEFAULT_DRIVE_ACCEL` is `1` — that is a statement about the *default*, not about
direction. **MotorLink**'s README inverts the unit again ("`0` = default
(`1` = 0.1ms/rpm)"), i.e. time per rpm.

Eight independent implementations were checked (tech-life-hacking/DDT_M0601C_111,
DDTRobot/motor-driver-examples, Il1yasviel/navigation_robot,
HarvestX/DDT-M0601C-112-U2D2, Ar-Ray-code/ddt_m06_ros2_driver,
takex5g/M5_DDTMotor_M15M06, LonelyMarch/Basic_Framework_MC02, MotorLink) along with the
DFRobot product pages, Hackster, ElectronicWings and Instructables. **None states the
direction, and no published measurement resolves it.**

Consequently this crate no longer picks a side, and no longer picks byte values that
depend on one: `BusTiming::stop_accel`, the `control` CLI default, and the shipped
`m0601-quad` `limits.accel` are all `0` (the motor's own default) where they were
previously `5`, `3` and `5` — numbers chosen on the assumption that larger is gentler,
which the upstream unit contradicts. `m0601-quad` warns on any nonzero `limits.accel`.
The direction-independent way to keep a launch under the 3 A trip is to bound the
*step*, with `SlewLimiter`, not to guess at this byte.

**Settling it** needs a rig, not another source: command a step from 0 to a fixed RPM
at `accel = 1`, repeat at `20` and `100`, log telemetry, compare time-to-setpoint. If
time-to-setpoint grows with the byte, larger is gentler; if it shrinks, the unit is
literal. Tracked in [#2](https://github.com/dougcalobrisi/m0601-rs/issues/2).

**7. Minor spec conflicts.** No-load current ≤ 0.25 A (product page) vs ≤ 0.2 A
(MotorLink README); mounting thread depth 5 mm (wiki) vs 6 mm (README); supply 18 V
(wiki/product page) vs a 12 V minimum (MotorLink).

**8. No official PDF datasheet exists.** The wiki's protocol section is published as
images only.

## Sources

Official: the [DFRobot wiki protocol
reference](https://wiki.dfrobot.com/fit1042/docs/23322) and the FIT1042/FIT1038
product pages. Cross-checked implementations: the [DDT vendor
sample](https://github.com/tech-life-hacking/DDT_M0601C_111) (authoritative where
sources disagree), [navigation_robot](https://github.com/Il1yasviel/navigation_robot)
(ESP32 C driver with test vectors), and
[MotorLink](https://github.com/MukeshSankhla/MotorLink).
