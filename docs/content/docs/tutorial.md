---
title: First spin
weight: 3
---

# Your first spin

This is the motor-on-the-bench walkthrough: from bare wires to a spinning wheel and
back to a controlled stop, with what you should see at each step and what to do when
you don't. It assumes you've built the CLI ([Getting started]({{< relref
"getting-started" >}})).

> [!CAUTION]
> A direct-drive hub motor has real torque (2 N·m stall) and no gearbox to slow it
> down. Before anything spins: get the wheel off the ground or clear of fingers,
> cables, and the edge of the bench. `Ctrl-C` brakes, but only while the process is
> alive to do it.

## 1. Wire it and power it

Signal cable to the adapter (white → A, orange → B, brown → ground), power cable to
18 V. If you've done this before and it didn't work, swap orange and white — the
M0601's A/B labels are backwards relative to many adapters.

## 2. Find the motor

```sh
m0601 scan
```

A fresh motor ships at ID `0x01`, so you should see something like:

```
Found 1 motor(s):
  - ID 0x01 (decimal 1)
Use:  --id 0x01
```

That `Use:` line is a copy-paste convenience for later commands. If instead you get
`No motors found`, the checklist it prints is the right order to work through:
18 V power on, brown wire grounded, and — most likely — A/B swapped. A silent bus is
almost always wiring, not software.

## 3. Read its state without moving it

```sh
m0601 info
```

`info` sends a single query and prints a configuration block plus one live readout —
mode, speed, current, position, winding temperature, and any fault bits. Nothing
moves. A healthy idle motor reads `+0 RPM`, a small current offset, and `Error:
0x00  OK`. If you see `Live readout: no valid response`, the bus found the motor
during scan but isn't getting a clean reply now — recheck the connection.

## 4. Spin it, briefly and on a timer

Now the wheel moves. `drive` holds one setpoint and resends it at 50 Hz until a
timer runs out or you interrupt, then it brakes. Start with a bounded run so it
stops itself:

```sh
m0601 drive velocity --rpm 100 --secs 3
```

The wheel ramps to 100 RPM, holds for three seconds, and brakes. You'll see a live
one-line readout while it runs and `Stopped and braked after 3.0 s.` at the end.
Because every exit path brakes — timer, `Ctrl-C`, even a panic — there's no way to
leave this command with the wheel coasting unless the whole process is killed.

Try reverse and a gentler ramp:

```sh
m0601 drive velocity --rpm -80 --secs 3         # the other way
m0601 drive velocity --rpm 200 --accel 40        # softer acceleration
```

That `--accel 40` matters more than it looks: acceleration `1` (the default) is the
motor's *fastest* ramp, and a big step at accel 1 on a loaded wheel can spike
current hard enough to trip the 3 A protection. Larger numbers ramp gentler.

## 5. Take the wheel

```sh
m0601 control --rpm 100
```

This opens a full-screen dashboard and drives from the keyboard. `F` and `B` go
forward and back at your preset; `1`–`5` jump to 50–250 RPM; the arrow keys nudge
±10; `S` stops; `K` is the electric brake. `Q` (or `Esc`, or `Ctrl-C`) quits — and
quitting forces velocity mode, zeroes, and brakes, so the wheel is always stopped
when you land back at the shell.

Watch the mode line. The dashboard shows the mode the **motor reports**, not just
what you asked for, and it turns red if the two disagree. That's deliberate: it's
exactly how a "brake" keypress could otherwise freewheel a wheel while the screen
cheerfully says BRAKING.

## Where to go next

- The [CLI guide]({{< relref "cli" >}}) documents every command in depth.
- If you're wiring this into your own robot code, switch to the
  [library guide]({{< relref "library" >}}).
- To understand *why* any of the above works the way it does — the 50 Hz floor,
  the braking dance, the mode display — read [Concepts]({{< relref "concepts" >}}).
