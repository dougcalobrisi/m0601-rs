---
title: set-id
weight: 6
---

# `set-id` — change a motor's address

```sh
m0601 set-id --new 0x02        # prompts for confirmation
m0601 set-id --new 0x02 --yes  # skips the prompt
```

Every motor on an RS485 bus needs a unique address. Fresh motors all ship at `0x01`,
so before you can put two on one bus you have to renumber them — one at a time, with
this command. The new address is written to flash and survives power cycles.

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--new <id>` | *(required)* | the new address, `0x01..0xFE` (hex or decimal) |
| `--yes` | off | skip the confirmation prompt |

## Why it insists on exactly one motor

The set-ID frame is **unaddressed**. It carries no target — every motor that hears
it takes the new address. Put two motors on the bus and run `set-id` once, and now
you have two motors at the same address, which you can only untangle by
disconnecting them one at a time and renumbering again. It's persistent state that's
tedious to undo, so the command works hard to stop you doing it by accident.

Its guard is an **exhaustive scan of all 254 addresses** before it writes:

```
====================================================
  M0601 ID Changer
  Port: /dev/ttyUSB0  ->  New ID: 0x05 (5)
====================================================
  WARNING: only ONE motor may be on the bus. ID is persistent.
  Checking the bus is not shared (polling all 254 IDs)... done.
```

Why all 254, and not the fast broadcast? Because a broadcast scan can't tell one
motor from several answering at once — their replies collide. The only way to *prove*
a single motor is present is to poll every address individually and see exactly one
answer. It's slow (~40 s), and that's the price of not corrupting persistent state.

The scan settles the outcome:

- **No motor** → `[x] No motor detected. Check power/wiring.` (exit non-zero)
- **More than one** → `[x] Multiple motors detected [0x01, 0x03]. Disconnect all but
  one.` (exit non-zero)
- **Already at the target** → `[!] Already at that ID. Nothing to do.` (success)

## Confirmation

With exactly one motor found and not already at the target, it asks:

```
Change 0x01 -> 0x02? type 'yes':
```

Anything other than `yes` (case-insensitive) cancels. `--yes` skips this — use it in
scripts, but only when you already know one motor is on the bus.

## After the write

The library sends the frame five times, waits half a second for the flash write to
settle, then re-queries to confirm:

- **Confirmed** → `[ok] SUCCESS — motor ID is now 0x02. Use --id 0x02.`
- **Reports a different ID** → `[x] Motor reports 0x05 — change may have failed. Try
  power-cycling.` (exit non-zero)
- **No answer** → `[?] No response after change. Power-cycle and run 'scan' to
  confirm.` (exit non-zero)

Avoid assigning `0xC8`: it's the broadcast query's own address, and a motor sitting
there can't be told apart from the query on an adapter that echoes.

## See also

- [`scan --full`]({{< relref "scan" >}}) — the same exhaustive poll, on demand.
- [Concepts → The bus]({{< relref "../concepts/the-bus" >}}) — addressing and why
  collisions look the way they do.
