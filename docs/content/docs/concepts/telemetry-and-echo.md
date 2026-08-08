---
title: Telemetry and echo
weight: 3
---

# Telemetry, echoes, and a dangerous alignment bug

Two things make decoding a reply less obvious than reading ten bytes off the wire:
the motor answers in two different layouts, and many USB adapters prepend a copy of
what you just sent. Get either wrong and you don't get an error — you get a plausible,
confidently wrong number. On a motor with real torque, one of those wrong numbers is
dangerous.

## Two layouts, one command byte apart

Bytes 0–5, 8, and 9 of a reply are the same in both layouts: address, mode, current,
speed, faults, checksum. Only bytes 6–7 differ, and which meaning they carry depends
on the command that asked:

- A reply to a **`0x74` query** puts the winding **temperature** in byte 6 and a
  coarse 8-bit **position** in byte 7 (~1.4° per step).
- A reply to a **drive frame** (or the broadcast) uses bytes 6–7 together as a 16-bit
  **position** (~0.011° per step) and carries **no temperature** at all.

So the query reply is the only place temperature lives, and the drive reply is the
only place the precise angle lives. The driver decodes each reply by the command that
elicited it, which is the whole reason `Feedback` can be trusted: a `Query` reply and
a `Drive` reply that are byte-for-byte identical decode to *different* physical
readings, and the driver knows which is which because it knows what it sent.

That the same bytes mean two things is not a hypothetical. It caused a real bug in
this crate before the protocol was pinned down: byte 6 was read as temperature on a
drive reply, where it's actually the high byte of the position. The `parse_feedback`
doctest now demonstrates the double-decode on purpose, as a regression guard.

## Adapter echo, and the alignment trap

Half-duplex USB-RS485 adapters frequently echo the host's own transmission back
before the motor's reply arrives. So the bytes you read often start with an exact copy
of the frame you sent, followed by the real reply. Stripping that echo is easy when
it's whole: a genuine reply can never be byte-identical to the frame you transmitted
(its second byte is a mode value, not your command byte), so an exact leading copy is
always an echo, and off it comes.

The danger is a *truncated* echo — the adapter gives back only part of the copy. Now
the leading bytes aren't a clean copy to strip, but they aren't the reply either, and
if you parse from the wrong offset you get a frame straddling the tail of the echo and
the head of the real reply. Here's what makes it genuinely nasty rather than merely
wrong: that straddling garbage **doesn't look like garbage.** It begins with the
addressed motor's own ID (a truncated echo starts with the ID you sent, exactly as a
real reply does), so it passes the per-motor ID check, and it decodes to plausible
values.

The driver measured how plausible. Across every possible cut point of the echo, a
wheel actually turning at **300 RPM read back as 0, 1, 258, or 512 RPM** depending on
where the truncation fell. And for seven of the nine cut points, that misread speed
was **under 10 RPM** — which is the exact threshold callers check before entering
position mode, the one place a wrong "it's basically stopped" reading will let you
command a move that a spinning wheel should have refused.

That's why the echo handling is deliberately all-or-nothing: strip a *whole* echo, and
otherwise require the remaining bytes to be a clean, non-empty, exact multiple of the
10-byte frame length. A partial echo doesn't satisfy that, so it's rejected as no
reply — `Ok(None)` — rather than handed back as telemetry. A missing reading is safe.
A confident wrong one is not.

## The wrong-neighbor guard

One more filter, for the same reason. On a shared bus a stale frame in the adapter's
buffer, or one motor's late answer landing inside another motor's transaction window,
could be handed back as *this* motor's telemetry — a wheel reporting its neighbor's
speed. So the driver drops any reply whose address byte isn't the motor it was talking
to, surfacing it as `Ok(None)`. This guard matters more on a multi-motor bus than the
CRC does, which is why telemetry is never rejected on its checksum but always on its
address.
