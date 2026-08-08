---
title: Stopping safely
weight: 4
---

# Stopping safely

Stopping a motor sounds like it should be the easy part — send zero. It isn't,
because on this motor zero doesn't mean what you'd assume, and because a stop often
has to run from the worst possible moment: a panic, a signal, a dropped connection.

## Why `safe_stop` switches modes before it sends anything

A zero setpoint means "stop" only in velocity mode. The identical zero-valued drive
frame means "rotate to 0°" in position mode — a stop command that could spin the wheel
up to half a turn on its way to "stopping" — and in current mode it means zero torque,
a coast, with the brake byte ignored entirely.

Now consider where a stop gets called from. A panic handler. A `SIGTERM`. A stop guard
unwinding mid-operation. In none of those places does the code reliably know what mode
the motor is currently in. So `safe_stop` doesn't assume — it *establishes* velocity
mode first, then sends zero, then brakes. Force the one mode where zero means stop,
and the sequence is correct no matter what the motor was doing when things went wrong.

The full sequence is five velocity-mode switch frames, then five zero-velocity frames,
then five brake frames, 20 ms apart — about 300 ms. It's a step to zero at the motor's
fastest setting followed by the electric brake, not a gentle ramp. And it's
best-effort: it swallows every I/O error and keeps sending, because even total failure
is safe. If not one frame gets through, the wheel still coasts to a stop, because the
frames stopped arriving. The fail-safe is the floor under everything.

## Stopping a whole vehicle without yawing

Stop four wheels one at a time and you've built a bug. Braking wheel 1 while wheels
2–4 still coast means, on a skid-steer chassis, one side biting while the other rolls
— an uncommanded yaw. The robot turns as it "stops."

`safe_stop_all` refuses to do that. It goes **round-major**: it sends step one of the
sequence to every motor, then step two to every motor, and so on — five velocity-mode
rounds, five zero rounds, five brake rounds. Every wheel gets the same command at
nearly the same moment, so they spin down together and the whole vehicle takes the same
~300 ms a single wheel would. (With enough motors that a round can't fit in its 20 ms
window, a round just runs long and the next starts late — the stop still completes, it
only takes a bit more than 300 ms.)

Like the single stop, it's best-effort and swallows errors: it runs on shutdown paths
where "keep telling the *other* motors to stop" beats bailing out on the first one that
didn't answer. And because `Bus` is `Clone`, a signal handler can hold its own handle
and stop the entire vehicle from outside the normal control flow.

## The one stop nothing can make

Every graceful and not-so-graceful exit brakes: normal completion, `?` errors, panics,
`Ctrl-C`, `SIGTERM`, `SIGHUP`. The exceptions are `SIGKILL` and losing power, where no
code runs at all. There the motor coasts, by protocol — which is safe, but worth
saying plainly: **`kill -9` is not an emergency stop.** It ends the process that was
holding the wheel at speed and lets it spin down on its own, rather than braking it. If
you need a hard stop, cut motor power; the software can only ever be as fast as its
~300 ms braked sequence, and only while it's alive to run it.
