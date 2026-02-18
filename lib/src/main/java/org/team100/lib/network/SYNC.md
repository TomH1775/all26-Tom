# Sync

This describes our new design for clock syncing between cameras and RoboRio.

## Original design

As originally [designed](https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc), the Network Tables system included
periodic timestamp synchronization using an implementation of [Cristian's_algorithm]( https://en.wikipedia.org/wiki/Cristian%27s_algorithm),
an idea so simple that it's hard to believe it has a name:

1. The client sends the current client time, $c_0$.
2. The server notes the time at receipt, $s_0$.
3. The server sends $s_0$ and $c_0$ to the client. 
4. The client notes the time at receipt, $c_1$
5. The difference is the round-trip-time (RTT):

```math
RTT = c_1 - c_0
```

6. The client can surmise that $s_0$ happened exactly between $c_0$ and $c_1$.
7. In Network Tables the offset is added to the localtime to obtain the server time:

```math
s_0 = c_1 - \frac{RTT}{2} + offset
```

or

```math
offset = s_0 + \frac{RTT}{2} - c_1 
```

This offset was used to compute the server time for each data frame, which appears in the `timestamp` field.

```math
timestamp = clienttime + offset
```

## Revision

Immediately after release, this design caused [trouble](https://github.com/wpilibsuite/allwpilib/issues/5224), and it was [changed](https://github.com/wpilibsuite/allwpilib/commit/8b7c6852cf70d0cb9168014b68508bae77ed3fc8) from a periodic estimate to a one-shot estimate done [once at connection time](https://github.com/wpilibsuite/allwpilib/commit/8b7c6852cf70d0cb9168014b68508bae77ed3fc8#diff-2375aa79a1c18a37a91f82a9015410f427d1001400b94522fda5b4889b615493R700).  The reason given was that the variability of the measured round-trip time was sometimes high, and since the round-trip-time is simply added to the offset, and because each new packet produced a new offset, with no filtering of any kind, there was sometimes high variability in the offset.  This led to symptoms such as producing timestamps for the future.

## Problem

The main problem with the solution above is that it reduces the scope of the solution
from general clock synchronization to a single offset snapshot.  In reality, clocks
are not simply offset from each other -- they also go different speeds, resulting
in clock drift.

Because we use the Network Tables timestamp offset mechanism in the pipeline for vision,
the clock drift has been a problem.  We haven't understood it as such until now
(Feb 2026) -- in the past we attempted to simply "add magic numbers" to correct
what seemed like "extra delay".  These magic numbers have been "hard to tune"
because, of course, the correct "extra delay" depends on how long the robot has
been turned on!  It's not extra delay at all, it's drift.

## Solution

IMHO the main problem with the initial design is not periodicity, it's that each
update mixed two very different quantities, and applied them as completely
authoritative.  The RTT measurement is short (milliseconds) and noisy,
varying in normal operation by a millisecond, and in abnormal operation
by tens of milliseconds.  The offset is very long (a billion seconds), and it
is not noisy at all: it varies slowly, smoothly, and consistently.  The change
in offset might be 3 ms per minute -- on the same scale as the RTT noise, but
this change in offset is meaningful to the camera pipeline.

So the solution is to model the estimated quantities more realistically:

1. Offset is a constant with a low bias
2. RTT is lognormal noise

