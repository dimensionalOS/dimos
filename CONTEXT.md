# Domain Context

This file defines the benchmark language used in code, artifacts, and reports.
It describes concepts, not implementations.

## Transport stack

The complete runtime path under test, including the DimOS or ROS API layer,
serialization, middleware, and any required router or daemon. A result belongs
to one named stack; it is not a measurement of a middleware library in
isolation.

## Delivery cohort

A group of configurations with comparable delivery behavior. The
`best-effort/drop` cohort may discard data under congestion. The
`reliable/block` cohort applies backpressure to preserve delivery. Results from
different cohorts are not ranked against each other.

## Workload

A repeatable message pattern: one or more topics, their payload sizes and
offered rates, and whether the publisher is paced or saturating the path.

## Environment

The process and network placement used for a trial. `local` runs separate
processes on the host. `emulated` runs two endpoint containers on a dedicated
Docker bridge with symmetric `tc netem` conditions on the same Linux machine.

## Network profile

A named, recorded set of link conditions such as delay, jitter, loss, and rate
limit. A profile is part of a trial's identity.

## Campaign

The complete reproducible run: a manifest plus a randomized sequence of trial
cells and their artifacts.

## Cell

One unique combination of stack, delivery cohort, workload, environment,
network profile, and topology. Repetitions of a cell estimate run-to-run
variation.

## Trial

One independently started execution of a cell, including warmup, measurement,
and drain periods.

## Sample

One message observation containing sequence, timing, and publish-call data.
Samples are the raw evidence from which trial summaries are derived.
