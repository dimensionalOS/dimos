# Store static TF edges as timeless

Static TF edges are published and stored separately from timestamped dynamic history, remain valid at every query time, and do not participate in dynamic time-tolerance checks. The fixed-rate static-transform republishing workaround is removed; only dynamic edges in a composed lookup must have a sufficiently close measured sample.

## Considered Options

- Timeless static edges: chosen because fixed mount geometry has no meaningful measurement time and should not fail strict capture-time lookups.
- Periodically restamped static transforms: rejected because it wastes transport bandwidth, models immutable geometry as changing state, and can still miss a strict time tolerance between publications.
- Exempt all old transforms from tolerance: rejected because that would also allow stale dynamic robot poses.

## Consequences

TF transport and buffers must preserve whether an edge is static. A composed path may mix timeless static edges with timestamped dynamic edges, but freshness validation applies only to the dynamic samples.
