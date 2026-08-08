# Replace perception obstacles with each scan

Every completed scan atomically replaces the perception-derived obstacle set while leaving static environment obstacles untouched. Simulation may derive Object Obstacle Proxies from complete mesh clouds, while real RGB-D perception uses conservatively padded upright boxes from partial segmented clouds; the real path does not use the sim convex-hull assumption or silently clamp obstacle width. The selected target remains registered, and the agreed Unchecked Contact Motion handles intentional contact legs.
