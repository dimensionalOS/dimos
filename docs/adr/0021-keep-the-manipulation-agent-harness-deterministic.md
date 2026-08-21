# Keep the manipulation agent harness deterministic

The initial Manipulation Agent Harness uses deterministic typed APIs, state transitions, trace extraction, evidence packaging, and policy-candidate lifecycle, with the task Agent as the only model interpreting Trial Evidence. We reject automatic verifier or diagnoser model calls because they would move behavioral prompting into hidden middleware, make benchmark gains harder to attribute, and introduce another stochastic research subject before the basic harness is proven.
