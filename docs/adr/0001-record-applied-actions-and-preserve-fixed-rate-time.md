# Record applied actions and preserve fixed-rate time

Robot-learning actions are hardware-accepted, post-arbitration position commands, not future measured joint states. Fixed-rate conversion excludes incomplete episodes by default; an explicit fill mode may preserve missing grid slots with causal holds only when every filled frame is marked, because silently dropping slots shortens demonstrations and changes their apparent motion speed.
