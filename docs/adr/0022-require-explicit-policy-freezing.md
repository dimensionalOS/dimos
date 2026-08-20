# Require explicit policy freezing

Every debug submission becomes an immutable Policy Candidate, and the Agent must explicitly freeze one candidate as the task's Policy Artifact before exploration ends. We reject last-submission-wins and automatic debug-score selection because diagnostic or narrowly successful later attempts can replace a more robust earlier policy, while a deterministic harness cannot infer strategic quality from sparse native outcomes alone.
