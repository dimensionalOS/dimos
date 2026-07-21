Subject: Feature idea — let the agent record its own training data

Hey team,

Quick one I want to float. Laying it out simply:

The feature: a skill that lets you tell the agent to record a training episode, hands-free.

Inputs: plain commands to the agent, like "start recording," "tag that as a successful grasp," "stop."

Outputs: a clean, labeled episode (camera, depth, lidar, poses, actions), exported in LeRobot/HDF5 and ready to train a VLA on.

Why: data is the bottleneck for training VLAs. We already have the recording and export plumbing (memory2 plus the learning module); we just can't drive it from the agent yet. And the real win isn't the recording, it's the labeling. Raw footage is close to useless. Footage with clean start/stop points and a success flag is the good stuff, and having the agent mark it live is a cheap way to get there.

Worth a prototype?

Yusuf
