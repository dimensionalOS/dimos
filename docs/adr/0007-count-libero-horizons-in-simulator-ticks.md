# Count LIBERO horizons in simulator ticks

The unified real-time LIBERO-PRO evaluation interprets each published suite action-step limit as the same number of continuously advancing 20 Hz simulator control ticks. Blueprint and simulator startup plus evaluator-owned settling finish before the counter starts; afterward every tick consumes the horizon even when the policy has not issued a new command. This preserves the native horizon's simulated physical duration, penalizes policy latency as real execution would, and avoids recreating paused inference by counting only policy updates.
