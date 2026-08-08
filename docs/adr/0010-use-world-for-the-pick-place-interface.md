# Use `world` for the pick/place interface

All generic pick/place inputs and outputs use the canonical `world` frame, including detections, object clouds, grasp candidates, obstacle proxies, and placement targets. Robot base and camera frames are transformed below this interface; missing transforms or frame mismatches fail before motion, and blueprint configuration cannot change the meaning of public coordinates.
