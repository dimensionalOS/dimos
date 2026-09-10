# OCCUpancyGrid/Pointcloud Questions

1. How many distinct rooms are enclosed by the mapped walls?
2. Between 190 and 215 seconds, which 1 m x 1 m square has the most free-to-occupied and occupied-to-free changes, ignoring newly observed space?
3. What is the shortest collision-free loop for a robot of radius 0.25 m, starting at `(x1, y1)`, visiting `(x2, y2)` and `(x3, y3)`, and returning to the start?
4. What is the largest circular robot radius that can travel from `(x1, y1)` to `(x2, y2)`?
5. If the opening nearest `(x3, y3)` is blocked, can a robot of radius 0.25 m still travel from `(x1, y1)` to `(x2, y2)`?
6. Where are all likely doorway-sized openings, 0.6-1.2 m wide, in the mapped walls?
7. Where is the largest circle that fits entirely inside known-free space, and what is its radius?
8. Between 440 and 464 seconds, which intervals show the strongest evidence of a person or moving object crossing the mapped space, excluding newly observed space?
9. Where is the deepest reachable hiding place, concealed by obstacles, for a robot of radius 0.25 m starting at `(x1, y1)`?
10. Are there two independent collision-free routes for a robot of radius 0.20 m between `(x1, y1)` and `(x2, y2)`, so blocking one route does not block the other?
11. Which doorway-sized opening is the most important bottleneck for reaching the rest of the map from `(x1, y1)`?
12. A robot of radius 0.25 m starts at `(x1, y1)`, facing 20 degrees, and moves at 0.50 m/s while turning at 15 degrees/s. Within eight seconds, when and where does it first collide with occupied or unknown space?
13. Between 380 and 410 seconds, when does `(x2, y2)` first become reachable from `(x1, y1)` for a robot of radius 0.20 m?
14. What is the total known-free floor area?
15. A robot of radius 0.30 m is at `(x1, y1)`, facing 90 degrees. Is the full one-meter corridor directly ahead clear enough to move forward without collision?

# PoseStamped

1. How far apart are the robot's starting and ending positions?
2. How long was the robot stationary?
3. When did the robot walk backward?
4. How many times did the robot's path cross itself?
5. How much can the robot's path be simplified into straight segments?
6. Where did the robot retrace an earlier path in the opposite direction?
7. Which location did the robot return to after the longest time?
8. When was the robot facing most directly away from its direction of travel?

# Wishlist QA - moshi
