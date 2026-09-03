### Occupancy Grid

1. how many rooms are there in the space?
2. whats a m square of most movement in the space?
3. whats the shortest collision free loop path between x1,y1, x2, y2, x3y3 in any order
4. whats the largest robot radius that can travel from x1,y1 to x2, y2
5. if the opening near x1,y1 gets blocked, can robot still reach x2,y2 from x3,y3
7. where are likely doors situated in the space
8. whats largest known free circular area in the room
9. what are potential times where someone mightve walked infront of you
10. what the deepest place inthe space, if youre plaing hidenseek where do you recommend hiding
11. are there 2 independent routes between x1,y1 and x2,y2. or owuld blocking route somwhere woudl restrict both
12. Which doorway-sized opening is the most important bottleneck for reaching the rest of the mapped area from (x y)?
13. If we lock twist pos x vel, and t theta, from (x,y) - whats the point in map that its gonna collude to, or is it never gonna collide?
14. At what point in the grid sequence did (x1, y1) first become reachable from (x2, y2) entirely through known-ree space?

### Pose Stamped/ Odometry

1. how much more does it have to travel, to creturn to its inital position?
2. what percentage of the travel is robot stationary
3. during what intervals did the robto walk backwards, if it did
4. how many insances in time, did the robot intersect with its own path
5. if youd optimzie the path, making it straight lines, turns with radius, reverse, roations as such, how would the result be
6. During which interval did the robot retrace an earlier path in the opposite direction?
7. hich previously visited location did the robot return to after the longest elapsed time?
8. During which interval was the robot’s facing direction least aligned with its direction of travel?
9. Is there a repeated patrol cycle in the trajectory, and what are its approximate start time, duration, and lengh?
