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
15. whats the total area in m2
16. is the space infront of you clear or occuleded at (x,y)

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

### Filtered QA Questions

1. What stretches of mapped floor have a clear passage width above 0.9 m?
2. How many people are seated at a desk?
3. When and where was the backpack last observed?
4. Where are the bottles?
5. Does every desk have a monitor on it?
6. Search until a bottle is located and return its position with a candidate frame.
7. Which named region held the most people at any point in the recording?
8. How many people were standing rather than seated, per region?
9. Which regions contain a trash can?
10. Are there boxes in the region containing the flag?
11. Navigate to the nearest region that is both free space and unoccupied.
12. Re-inspect the region containing the oven at fixed intervals and report what changed.
13. Report every interval in which a chosen region held no people for longer than 60 s.
14. Report the timestamp at which a chosen region transitions from occupied to empty.
15. Is the traffic cone in the same position at the end of the recording as at the start?
16. Report any interval in which three or more people stand within 2 m of each other.
17. Is the oven in an active or idle state?
18. Which 30-second bucket contains the most person-entry events?
