# Select the first feasible provider candidate

The configured Grasp Provider owns candidate ranking, and the transaction checks candidates in that order up to one maximum count. Safety and complete-sequence feasibility validation are mandatory and cannot be configured off; the first candidate with feasible preparation, approach, grasp, and retreat is selected, while rejected candidates are summarized by failed stage. Separate filter and ranking modes are removed.
