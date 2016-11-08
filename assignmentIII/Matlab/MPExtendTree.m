function [] = MPExtendTree(vid, sto)
% Extend the tree from the state with index vid toward the state sto
% At each time, make a small step with magnitude params.distOneStep
% Add each intermediate valid state to the tree data structure.
% Stop as soon as an intermediate invalid state is encountered
%   - You can check state validity by first setting the robot position and
%   - then calling the function IsStateValid
% Also stop if an intermediate state reaches the goal
%   - You can check if robot has reached the goal by first setting the
%   - robot position and then calling the function HasRobotReachedGoal

global mp;
global params;

end

