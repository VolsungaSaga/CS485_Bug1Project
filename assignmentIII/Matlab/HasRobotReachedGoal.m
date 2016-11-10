function status  = HasRobotReachedGoal()
  global params;
  rx = params.robot(1);
  ry = params.robot(2);
  gx = params.goal(1);
  gy = params.goal(2);
  status = (rx - gx) * (rx - gx) + (ry - gy) * (ry - gy) <= params.goal(3) * params.goal(3);
end

