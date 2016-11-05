function status  = IsValidState()
  global params;
  status = params.robot(1) >= params.xmin && params.robot(1) <= params.xmax && ...
                params.robot(2) >= params.ymin && params.robot(2) <= params.ymax;
   if status == 1
       rx = params.robot(1);
       ry = params.robot(2);
       rr  = params.robot(3);
       n = length(params.obstacles);
       for k = 1 : 3 : n
           ox = params.obstacles(k);
           oy = params.obstacles(k + 1);
           if ((rx - ox) * (rx - ox) + (ry  - oy) * (ry - oy)) < (rr + params.obstacles(k + 2))^2
               status = 0;
               return;
           end
       end
   end
end

