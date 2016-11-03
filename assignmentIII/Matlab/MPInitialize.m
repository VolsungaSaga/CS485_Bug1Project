function [] = MPInitialize()
global mp;
global params;

mp.xpts          = [params.robot(1)];
mp.ypts          = [params.robot(2)];
mp.parents     = [-1];
mp.nchildren   = [0];
mp.vidAtGoal  = -1;
mp.sto        = [Inf Inf];
mp.vidNear = [Inf Inf];
end

