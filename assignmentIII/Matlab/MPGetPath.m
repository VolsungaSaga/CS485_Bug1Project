function [xpts, ypts] = MPGetPath()
global mp;

xpts = [];
ypts = [];
vid   = mp.vidAtGoal;
while 1
    xpts = [mp.xpts(vid), xpts];
    ypts = [mp.ypts(vid), ypts];
    vid   = mp.parents(vid);
    if vid <= 0
        return;
    end
end

end

