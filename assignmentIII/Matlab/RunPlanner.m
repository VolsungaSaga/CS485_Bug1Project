function [] = RunPlanner(fname)
global params;
global mp;

SetupFromFile(fname);
MPInitialize();

selectedCircle  = -2;
mode             = 0;
shouldRun      = 0;
method          = 1;
xptsPath         = [];
yptsPath         = [];
done              = 0;
totalSolveTime= 0;

dthetas    = 0 : 2 * pi / 15 : 2 * pi;
xptsCircle = cos(dthetas);
yptsCircle = sin(dthetas);

    function [] = Draw()
        clf;
        figure(1);
        hold on; grid on;
        set(gcf, 'Renderer', 'painters');
        set(gcf,'DoubleBuffer','on');
        set(gca, 'xlim', [params.xmin - 0.5, params.xmax + 0.5]);
        set(gca, 'ylim', [params.ymin - 0.5, params.ymax + 0.5]);
        fill(params.goal(1) + params.goal(3) * xptsCircle, ...
            params.goal(2) + params.goal(3) * yptsCircle, [0 1 0]);
        n = length(params.obstacles);
        for i = 1 : 3 : n
            fill(params.obstacles(i)        + params.obstacles(i + 2) * xptsCircle, ...
                params.obstacles(i + 1)  + params.obstacles(i + 2) * yptsCircle,  [0 0 1]);
        end
        plot(mp.xpts, mp.ypts, 'gs', 'LineWidth', 1);
        n = length(mp.xpts);
        for k = 2 : 1 : n
            plot([mp.xpts(k), mp.xpts(mp.parents(k))], [mp.ypts(k), mp.ypts(mp.parents(k))], 'Color', [0 0 0]);
        end
        fill(params.robot(1) + params.robot(3) * xptsCircle, ...
            params.robot(2) + params.robot(3) * yptsCircle, [1 0 0]);
        
        plot(xptsPath, yptsPath, '--bo', 'LineWidth', 2, 'MarkerFaceColor', 'r');
        
        plot(mp.sto(1), mp.sto(2), 'ko', 'MarkerSize', 7, 'LineWidth', 6);
        text(mp.sto(1), mp.sto(2), 'rand', 'FontSize', 14, 'Color', [1 0 1], 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
        
        plot(mp.vidNear(1), mp.vidNear(2), 'ko', 'MarkerSize', 7, 'LineWidth', 6);
        text(mp.vidNear(1), mp.vidNear(2), 'near', 'FontSize', 14, 'Color', [0.3 0.2 0.6], 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
     
         text(params.robot(1), params.robot(2), 'new', 'FontSize', 14, 'Color', [0.3 0.2 0.6], 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
    
        drawnow;
    end

    function [] = MyBtnUpFcn(varargin)
        selectedCircle = -2;
    end

    function [] = MyBtnDownFcn(varargin)
        if length(mp.xpts) > 2
            return;
        end
        
        cp  = get(gca, 'CurrentPoint');
        selectedCircle = -2;
        if norm([cp(1, 1) - params.robot(1), cp(1, 2) - params.robot(2)]) <= params.robot(3)
            selectedCircle = 0;
        elseif norm([cp(1, 1) - params.goal(1), cp(1, 2) - params.goal(2)]) <= params.goal(3)
            selectedCircle = -1;
        else
            n = length(params.obstacles);
            for k = 1 : 3 : n
                if norm([cp(1, 1) - params.obstacles(k), cp(1, 2) - params.obstacles(k + 1)]) <= params.obstacles(k + 2)
                    selectedCircle = 1 + fix(k/3);
                    return;
                end
            end
        end
        if selectedCircle < -1
            params.obstacles = [params.obstacles, cp(1, 1), cp(1, 2), 1.0];
        end
    end

    function [] = MyBtnMotionFcn(varargin)
        if length(mp.xpts) > 2
            return;
        end
        
        if selectedCircle >= -1
            cp = get(gca, 'CurrentPoint');
            if selectedCircle == -1
                if mode == 1
                    params.goal(3) = norm([cp(1, 1) - params.goal(1), cp(1, 2) - params.goal(2)]);
                else
                    params.goal(1) = cp(1, 1);
                    params.goal(2) = cp(1, 2);
                end
            elseif selectedCircle == 0
                if mode == 1
                    params.robot(3) = norm([cp(1, 1) - params.robot(1), cp(1, 2) - params.robot(2)]);
                else
                    params.robot(1) = cp(1, 1);
                    params.robot(2) = cp(1, 2);
                end
            else
                i = 3 * selectedCircle;
                if mode == 1
                    params.obstacles(i) = norm([cp(1, 1) - params.obstacles(i - 2), cp(1, 2) - params.goal(i - 1)]);
                else
                    params.obstacles(i - 2) = cp(1, 1);
                    params.obstacles(i - 1) = cp(1, 2);
                end
            end
            i = 3 * selectedCircle;
        end
    end

    function [] = MyKeyPressFcn(varargin)
        key = get(gcf, 'CurrentCharacter');
        if key == 'r'
            mode = ~mode;
        elseif key == 'p'
            shouldRun = ~shouldRun;
        elseif key >= '1' && key <= '5'
            shouldRun = 1;
            method     = key - '0' + 0;
        elseif key == 27
            done = 1;
        end
    end

Draw();
set(gcf, 'windowbuttondownfcn', {@MyBtnDownFcn});
set(gcf, 'windowbuttonupfcn', {@MyBtnUpFcn});
set(gcf, 'windowbuttonmotionfcn', {@MyBtnMotionFcn});
set(gcf, 'KeyPressFcn', {@MyKeyPressFcn});

while ~done
    if shouldRun
        k = 1;
        tstart = cputime;
        while k < 2 && mp.vidAtGoal <= 0
            if method == 1
                MPExtendRandom();
            elseif method == 2
                MPExtendRRT();
            elseif method == 3
                MPExtendEST();
            elseif method == 4
                MPExtendMyApproach();
            elseif method == 5
                MPExtendESTGrid();
            end
            k = k + 1;
        end
        elapsed            = cputime - tstart;
        totalSolveTime = totalSolveTime + elapsed;
        if elapsed > 0.05
            fprintf('TotalSolveTime = %f [Solved = %d] [NrVertices = %d]\n', totalSolveTime, mp.vidAtGoal >= 1, length(mp.xpts));
        end
        
        if length(xptsPath) == 0 && mp.vidAtGoal >= 1
            [xptsPath, yptsPath]  = MPGetPath();
        end
        
    end
    Draw();
    pause(0.5);
end
fprintf('TotalSolveTime = %f [Solved = %d] [NrVertices = %d]\n', totalSolveTime, mp.vidAtGoal >= 1, length(mp.xpts));
end




