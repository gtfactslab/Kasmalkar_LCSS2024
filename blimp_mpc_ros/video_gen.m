clear
close all
clc

play_video('logs/videos_5_1/cbf_triangle_video', 1, 180)

function play_video(filename, speed, time_end)
    [tt, x, xd, u, ref, err, st] = load_data(filename + ".csv");

    margin = 0.2;
    dT = 0.01;

    iterations = floor(time_end/dT);

    first_iteration = 1;

    for t = 1:iterations
        cla
        
        disp(t + " / " + iterations)
        plot3(ref(:, 1), ref(:, 2), ref(:, 3), 'LineWidth', 2.5, 'Color', [0.9600    0.4660    0.1600], 'LineStyle', '-');
        hold on
        plot3(x(1:t, 7), x(1:t, 8), x(1:t, 9), 'LineWidth', 2.5, 'Color', [0.1490    0.5490    0.8660]);
        scatter3(x(t, 7), x(t, 8), x(t, 9), 'LineWidth', 5, 'Color', [0.9290    0.6940    0.1250]);
        
        xlabel('x')
        ylabel('y')
        zlabel('z')
        xlim([min(x(:, 7))-margin, max(x(:, 7))+margin])
        ylim([min(x(:, 8))-margin, max(x(:, 8))+margin])
        zlim([min(x(:, 9))-margin, max(x(:, 9))+margin])
    
        set(gcf, 'color', 'white')
        set(gca, 'ydir', 'reverse')
        set(gca, 'zdir', 'reverse')
        % set(gcf, 'renderer', 'painters')
        drawnow
    
        pbaspect([1 1 1])
    
        if first_iteration
            input("");
            first_iteration = 0;
        end
    end
end

function [time, ...
          state, ...
          state_dot, ...
          u, ...
          ref, ...
          error, ...
          solve_time] ...
          = load_data(file)
    dataset = readmatrix(file);
    time = dataset(:, 1);
    state = dataset(:, 2:13);
    state_dot = dataset(:, 14:25);
    u = dataset(:, 26:29);
    ref = dataset(:, 30:33);
    error = ref - state(:, [7 8 9 12]);
    solve_time = dataset(:, 38);
end