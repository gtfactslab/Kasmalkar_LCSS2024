clear
close all
clc

% control_effort pos_err yaw_err roll_osc_rms pitch_osc_rms solve_time
disp("Control effort, Pos err, Yaw err, Roll, Pitch, Solve time")


figure

%% Line follower

line_cbf_1 = 'logs/final_testing_4_25/cbf_line.csv';
line_cbf_2 = 'logs/final_testing_4_25/cbf_line2.csv';
line_cbf_3 = 'logs/final_testing_4_25/cbf_line3.csv';

line_lqr_1 = 'logs/final_testing_4_22/lqr_line.csv';
line_lqr_2 = 'logs/final_testing_4_22/lqr_line2.csv';
line_lqr_3 = 'logs/final_testing_4_22/lqr_line3.csv';

line_cbf_1_metrics = compute_metrics(line_cbf_1);
line_cbf_2_metrics = compute_metrics(line_cbf_2);
line_cbf_3_metrics = compute_metrics(line_cbf_3);
line_cbf_metrics = [line_cbf_1_metrics
                    line_cbf_2_metrics
                    line_cbf_3_metrics];

line_cbf_metrics_means = mean(line_cbf_metrics, 1)

line_lqr_1_metrics = compute_metrics(line_lqr_1);
line_lqr_2_metrics = compute_metrics(line_lqr_2);
line_lqr_3_metrics = compute_metrics(line_lqr_3);
line_lqr_metrics = [line_lqr_1_metrics
                    line_lqr_2_metrics
                    line_lqr_3_metrics];

line_lqr_metrics_means = mean(line_lqr_metrics, 1)

% plot_data(line_cbf_1, 60, 'Line Trajectory')
% plot_data(line_cbf_2, 60, 'Line Trajectory', 'opengl')
% plot_data(line_cbf_3, 60, 'Line Trajectory')

%% Helix follower

helix_cbf_1 = 'logs/final_testing_4_25/cbf_helix.csv';
helix_cbf_2 = 'logs/final_testing_4_25/cbf_helix2.csv';
helix_cbf_3 = 'logs/final_testing_4_25/cbf_helix3.csv';

helix_lqr_1 = 'logs/final_testing_4_22/lqr_helix.csv';
helix_lqr_2 = 'logs/final_testing_4_22/lqr_helix2.csv';
helix_lqr_3 = 'logs/final_testing_4_22/lqr_helix3.csv';

helix_cbf_1_metrics = compute_metrics(helix_cbf_1);
helix_cbf_2_metrics = compute_metrics(helix_cbf_2);
helix_cbf_3_metrics = compute_metrics(helix_cbf_3);
helix_cbf_metrics = [helix_cbf_1_metrics
                    helix_cbf_2_metrics
                    helix_cbf_3_metrics];

helix_cbf_metrics_means = mean(helix_cbf_metrics, 1)

helix_lqr_1_metrics = compute_metrics(helix_lqr_1);
helix_lqr_2_metrics = compute_metrics(helix_lqr_2);
helix_lqr_3_metrics = compute_metrics(helix_lqr_3);
helix_lqr_metrics = [helix_lqr_1_metrics
                    helix_lqr_2_metrics
                    helix_lqr_3_metrics];

helix_lqr_metrics_means = mean(helix_lqr_metrics, 1)

plot_data(helix_cbf_1, 60, 'Helix Trajectory', 'opengl', 1)
% plot_data(helix_cbf_2, 60, 'Helix Trajectory')
% plot_data(helix_cbf_3, 60, 'Helix Trajectory')

%% Triangle follower

triangle_cbf_1 = 'logs/final_testing_4_25/cbf_triangle.csv';
triangle_cbf_2 = 'logs/final_testing_4_25/cbf_triangle2.csv';
triangle_cbf_3 = 'logs/final_testing_4_25/cbf_triangle3.csv';

triangle_lqr_1 = 'logs/final_testing_4_22/lqr_triangle.csv';
triangle_lqr_2 = 'logs/final_testing_4_22/lqr_triangle2.csv';
triangle_lqr_3 = 'logs/final_testing_4_22/lqr_triangle3.csv';

triangle_cbf_1_metrics = compute_metrics(triangle_cbf_1);
triangle_cbf_2_metrics = compute_metrics(triangle_cbf_2);
triangle_cbf_3_metrics = compute_metrics(triangle_cbf_3);
triangle_cbf_metrics = [triangle_cbf_1_metrics
                    triangle_cbf_2_metrics
                    triangle_cbf_3_metrics];

triangle_cbf_metrics_means = mean(triangle_cbf_metrics, 1)

triangle_lqr_1_metrics = compute_metrics(triangle_lqr_1);
triangle_lqr_2_metrics = compute_metrics(triangle_lqr_2);
triangle_lqr_3_metrics = compute_metrics(triangle_lqr_3);
triangle_lqr_metrics = [triangle_lqr_1_metrics
                    triangle_lqr_2_metrics
                    triangle_lqr_3_metrics];

triangle_lqr_metrics_means = mean(triangle_lqr_metrics, 1)

plot_data(triangle_cbf_1, 180, 'Sawtooth Trajectory', 'painters', 3)
% plot_data(triangle_cbf_2, 180, 'Sawtooth Trajectory')
% plot_data(triangle_cbf_3, 180, 'Sawtooth Trajectory')

%% Raw FBL
fbl_line = compute_metrics("logs/fbl_testing_5_2/fbl_line.csv")
fbl_helix = compute_metrics("logs/fbl_testing_5_2/fbl_helix.csv")
fbl_triangle = compute_metrics("logs/fbl_testing_5_2/fbl_triangle.csv")

%% CBFs vs FBL

figure

plot_cbf_vs_fbl("logs/cbf_vs_fbl_data_4_27/fbl_line.csv", ...
                 ["logs/cbf_vs_fbl_data_4_27/cbf_line_75deg.csv"], ...
                 [7.5], ...
                 "Line", ...
                 20, ...
                 12, ...
                 1)
disp(" ")
plot_cbf_vs_fbl("logs/cbf_vs_fbl_data_4_27/fbl_triangle.csv", ...
                ["logs/cbf_vs_fbl_data_4_27/cbf_triangle_10deg.csv"], ...
                 [10], ...
                 "Sawtooth", ...
                 25, ...
                 30, ...
                 3)

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

function metrics = compute_metrics(file)
    
    [t, x, xd, u, ref, err, st] = load_data(file);

    control_effort = 0;
    for i = 1:length(u)
        control_effort = control_effort + norm(u(i, :));
    end
    control_effort = control_effort / length(u);

    pos_err = 0;
    yaw_err = 0;
    for i = 1:length(err)
        pos_err = pos_err + norm(err(i, 1:3));
        yaw_err = yaw_err + norm(err(i, 4));
    end
    pos_err = pos_err / length(err);
    yaw_err = yaw_err / length(err) * 180/pi;

    roll_osc_max = max(abs(x(:, 10)))*180/pi;
    pitch_osc_max = max(abs(x(:, 11)))*180/pi;

    solve_time = mean(st)/1e9;
    
    metrics = [control_effort pos_err yaw_err roll_osc_max pitch_osc_max solve_time];
end

function plot_data(cbf_file, time_end, plot_title, renderer, sp_start)
    [t, cbf_x, ~, ~, cbf_ref, cbf_err, ~] = load_data(cbf_file);

    % Trajectory tracking
    subplot(2, 2, sp_start)

    colororder([rgb2hex([0    0.4470    0.7410]), ...
                rgb2hex([0.8500    0.3250    0.0980]), ...
                rgb2hex([0.9290    0.6940    0.1250])])
    plot(t, cbf_x(:, 7:9), 'LineWidth', 2, 'LineStyle', '-')
    hold on
    plot(t, cbf_ref(:, 1:3), 'LineWidth', 2, 'LineStyle', ':')

    xlim([0 time_end])
    xlabel('Time (sec)')
    ylabel('Position (m)')
    legend('x', 'y', 'z', ...
           'x_{ref}', 'y_{ref}','z_{ref}')
    title(plot_title + ": Position tracking")
    pbaspect([1 1 1])

    % Attitude oscillations
    % subplot(132)
    % plot(t, cbf_x(:, 10) * 180/pi, 'LineWidth', 0.75, 'LineStyle', '-', 'Color', '#D95319')
    % hold on
    % plot(t, cbf_x(:, 11) * 180/pi, 'LineWidth', 0.75, 'LineStyle', '-', 'Color', '#7E2F8E')
    % plot(t, (cbf_x(:, 12) - cbf_ref(:, 4)) * 180/pi, 'LineWidth', 1, 'LineStyle', '-', 'Color', '#77AC30')
    % yline(5, 'LineStyle', '--', 'LineWidth', 2)
    % yline(-5, 'LineStyle', '--', 'LineWidth', 2)
    % 
    % ylim([-10 10])
    % xlabel('Time (sec)')
    % ylabel('Angle (deg)')
    % title(plot_title + ": Roll, Pitch, and Yaw")
    % xlim([0 time_end])
    % legend('phi', 'theta', 'psi error')
    % 
    % set(gcf, 'color', 'white')
    % pbaspect([1 1 1])

    % 3D trajectory plot
    subplot(2, 2, sp_start+1)
    plot3(cbf_x(:, 7), cbf_x(:, 8), cbf_x(:, 9), 'LineWidth', 2.5, 'Color', [0.1490    0.5490    0.8660], 'LineStyle', '-');
    hold on
    plot3(cbf_ref(:, 1), cbf_ref(:, 2), cbf_ref(:, 3), 'LineWidth', 2.5, 'Color', [0.9600    0.4660    0.1600], 'LineStyle', '-');
    
    title(plot_title)
    xlabel('x')
    ylabel('y')
    zlabel('z')
    pbaspect([1 1 1])
    margin = 0.2;
    xlim([min(cbf_x(:,7))-margin max(cbf_x(:, 7))+margin])
    ylim([min(cbf_x(:, 8))-margin max(cbf_x(:, 8))+margin])
    zlim([min(cbf_x(:, 9))-margin max(cbf_x(:, 9))+margin])
    set(gca, 'ydir', 'reverse')
    set(gca, 'zdir', 'reverse')
    set(gcf, 'color', 'white')

    legend('FBL+CBF', 'Reference')
    set(gcf, 'Renderer', renderer)
end

function plot_cbf_vs_fbl(fbl_file, cbf_files, cbfs, name, ylims, time_end, sp_start)
    colororder(["b", "g", "m"])
    
    N = 1 + length(cbf_files);

    [tt, fbl_x, ~, ~, ~, ~, ~] = load_data(fbl_file);
    
    subplot(2, 2, sp_start)
    plot(tt, fbl_x(:, 10) * 180/pi, 'Color', [0.1490    0.5490    0.8660], 'LineWidth', 2)
    hold on
    plot(tt, fbl_x(:, 11) * 180/pi, 'Color', [0.9600    0.4660    0.1600], 'LineWidth', 2)
    xlim([0 time_end])
    ylim([-ylims ylims])
    xlabel('Time')
    ylabel('Angle (deg)')
    title(name + " Trajectory Without CBFs")
    legend('Roll', 'Pitch')
    pbaspect([1 1 1])

    disp(name + " trajectory CBF vs FBL:")
    disp("FBL:")
    metrics = compute_metrics(fbl_file);
    fbl_pos_err = metrics(2);
    fbl_yaw_err = metrics(3);
    disp("Pos error: " + fbl_pos_err + ", yaw error: " + fbl_yaw_err)
    disp(" ")

    for n = 1:length(cbf_files)
        subplot(2, 2, sp_start+n)
        [tt, cbf_x, ~, ~, ~, ~, ~] = load_data(cbf_files(n));
        plot(tt, cbf_x(:, 10) * 180/pi, 'Color', [0.1490    0.5490    0.8660], 'LineWidth', 2)
        hold on
        plot(tt, cbf_x(:, 11) * 180/pi, 'Color', [0.9600    0.4660    0.1600], 'LineWidth', 2)
        xlim([0 time_end])
        yline(cbfs(n), 'LineWidth', 3, 'LineStyle', '--')
        yline(-cbfs(n), 'LineWidth', 3, 'LineStyle', '--')
        ylim([-ylims ylims])
        xlabel('Time')
        ylabel('Angle (deg)')
        title(name + " Trajectory With " + cbfs(n) + "° CBF")
        legend('Roll', 'Pitch')
        pbaspect([1 1 1])

        disp("CBF = " + cbfs(n) + "°")
        metrics = compute_metrics(cbf_files(n));
        cbf_pos_err = metrics(2);
        cbf_yaw_err = metrics(3);
        disp("Pos error: " + cbf_pos_err + ", yaw error: " + cbf_yaw_err)
        disp(" ")
    end

    set(gcf, 'color', 'white')
end
