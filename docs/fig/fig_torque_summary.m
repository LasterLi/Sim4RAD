function fig_torque_summary()
%FIG_TORQUE_SUMMARY  Joint torque demand of the three layouts, three scenarios.
%
%   Figure for the technical report. For every layout, the static half-arm
%   requirement, the worst-case static torque (exhaustive posture search) and
%   the maximum torque of the shared 2 s commanded motion are recomputed and
%   shown as a grouped bar chart.
%
%   All values are recomputed by this script (no hard-coded numbers): the
%   layout overrides below are the same blocks that are commented in
%   config_modeling.yaml and config_manual_robot.yaml for the case study.
%
%   Produces docs/fig/fig_torque_summary.png (600 dpi) and .pdf.
%   Run from the repository root:  fig_torque_summary

    NS = 50000;          % static samples
    DURATION = 2;        % s, constraint mode 1 of the commanded motion

    config = fun_read_config();

    names   = {'A', 'B', 'C'};
    lower   = [-180,-45,-180,-150,-180,-45,-90; ...
               -45,-180,-180,-150,-180,-45,-90; ...
               -180,-45,-180,-150,-180,-45,-180] * pi / 180;
    upper   = [60,180,180,10,180,45,90; ...
               180,60,180,10,180,45,90; ...
               60,180,180,10,180,45,180] * pi / 180;
    loadPos = [0.1, 0, 0, 0, 1.5708, 0; ...
               0.1, 0, 0, 0, 1.5708, 0; ...
               0, 0, 0.1, 0, 0, 0];
    endCfg  = [45, -30, 40, -60, 45, 30, 45; ...
               -30, 45, 40, -60, 45, 30, 45; ...
               45, -30, 40, -60, 45, 30, 45] * pi / 180;

    % link-frame changes with respect to layout A: {layout, link, frame}
    linkOv = {2, 1, [0, 0, 0, 0, 90, 0]; ...
              2, 2, [0, 0.1, 0, -90, 0, 0]; ...
              2, 3, [0, 0, 0, 0, 90, 0]; ...
              3, 7, [0, 0, 0, 0, 90, 0]};

    halfArm = zeros(3, 7);
    worst   = zeros(3, 7);
    motion  = zeros(3, 7);

    for k = 1:3
        cfg = config;
        cfg.modeling.joint_lower_limit = lower(k, :);
        cfg.modeling.joint_upper_limit = upper(k, :);
        cfg.modeling.end_load_pos      = loadPos(k, :);
        for i = 1:size(linkOv, 1)
            if linkOv{i, 1} == k
                cfg.manual_robot.(sprintf('Link%d_pose', linkOv{i, 2})) = linkOv{i, 3};
            end
        end
        robot = fun_build_robot(cfg);

        % --- static: exhaustive worst case and half-arm sampling maximum ---
        [tau_worst, q_all, tau_all] = fun_search_static_load(robot, cfg);
        worst(k, :) = tau_worst;

        % evalc captures the printout; its first output is the text, so the
        % call is written with a leading ignored output.
        rng(1);
        [~, q_s, arm_s, tau_s] = evalc(['fun_static_sampling(robot, cfg.modeling.joint_lower_limit, ' ...
                                        'cfg.modeling.joint_upper_limit, NS);']);
        halfArm(k, :) = max(tau_s(arm_s <= 0.4, :), [], 1);

        % --- dynamics: the same commanded motion for all layouts, 2 s ---
        cd = cfg;
        cd.dynamics.trajectory_type      = 2;
        cd.dynamics.planning_space       = 'joint';
        cd.dynamics.interpolation_space  = 'joint';
        cd.dynamics.interpolation_method = 'cubic';
        cd.dynamics.constraint_mode      = 1;
        cd.dynamics.trajectory_duration  = DURATION;
        cd.dynamics.start_configuration  = zeros(1, 7);
        cd.dynamics.start_velocity       = zeros(1, 7);
        cd.dynamics.end_configuration    = endCfg(k, :);
        cd.dynamics.end_velocity         = zeros(1, 7);

        [t, q, qd, qdd] = fun_trajectory_generation(robot, cd);
        [t, q, qd, qdd] = fun_trajectory_constraint(robot, t, q, qd, qdd, cd);
        [transmission, ~] = fun_get_joint_dynamics_capability(cd.dynamics);
        [~, tau_out] = fun_trajectory_inverse_dynamics(robot, t, q, qd, qdd, ...
                                                       transmission, zeros(1, 6));
        motion(k, :) = max(abs(tau_out), [], 1);

        fprintf('layout %s: half-arm %s | worst %s | motion %s\n', names{k}, ...
                num2str(halfArm(k, :), '%.2f '), num2str(worst(k, :), '%.2f '), ...
                num2str(motion(k, :), '%.2f '));
    end

    % ------------------------------- figure ---------------------------------
    font = 'Times New Roman';
    fig = figure('Color', 'w', 'Units', 'inches', 'Position', [1 1 6.5 2.5]);

    bFirst = [];
    for k = 1:3
        ax = axes('Parent', fig, 'Position', [0.075 + (k - 1) * 0.325, 0.19, 0.275, 0.61]);
        hold(ax, 'on');
        b = bar(ax, [halfArm(k, :); worst(k, :); motion(k, :)]', 'grouped', ...
                'EdgeColor', 'k', 'LineWidth', 0.5);
        b(1).FaceColor = [0.80 0.80 0.80];
        b(2).FaceColor = [0.35 0.35 0.35];
        b(3).FaceColor = [1.00 1.00 1.00];
        if k == 1
            bFirst = b;
        end

        xlim(ax, [0.4 7.6]);
        ylim(ax, [0 62]);
        xlabel(ax, 'joint', 'FontName', font, 'FontSize', 9);
        if k == 1
            ylabel(ax, 'torque demand (N m)', 'FontName', font, 'FontSize', 9);
        end
        title(ax, sprintf('(%s) Layout %s', char('a' + k - 1), names{k}), ...
              'FontName', font, 'FontSize', 9, 'FontWeight', 'normal');
        set(ax, 'FontName', font, 'FontSize', 8, 'TickDir', 'out', 'LineWidth', 0.6, ...
            'Box', 'on');
    end

    % One shared legend, placed in the free strip above the panel titles.
    lg = legend(bFirst, {'half-arm', 'worst case', '2 s motion'}, ...
                'FontName', font, 'FontSize', 8, 'Box', 'off', ...
                'Orientation', 'horizontal');
    lg.Position = [0.29 0.90 0.42 0.09];

    here = fileparts(mfilename('fullpath'));
    exportgraphics(fig, fullfile(here, 'fig_torque_summary.png'), ...
                   'Resolution', 600, 'BackgroundColor', 'white');
    exportgraphics(fig, fullfile(here, 'fig_torque_summary.pdf'), ...
                   'ContentType', 'vector', 'BackgroundColor', 'white');
    disp(['Written: ', fullfile(here, 'fig_torque_summary.png')]);
end
