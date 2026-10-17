function fig_torque_versus_lever_arm()
%FIG_TORQUE_VERSUS_LEVER_ARM  Torque--lever-arm clouds for layout A.
%
%   Figure for the technical report: static gravity torque of every joint
%   against the horizontal distance of the end point from the base axis, over
%   random samples of the joint box. Replaces the default toolkit plot with a
%   single-panel, print-ready version.
%
%   Produces docs/fig/fig_torque_versus_lever_arm.png (600 dpi) and .pdf.
%   Run from the repository root:  fig_torque_versus_lever_arm

    NS = 20000;          % samples
    ARM_CUT = 0.4;       % half-arm-length envelope, m

    config = fun_read_config();
    robot = fun_build_robot(config);

    % evalc returns the captured text as its first output, then the outputs of
    % the evaluated call, which for fun_static_sampling are [q, arm, tau].
    rng(1);
    [~, q_s, arm, tau] = evalc('fun_static_sampling(robot, config.modeling.joint_lower_limit, config.modeling.joint_upper_limit, NS);');

    font = 'Times New Roman';
    colors = [0.00 0.45 0.70; 0.85 0.33 0.10; 0.47 0.67 0.19; ...
              0.49 0.18 0.56; 0.93 0.69 0.13; 0.30 0.75 0.93; 0.64 0.08 0.18];

    fig = figure('Color', 'w', 'Units', 'inches', 'Position', [1 1 6.5 2.6]);
    ax = axes('Parent', fig, 'Position', [0.085 0.20 0.895 0.76]);
    hold(ax, 'on');

    % Only a subset is drawn as a cloud: it is background context, and keeping
    % the marker count low keeps the exported vector file small.
    ip = randperm(numel(arm), min(1000, numel(arm)));
    h = cell(7, 1);
    for j = 1:7
        h{j} = scatter(ax, arm(ip), tau(ip, j), 1, colors(j, :), 'filled', ...
                       'MarkerFaceAlpha', 0.10, 'MarkerEdgeAlpha', 0.10);
    end

    % Binned maximum per joint: this is the quantity that defines the torque
    % requirement at a given lever-arm envelope (rated torque). The bins hold an
    % equal number of samples, so that the running maximum also reaches the
    % sparsely sampled postures near the kinematic limit.
    NB = 10;
    as = sort(arm);
    edges = as(round(linspace(1, numel(as), NB + 1)));
    ctr = nan(NB, 1);
    env = nan(NB, 7);
    for b = 1:NB
        m = arm >= edges(b) & arm <= edges(b + 1);
        if any(m)
            ctr(b) = median(arm(m));
            for j = 1:7
                env(b, j) = max(tau(m, j));
            end
        end
    end
    for j = 1:7
        plot(ax, ctr, env(:, j), '-', 'Color', colors(j, :), 'LineWidth', 1.0);
        plot(ax, ctr, env(:, j), '.', 'Color', colors(j, :), 'MarkerSize', 5);
    end

    yline = ARM_CUT;
    plot(ax, [yline yline], [0 60], 'k--', 'LineWidth', 0.8);
    text(ax, yline + 0.01, 57, 'half-arm envelope', 'FontName', font, ...
         'FontSize', 8, 'VerticalAlignment', 'top');

    xlabel(ax, 'horizontal lever arm of the end point (m)', 'FontName', font, 'FontSize', 9);
    ylabel(ax, 'static joint torque (N m)', 'FontName', font, 'FontSize', 9);
    xlim(ax, [0 0.85]);
    ylim(ax, [0 60]);
    box(ax, 'on');
    set(ax, 'FontName', font, 'FontSize', 8, 'TickDir', 'out', 'LineWidth', 0.6);

    lg = legend(ax, [h{:}], arrayfun(@(j) sprintf('J%d', j), 1:7, 'UniformOutput', false), ...
                'Location', 'northwest', 'NumColumns', 2, 'FontName', font, ...
                'FontSize', 8, 'Box', 'off');
    lg.ItemTokenSize = [8 8];

    here = fileparts(mfilename('fullpath'));
    exportgraphics(fig, fullfile(here, 'fig_torque_versus_lever_arm.png'), ...
                   'Resolution', 600, 'BackgroundColor', 'white');
    exportgraphics(fig, fullfile(here, 'fig_torque_versus_lever_arm.pdf'), ...
                   'ContentType', 'vector', 'BackgroundColor', 'white');
    disp(['Written: ', fullfile(here, 'fig_torque_versus_lever_arm.png')]);
end
