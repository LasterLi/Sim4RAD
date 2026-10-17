function fig_design_flow()
%FIG_DESIGN_FLOW  Draw the design-flow diagram used in docs/case_study.md.
%
%   Produces two files next to this script:
%       fig_design_flow.png   600 dpi, for the Markdown document
%       fig_design_flow.pdf   vector, for inclusion in the LaTeX report
%
%   The figure is drawn at its final printed size (6.5 x 1.8 in), so it can be
%   included in LaTeX with width=\linewidth without rescaling the fonts.
%   Explanations are deliberately kept out of the figure and belong to the
%   caption.
%
%   Run from the repository root:
%       fig_design_flow

    W = 6.5;   % figure width, inches (= \linewidth of the report)
    H = 1.8;   % figure height, inches

    fig = figure('Color', 'w', 'Units', 'inches', 'Position', [1 1 W H]);
    ax = axes('Parent', fig, 'Position', [0 0 1 1], 'XLim', [0 W], 'YLim', [0 H], ...
              'Visible', 'off');
    axes(ax);
    hold on;

    font = 'Times New Roman';
    cShade = [0.90 0.90 0.90];

    nrm = @(x, y) [x / W, y / H];   % data coordinates -> normalized

    %% Steps
    bw = 1.30;  % box width
    bh = 0.62;  % box height
    yb = 0.85;  % bottom edge of the boxes
    x0 = [0.05 1.75 3.45 5.15];
    shaded = [false true false true];

    labels = {{'1  Requirements'}, ...
              {'2  Scheme simulation', '& optimization'}, ...
              {'3  Detailed design'}, ...
              {'4  Verification'}};

    for i = 1:4
        if shaded(i)
            fc = cShade;
        else
            fc = 'w';
        end
        rectangle('Position', [x0(i), yb, bw, bh], 'FaceColor', fc, ...
                  'EdgeColor', 'k', 'LineWidth', 0.8);
        text(x0(i) + bw / 2, yb + bh / 2, labels{i}, ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'FontName', font, 'FontSize', 9, 'Color', 'k');
    end

    %% Hand-off between consecutive steps
    yA = yb + bh / 2;
    handOff = {'goals', 'specs', 'URDF'};
    for i = 1:3
        p1 = nrm(x0(i) + bw + 0.04, yA);
        p2 = nrm(x0(i + 1) - 0.04, yA);
        annotation('arrow', [p1(1) p2(1)], [p1(2) p2(2)], 'Color', 'k', ...
                   'LineWidth', 0.8, 'HeadStyle', 'vback2', ...
                   'HeadWidth', 5, 'HeadLength', 5);
        text(x0(i) + bw + 0.20, yA - 0.16, handOff{i}, ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'FontName', font, 'FontSize', 8, 'Color', 'k');
    end

    %% Iteration: verification may send the design back to detailed design
    yF = 0.40;
    xRight = x0(4) + bw / 2;
    xLeft  = x0(3) + bw / 2;
    a = nrm(xRight, yb);
    b = nrm(xRight, yF);
    c = nrm(xLeft, yF);
    d = nrm(xLeft, yb);
    annotation('line',  [a(1) b(1)], [a(2) b(2)], 'Color', 'k', 'LineWidth', 0.8, 'LineStyle', '--');
    annotation('line',  [b(1) c(1)], [b(2) c(2)], 'Color', 'k', 'LineWidth', 0.8, 'LineStyle', '--');
    annotation('arrow', [c(1) d(1)], [c(2) d(2)], 'Color', 'k', 'LineWidth', 0.8, ...
               'LineStyle', '--', 'HeadStyle', 'vback2', 'HeadWidth', 5, 'HeadLength', 5);
    text((xRight + xLeft) / 2, yF + 0.05, 're-design', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
         'FontName', font, 'FontSize', 8, 'FontAngle', 'italic', 'Color', 'k');

    %% Legend
    rectangle('Position', [0.05, 0.13, 0.16, 0.11], 'FaceColor', cShade, ...
              'EdgeColor', 'k', 'LineWidth', 0.8);
    text(0.27, 0.185, 'Steps implemented in Sim4RAD', ...
         'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
         'FontName', font, 'FontSize', 8, 'Color', 'k');

    %% Export
    drawnow;
    here = fileparts(mfilename('fullpath'));
    exportgraphics(fig, fullfile(here, 'fig_design_flow.png'), ...
                   'Resolution', 600, 'BackgroundColor', 'white');
    exportgraphics(fig, fullfile(here, 'fig_design_flow.pdf'), ...
                   'ContentType', 'vector', 'BackgroundColor', 'white');
    disp(['Written: ', fullfile(here, 'fig_design_flow.png')]);
    disp(['Written: ', fullfile(here, 'fig_design_flow.pdf')]);
end
