function fig = open_refuge_live_twin_view(rows, status, planText, bringToFront)
%OPEN_REFUGE_LIVE_TWIN_VIEW Open and foreground the live digital twin view.
%
% This wrapper keeps the renderer testable outside the operator panel and
% forces the figure back onto the visible desktop if MATLAB creates it behind
% another window or on a stale monitor position.

rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

if nargin < 1 || isempty(rows)
    rows = struct([]);
end
if nargin < 2 || ~isstruct(status)
    status = struct();
end
if nargin < 3
    planText = "Waiting for ROS DB";
end
if nargin < 4
    bringToFront = true;
end

refuge_live_twin_render(rows, status, string(planText));
drawnow expose;

figs = findall(0, 'Type', 'figure', 'Name', 'Refuge Live Digital Twin');
if isempty(figs)
    error('open_refuge_live_twin_view:noFigure', ...
        'refuge_live_twin_render returned without creating Refuge Live Digital Twin figure.');
end

fig = figs(1);
try
    currentFig = get(0, 'CurrentFigure');
    if ~isempty(currentFig) && any(currentFig == figs)
        fig = currentFig;
    end
catch
end
try
    set(figs, 'Units', 'pixels');
    set(figs, 'Visible', 'on');
catch
end
try
    set(figs, 'WindowStyle', 'normal');
catch
end
if bringToFront
    try
        fig.WindowState = 'normal';
    catch
    end
    try
        set(fig, 'Position', liveViewerPosition(1360, 780));
    catch
    end
    try
        movegui(fig, 'west');
    catch
    end
    try
        movegui(fig, 'onscreen');
    catch
    end
    try
        set(0, 'CurrentFigure', fig);
    catch
    end
    try
        figure(fig);
    catch
    end
    try
        shg;
    catch
    end
    drawnow expose;
end
end

function pos = liveViewerPosition(w, h)
screen = get(0, 'ScreenSize');
margin = 35;
w = min(w, max(900, screen(3) - 2 * margin));
h = min(h, max(620, screen(4) - 2 * margin));
left = screen(1) + margin;
bottom = screen(2) + max(margin, screen(4) - h - margin);
pos = [left bottom w h];
end
