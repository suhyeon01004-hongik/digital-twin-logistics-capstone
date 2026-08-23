function refuge_live_twin_render(rows, status, planText)
%REFUGE_LIVE_TWIN_RENDER Direct live renderer for the physical refuge twin.
%
% Draws the physical DB on top of the same geometry used by the original
% parcel_manual MATLAB simulation.  The renderer intentionally does not call
% parcel_manual_core_step so DB resync cannot delete/reseed boxes while the
% digital twin view is running.

persistent fig ax3d axF axCamera state

if nargin == 0
    rows = struct([]);
end
if nargin < 2 || ~isstruct(status)
    status = struct();
end
if nargin < 3
    planText = "";
end
planText = string(planText);

g = geom();
if isempty(fig) || ~isvalid(fig) || isempty(ax3d) || ~isvalid(ax3d)
    [fig, ax3d, axF, axCamera, state] = initFigure(g);
end

rows = sortRows(rows);
targetId = round(numberField(status, 'target', 0));
pending = struct();
if isfield(status, 'pending_move') && isstruct(status.pending_move)
    pending = status.pending_move;
end
activeBelt = dbBeltToDisplay(numberField(pending, 'belt', NaN));

updateActiveBelts(state, activeBelt);
state = updatePackages(ax3d, axF, state, rows, targetId, g);
updateMachineActors(state, status, g);
state = updateUnloadActors(ax3d, axF, state, status, g);
updateStatusOverlay(ax3d, axCamera, state, status, planText, targetId, rows);
drawnow limitrate nocallbacks;
end

function [fig, ax3d, axF, axCamera, state] = initFigure(g)
fig = figure('Name', 'Refuge Live Digital Twin', ...
    'Color', [1 1 1], ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'Visible', 'on', ...
    'WindowStyle', 'normal', ...
    'NumberTitle', 'off', ...
    'Position', visiblePosition(1360, 780));

ax3d = axes(fig, 'Position', [0.040 0.150 0.555 0.800]);
axF = gobjects(3,1);
axF(3) = axes(fig, 'Position', [0.625 0.705 0.335 0.245]);
axF(2) = axes(fig, 'Position', [0.625 0.430 0.335 0.245]);
axF(1) = axes(fig, 'Position', [0.625 0.155 0.335 0.245]);
axCamera = axes(fig, 'Position', [0.625 0.025 0.335 0.100]);

configure3DAxis(ax3d, g);
for floorNo = 1:3
    configure2DAxis(axF(floorNo), floorNo, g);
end
configureCameraAxis(axCamera);

state = struct();
state.belt3d = gobjects(3,4);
state.belt2d = cell(3,1);
state.gap3d = gobjects(3,4);
state.gap2d = cell(3,1);
state.pkg3d = gobjects(0,1);
state.pkg3dText = gobjects(0,1);
state.pkg2d = cell(3,1);
state.pkg2dText = cell(3,1);
state.platform3d = patch(ax3d, 'Vertices', nan(8,3), ...
    'Faces', cuboidFaces(), 'FaceColor', [0.45 0.65 0.95], ...
    'EdgeColor', [0.10 0.28 0.65], 'LineWidth', 1.4, ...
    'FaceAlpha', 0.80, 'Visible', 'off');
state.platform2d = gobjects(3,1);
state.pusher3d = patch(ax3d, 'Vertices', nan(8,3), ...
    'Faces', cuboidFaces(), 'FaceColor', [0.18 0.22 0.26], ...
    'EdgeColor', [0.04 0.05 0.06], 'LineWidth', 1.0, ...
    'FaceAlpha', 0.95, 'Visible', 'off');
state.sidePusher3d = patch(ax3d, 'Vertices', nan(8,3), ...
    'Faces', cuboidFaces(), 'FaceColor', [0.60 0.08 0.08], ...
    'EdgeColor', [0.25 0.00 0.00], 'LineWidth', 1.0, ...
    'FaceAlpha', 0.95, 'Visible', 'off');
state.platformBox3d = patch(ax3d, 'Vertices', nan(8,3), ...
    'Faces', cuboidFaces(), 'FaceColor', [0.18 0.62 0.85], ...
    'EdgeColor', [0.02 0.12 0.18], 'LineWidth', 1.4, ...
    'FaceAlpha', 0.94, 'Visible', 'off');
state.platformBox3dText = text(ax3d, NaN, NaN, NaN, '', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontWeight', 'bold', 'Color', [0 0 0], 'Visible', 'off', 'Clipping', 'on');
state.pusher2d = gobjects(3,1);
state.sidePusher2d = gobjects(3,1);
state.platformBox2d = gobjects(3,1);
state.platformBox2dText = gobjects(3,1);
state.unload3d = gobjects(0,1);
state.unload3dText = gobjects(0,1);
state.unload2d = cell(3,1);
state.unload2dText = cell(3,1);
state.tofText = gobjects(0,1);
state.infoText = text(ax3d, g.infoX, g.infoY, g.infoZ, '', ...
    'FontWeight', 'bold', 'FontSize', 10, 'Color', [0.04 0.10 0.20], ...
    'Interpreter', 'none', 'Clipping', 'off');
state.faultText = text(ax3d, g.infoX, g.infoY, g.infoZ + 0.10, '', ...
    'FontWeight', 'bold', 'FontSize', 12, 'Color', [0.80 0.00 0.00], ...
    'Visible', 'off', 'Interpreter', 'none', 'Clipping', 'off');
state.cameraBox = patch(axCamera, NaN, NaN, [0.95 0.46 0.12], ...
    'EdgeColor', [0.00 0.95 0.25], 'LineWidth', 2.5);
state.cameraText = text(axCamera, 0, 0, '-', ...
    'Color', [1 1 1], 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

for floorNo = 1:3
    [state.belt3d(floorNo,:), state.gap3d(floorNo,:)] = draw3DFloorBase(ax3d, floorNo, g);
    [state.belt2d{floorNo}, state.gap2d{floorNo}] = draw2DFloorBase(axF(floorNo), floorNo, g);
    state.pkg2d{floorNo} = gobjects(0,1);
    state.pkg2dText{floorNo} = gobjects(0,1);
    state.unload2d{floorNo} = gobjects(0,1);
    state.unload2dText{floorNo} = gobjects(0,1);
    state.platform2d(floorNo) = patch(axF(floorNo), NaN, NaN, [0.45 0.65 0.95], ...
        'EdgeColor', [0.10 0.28 0.65], 'LineWidth', 1.4, ...
        'FaceAlpha', 0.75, 'Visible', 'off');
    state.pusher2d(floorNo) = patch(axF(floorNo), NaN, NaN, [0.18 0.22 0.26], ...
        'EdgeColor', [0.04 0.05 0.06], 'LineWidth', 1.0, ...
        'FaceAlpha', 0.95, 'Visible', 'off');
    state.sidePusher2d(floorNo) = patch(axF(floorNo), NaN, NaN, [0.60 0.08 0.08], ...
        'EdgeColor', [0.25 0.00 0.00], 'LineWidth', 1.0, ...
        'FaceAlpha', 0.95, 'Visible', 'off');
    state.platformBox2d(floorNo) = patch(axF(floorNo), NaN, NaN, [0.18 0.62 0.85], ...
        'EdgeColor', [0.02 0.12 0.18], 'LineWidth', 1.4, ...
        'FaceAlpha', 0.94, 'Visible', 'off');
    state.platformBox2dText(floorNo) = text(axF(floorNo), NaN, NaN, '', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontWeight', 'bold', 'Color', [0 0 0], 'Visible', 'off', 'Clipping', 'on');
end

end

function configure3DAxis(ax, g)
hold(ax, 'on');
axis(ax, 'equal');
axis(ax, [g.xMin3 g.xMax3 g.yMin3 g.yMax3 g.zMin g.zMax3]);
grid(ax, 'on');
ax.GridAlpha = 0.15;
ax.Color = [0.98 0.98 0.98];
ax.SortMethod = 'depth';
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
zlabel(ax, 'height [m]');
title(ax, 'Live Digital Twin 3D', 'FontWeight', 'bold');
view(ax, 38, 24);
try
    ax.Toolbar.Visible = 'off';
    disableDefaultInteractivity(ax);
catch
end
end

function configure2DAxis(ax, floorNo, g)
hold(ax, 'on');
axis(ax, 'equal');
axis(ax, [g.xMin2 g.xMax2 g.yMin2 g.yMax2]);
ax.Color = [1 1 1];
ax.XTick = [];
ax.YTick = [];
box(ax, 'on');
title(ax, sprintf('Floor %d 2D Belt State', floorNo), 'FontWeight', 'bold');
try
    ax.Toolbar.Visible = 'off';
    disableDefaultInteractivity(ax);
catch
end
end

function configureCameraAxis(ax)
hold(ax, 'on');
axis(ax, [-0.14 0.14 -0.055 0.055]);
set(ax, 'YDir', 'reverse');
ax.Color = [0.06 0.08 0.10];
ax.XTick = [];
ax.YTick = [];
box(ax, 'on');
title(ax, 'Platform Camera / YOLO', 'Color', [1 1 1], 'FontWeight', 'bold');
end

function [beltPatches, gapPatches] = draw3DFloorBase(ax, floorNo, g)
beltPatches = gobjects(1,4);
gapPatches = gobjects(1,4);
z = floorZ(floorNo, g);
for belt = 1:4
    rect = beltRect(belt, g);
    beltPatches(belt) = patch3Rect(ax, rect, z, [0.64 0.64 0.64], [0.15 0.16 0.18], 1.2);
    gapPatches(belt) = patch3Rect(ax, gapRect(belt, g), z + 0.004, [0.60 0.78 0.64], [0.20 0.55 0.25], 0.8);
    gapPatches(belt).LineStyle = '--';
    gapPatches(belt).FaceAlpha = 0.42;
end
patch3Rect(ax, waitAreaRect(g), z + 0.002, [1.00 0.90 0.90], [0.70 0.00 0.00], 1.2);
if floorNo == 1
    patch3Rect(ax, platformRect(g), z + 0.003, [0.86 0.88 0.90], [0.45 0.48 0.52], 1.0);
end
label3D(ax, g.b4x, mid(g.b4Start, g.b4End), z + 0.018, 'B4');
label3D(ax, mid(g.b1Left, g.b1Right), g.b1y, z + 0.018, 'B1');
label3D(ax, g.b2x, mid(g.b2Bottom, g.b2Top), z + 0.018, 'B2');
label3D(ax, mid(g.b3Left, g.b3Right), g.b3y, z + 0.018, 'B3');
label3D(ax, g.waitLeft + g.waitLength/2, g.waitBottom + g.waitWidth/2, z + 0.020, sprintf('Wait F%d', floorNo));
if floorNo == 1
    label3D(ax, g.platformX, g.platformY, z + 0.020, 'Platform');
end
text(ax, g.xMin3 + 0.02, g.yMin3 + 0.02, z + 0.03, sprintf('F%d', floorNo), ...
    'FontWeight', 'bold', 'Color', [0.10 0.10 0.10]);
end

function [beltPatches, gapPatches] = draw2DFloorBase(ax, floorNo, g)
beltPatches = gobjects(1,4);
gapPatches = gobjects(1,4);
for belt = 1:4
    beltPatches(belt) = patch2Rect(ax, rotate2DRect(beltRect(belt, g)), [0.70 0.70 0.70], [0.10 0.12 0.15], 1.8);
    gapPatches(belt) = patch2Rect(ax, rotate2DRect(gapRect(belt, g)), [0.60 0.78 0.64], [0.20 0.55 0.25], 1.0);
    gapPatches(belt).LineStyle = '--';
    gapPatches(belt).FaceAlpha = 0.45;
end
patch2Rect(ax, rotate2DRect(waitAreaRect(g)), [1.00 0.90 0.90], [0.70 0.00 0.00], 1.4);
if floorNo == 1
    patch2Rect(ax, rotate2DRect(platformRect(g)), [0.86 0.88 0.90], [0.45 0.48 0.52], 1.0);
end
label2D(ax, g.b4x, mid(g.b4Start, g.b4End), 'B4');
label2D(ax, mid(g.b1Left, g.b1Right), g.b1y, 'B1');
label2D(ax, g.b2x, mid(g.b2Bottom, g.b2Top), 'B2');
label2D(ax, mid(g.b3Left, g.b3Right), g.b3y, 'B3');
label2D(ax, g.waitLeft + g.waitLength/2, g.waitBottom + g.waitWidth/2, sprintf('Wait F%d', floorNo), [0.55 0 0]);
if floorNo == 1
    label2D(ax, g.platformX, g.platformY, 'Platform', [0.20 0.22 0.25]);
end
end

function updateActiveBelts(state, activeBelt)
for floorNo = 1:3
    for belt = 1:4
        if activeBelt == belt
            color = [0.20 0.58 1.00];
            edge = [0.02 0.20 0.55];
        else
            color = [0.64 0.64 0.64];
            edge = [0.15 0.16 0.18];
        end
        state.belt3d(floorNo,belt).FaceColor = color;
        state.belt3d(floorNo,belt).EdgeColor = edge;
        state.belt2d{floorNo}(belt).FaceColor = color;
        state.belt2d{floorNo}(belt).EdgeColor = edge;
    end
end
end

function state = updatePackages(ax3d, axF, state, rows, targetId, g)
n = numel(rows);
state = ensure3DHandles(ax3d, state, n);
for floorNo = 1:3
    state = ensure2DHandles(axF(floorNo), state, floorNo, n);
end

for ii = 1:numel(state.pkg3d)
    if ii > n
        hidePackage(state.pkg3d(ii), state.pkg3dText(ii));
        for floorNo = 1:3
            hidePackage(state.pkg2d{floorNo}(ii), state.pkg2dText{floorNo}(ii));
        end
        continue;
    end

    row = rows(ii);
    floorNo = rowFloor(row);
    belt = dbBeltToDisplay(numberField(row, 'belt', NaN));
    posM = rowPosM(row);
    id = round(numberField(row, 'id', 0));
    if floorNo < 1 || floorNo > 3 || belt < 1 || belt > 4 || isnan(posM) || id <= 0
        hidePackage(state.pkg3d(ii), state.pkg3dText(ii));
        for f = 1:3
            hidePackage(state.pkg2d{f}(ii), state.pkg2dText{f}(ii));
        end
        continue;
    end

    color = packageColor(row);
    if id == targetId && targetId > 0
        edgeColor = [1.00 0.05 0.02];
        lineWidth = 3.0;
    else
        edgeColor = [0.05 0.08 0.12];
        lineWidth = 1.5;
    end

    [xy, center] = packagePolygon(row, belt, posM, id, g);
    heightM = rowLengthM(row, {'height','box_height','h'}, 0.075);
    baseZ = floorZ(floorNo, g) + 0.004;
    [vertices, faces] = cuboidFromFootprint(xy, baseZ, max(heightM, 0.035));
    set(state.pkg3d(ii), 'Vertices', vertices, 'Faces', faces, ...
        'FaceColor', color, ...
        'EdgeColor', edgeColor, 'LineWidth', lineWidth, 'Visible', 'on');
    set(state.pkg3dText(ii), 'Position', [center(1), center(2), baseZ + heightM + 0.016], ...
        'String', sprintf('%d', id), 'Visible', 'on');

    [rx, ry] = rotate2DPoint(xy(:,1), xy(:,2));
    [tx, ty] = rotate2DPoint(center(1), center(2));
    for f = 1:3
        if f == floorNo
            set(state.pkg2d{f}(ii), 'XData', rx, 'YData', ry, ...
                'FaceColor', color, 'EdgeColor', edgeColor, ...
                'LineWidth', lineWidth, 'Visible', 'on');
            set(state.pkg2dText{f}(ii), 'Position', [tx, ty, 0], ...
                'String', sprintf('%d', id), 'Visible', 'on');
        else
            hidePackage(state.pkg2d{f}(ii), state.pkg2dText{f}(ii));
        end
    end
end
end

function state = ensure3DHandles(ax, state, n)
while numel(state.pkg3d) < n
    state.pkg3d(end+1) = patch(ax, 'Vertices', nan(8,3), ...
        'Faces', cuboidFaces(), 'FaceColor', [0.20 0.60 0.90], ...
        'EdgeColor', [0.05 0.08 0.12], 'LineWidth', 1.5, ...
        'Visible', 'off');
    state.pkg3dText(end+1) = text(ax, NaN, NaN, NaN, '', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontWeight', 'bold', 'Color', [0 0 0], 'Visible', 'off', 'Clipping', 'on');
end
end

function state = ensure2DHandles(ax, state, floorNo, n)
while numel(state.pkg2d{floorNo}) < n
    state.pkg2d{floorNo}(end+1) = patch(ax, NaN, NaN, [0.20 0.60 0.90], ...
        'EdgeColor', [0.05 0.08 0.12], 'LineWidth', 1.5, 'Visible', 'off');
    state.pkg2dText{floorNo}(end+1) = text(ax, NaN, NaN, '', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontWeight', 'bold', 'Color', [0 0 0], 'Visible', 'off', 'Clipping', 'on');
end
end

function hidePackage(patchHandle, textHandle)
if isvalid(patchHandle)
    patchHandle.Visible = 'off';
end
if isvalid(textHandle)
    textHandle.Visible = 'off';
end
end

function updateMachineActors(state, status, g)
platform = structField(status, 'platform');
pusher = structField(status, 'pusher');

platformFloor = round(numberField(platform, 'floor', 1));
platformFloor = min(max(platformFloor, 1), 3);
platformZ = platformZFromStatus(platform, platformFloor, g);
platformBodyZ = platformZ + 0.004;
tiltDeg = numberField(platform, {'tilt_deg','target_tilt_deg','servo_deg','angle_deg'}, 0.0);
platformActive = logicalField(platform, {'visible','busy','homed'}, false) ...
    || abs(numberField(platform, 'z_mm', 0.0)) > 0.5 ...
    || platformFloor ~= 1 ...
    || abs(tiltDeg) > 0.2 ...
    || numberField(platform, 'box_id', 0) > 0 ...
    || pusherIsActive(pusher);
if platformActive
    [vertices, faces] = cuboidFromTiltedRect(platformRect(g), platformBodyZ, 0.012, tiltDeg);
    set(state.platform3d, 'Vertices', vertices, 'Faces', faces, 'Visible', 'on');
    for floorNo = 1:3
        if floorNo == platformFloor
            setPatch2Rect(state.platform2d(floorNo), rotate2DRect(platformRect(g)), 'on');
        else
            state.platform2d(floorNo).Visible = 'off';
        end
    end
else
    state.platform3d.Visible = 'off';
    for floorNo = 1:3
        state.platform2d(floorNo).Visible = 'off';
    end
end

updateMainPusherActor(state, pusher, platformFloor, platformBodyZ, g);
updateSidePusherActor(state, pusher, platformFloor, platformBodyZ, g);
updatePlatformParcelActor(state, status, platformFloor, platformBodyZ, g);
end

function updateMainPusherActor(state, pusher, platformFloor, platformZ, g)
mainMm = numberField(pusher, {'main_mm','pusher_mm','main'}, 0.0);
active = logicalField(pusher, {'main_active','pusher_active'}, false) || abs(mainMm) > 0.5;
if ~active
    state.pusher3d.Visible = 'off';
    for floorNo = 1:3
        state.pusher2d(floorNo).Visible = 'off';
    end
    return;
end
travelM = 0.18;
frac = min(max((mainMm / 1000.0) / travelM, 0.0), 1.0);
barY = g.platformY - g.platformH/2 + 0.028 + frac * max(g.platformH - 0.070, 0.01);
rect = [g.platformX - g.platformW/2 + 0.018, barY, g.platformW - 0.036, 0.022];
[vertices, faces] = cuboidFromRect(rect, platformZ + 0.014, 0.040);
set(state.pusher3d, 'Vertices', vertices, 'Faces', faces, 'Visible', 'on');
for floorNo = 1:3
    if floorNo == platformFloor
        setPatch2Rect(state.pusher2d(floorNo), rotate2DRect(rect), 'on');
    else
        state.pusher2d(floorNo).Visible = 'off';
    end
end
end

function updateSidePusherActor(state, pusher, platformFloor, platformZ, g)
sideMm = numberField(pusher, {'side_mm','side'}, 0.0);
active = logicalField(pusher, {'side_active'}, false) || abs(sideMm) > 0.5;
if ~active
    state.sidePusher3d.Visible = 'off';
    for floorNo = 1:3
        state.sidePusher2d(floorNo).Visible = 'off';
    end
    return;
end
travelM = 0.18;
frac = min(max((sideMm / 1000.0) / travelM, 0.0), 1.0);
barX = g.platformX + g.platformW/2 - 0.030 - frac * max(g.platformW - 0.070, 0.01);
rect = [barX, g.platformY - g.platformH/2 + 0.018, 0.022, g.platformH - 0.036];
[vertices, faces] = cuboidFromRect(rect, platformZ + 0.014, 0.040);
set(state.sidePusher3d, 'Vertices', vertices, 'Faces', faces, 'Visible', 'on');
for floorNo = 1:3
    if floorNo == platformFloor
        setPatch2Rect(state.sidePusher2d(floorNo), rotate2DRect(rect), 'on');
    else
        state.sidePusher2d(floorNo).Visible = 'off';
    end
end
end

function updatePlatformParcelActor(state, status, platformFloor, platformZ, g)
parcel = structField(status, 'platform_parcel');
visible = logicalField(parcel, 'visible', false) && round(numberField(parcel, {'type','box_type'}, 0)) > 0;
if ~visible
    hidePackage(state.platformBox3d, state.platformBox3dText);
    for floorNo = 1:3
        hidePackage(state.platformBox2d(floorNo), state.platformBox2dText(floorNo));
    end
    return;
end
floorNo = round(numberField(parcel, 'floor', platformFloor));
floorNo = min(max(floorNo, 1), 3);
longSide = rowLengthM(parcel, {'long_side','long_mm','long'}, 0.12);
shortSide = rowLengthM(parcel, {'short_side','short_mm','short'}, 0.09);
heightM = rowLengthM(parcel, {'height','box_height','h'}, 0.075);
pusherMm = numberField(parcel, 'pusher_mm', numberField(structField(status, 'pusher'), {'main_mm','pusher_mm','main'}, 0));
platform = platformRect(g);
yStart = platform(2) + shortSide / 2 + 0.010;
yEnd = g.b4Start + shortSide / 2;
frac = min(max(pusherMm / 260.0, 0.0), 1.0);
center = [g.platformX, yStart + frac * (yEnd - yStart)];
xy = [
    center(1) - longSide/2, center(2) - shortSide/2
    center(1) + longSide/2, center(2) - shortSide/2
    center(1) + longSide/2, center(2) + shortSide/2
    center(1) - longSide/2, center(2) + shortSide/2
];
baseZ = platformZ + 0.012;
[vertices, faces] = cuboidFromFootprint(xy, baseZ, max(heightM, 0.035));
color = packageColor(parcel);
set(state.platformBox3d, 'Vertices', vertices, 'Faces', faces, ...
    'FaceColor', color, 'EdgeColor', [0.02 0.12 0.18], ...
    'LineWidth', 1.4, 'Visible', 'on');
label = sprintf('P? T%d', round(numberField(parcel, {'type','box_type'}, 0)));
set(state.platformBox3dText, 'Position', [center(1), center(2), baseZ + heightM + 0.014], ...
    'String', label, 'Visible', 'on');
[rx, ry] = rotate2DPoint(xy(:,1), xy(:,2));
[tx, ty] = rotate2DPoint(center(1), center(2));
for f = 1:3
    if f == floorNo
        set(state.platformBox2d(f), 'XData', rx, 'YData', ry, ...
            'FaceColor', color, 'Visible', 'on');
        set(state.platformBox2dText(f), 'Position', [tx, ty, 0], ...
            'String', label, 'Visible', 'on');
    else
        hidePackage(state.platformBox2d(f), state.platformBox2dText(f));
    end
end
end

function state = updateUnloadActors(ax3d, axF, state, status, g)
unload = structField(status, 'unload');
packages = struct([]);
if isstruct(unload) && isfield(unload, 'packages') && isstruct(unload.packages)
    packages = unload.packages(:);
end
n = numel(packages);
state = ensureUnload3DHandles(ax3d, state, n);
for floorNo = 1:3
    state = ensureUnload2DHandles(axF(floorNo), state, floorNo, n);
end

for ii = 1:max(numel(state.unload3d), n)
    if ii > n
        if ii <= numel(state.unload3d)
            hidePackage(state.unload3d(ii), state.unload3dText(ii));
        end
        for floorNo = 1:3
            if ii <= numel(state.unload2d{floorNo})
                hidePackage(state.unload2d{floorNo}(ii), state.unload2dText{floorNo}(ii));
            end
        end
        continue;
    end

    pkg = packages(ii);
    floorNo = round(numberField(pkg, 'floor', 3));
    floorNo = min(max(floorNo, 1), 3);
    id = round(numberField(pkg, 'id', ii));
    color = packageColor(pkg);
    [xy, center] = unloadPackagePolygon(pkg, g);
    heightM = rowLengthM(pkg, {'height','box_height','h'}, 0.075);
    baseZ = floorZ(floorNo, g) + 0.010;
    [vertices, faces] = cuboidFromFootprint(xy, baseZ, max(heightM, 0.035));
    set(state.unload3d(ii), 'Vertices', vertices, 'Faces', faces, ...
        'FaceColor', color, 'EdgeColor', [0.50 0.00 0.00], ...
        'LineWidth', 1.4, 'Visible', 'on');
    set(state.unload3dText(ii), 'Position', [center(1), center(2), baseZ + heightM + 0.014], ...
        'String', sprintf('U%d', id), 'Visible', 'on');

    [rx, ry] = rotate2DPoint(xy(:,1), xy(:,2));
    [tx, ty] = rotate2DPoint(center(1), center(2));
    for f = 1:3
        if f == floorNo
            set(state.unload2d{f}(ii), 'XData', rx, 'YData', ry, ...
                'FaceColor', color, 'EdgeColor', [0.50 0.00 0.00], ...
                'LineWidth', 1.3, 'Visible', 'on');
            set(state.unload2dText{f}(ii), 'Position', [tx, ty, 0], ...
                'String', sprintf('U%d', id), 'Visible', 'on');
        else
            hidePackage(state.unload2d{f}(ii), state.unload2dText{f}(ii));
        end
    end
end
end

function state = ensureUnload3DHandles(ax, state, n)
while numel(state.unload3d) < n
    state.unload3d(end+1) = patch(ax, 'Vertices', nan(8,3), ...
        'Faces', cuboidFaces(), 'FaceColor', [0.20 0.60 0.90], ...
        'EdgeColor', [0.50 0.00 0.00], 'LineWidth', 1.4, ...
        'Visible', 'off');
    state.unload3dText(end+1) = text(ax, NaN, NaN, NaN, '', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontWeight', 'bold', 'Color', [0 0 0], 'Visible', 'off', 'Clipping', 'on');
end
end

function state = ensureUnload2DHandles(ax, state, floorNo, n)
while numel(state.unload2d{floorNo}) < n
    state.unload2d{floorNo}(end+1) = patch(ax, NaN, NaN, [0.20 0.60 0.90], ...
        'EdgeColor', [0.50 0.00 0.00], 'LineWidth', 1.3, 'Visible', 'off');
    state.unload2dText{floorNo}(end+1) = text(ax, NaN, NaN, '', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontWeight', 'bold', 'Color', [0 0 0], 'Visible', 'off', 'Clipping', 'on');
end
end

function updateStatusOverlay(ax3d, axCamera, state, status, planText, targetId, rows)
mode = textField(status, 'mode', '-');
fault = textField(status, 'fault', '');
moving = round(numberField(status, 'hardware_moving', 0));
boxes = numel(rows);
tof = numericArrayField(status, 'tof');
if numel(tof) >= 7
    tofText = sprintf('CH0 %.0f | CH2 %.0f | CH4 %.0f | CH6 %.0f', tof(1), tof(3), tof(5), tof(7));
else
    tofText = 'CH0/2/4/6 -';
end
actorText = machineActorText(status);
state.infoText.String = sprintf('Mode %s | Boxes %d | Target P%d | Moving %d\n%s\n%s | %s', ...
    mode, boxes, targetId, moving, planText, tofText, actorText);
if fault ~= ""
    state.faultText.String = "FAULT: " + fault;
    state.faultText.Visible = 'on';
else
    state.faultText.Visible = 'off';
end
title(ax3d, 'Live Digital Twin 3D', 'FontWeight', 'bold');
updateCameraPanel(axCamera, state, status, rows, targetId);
updateTofLabels(state, tof);
end

function updateCameraPanel(ax, state, status, rows, targetId)
target = struct();
for ii = 1:numel(rows)
    if round(numberField(rows(ii), 'id', 0)) == targetId
        target = rows(ii);
        break;
    end
end
if isempty(fieldnames(target))
    platformParcel = structField(status, 'platform_parcel');
    if isstruct(platformParcel) && logicalField(platformParcel, 'visible', false)
        longSide = rowLengthM(platformParcel, {'long_side','long_mm','long'}, 0.12);
        shortSide = rowLengthM(platformParcel, {'short_side','short_mm','short'}, 0.09);
        w = min(0.23, max(0.045, longSide));
        h = min(0.08, max(0.025, shortSide * 0.55));
        state.cameraBox.XData = [-w/2 w/2 w/2 -w/2];
        state.cameraBox.YData = [-h/2 -h/2 h/2 h/2];
        state.cameraBox.FaceColor = packageColor(platformParcel);
        state.cameraBox.Visible = 'on';
        state.cameraText.String = sprintf('Platform Type %d   %.0f x %.0f mm', ...
            round(numberField(platformParcel, {'type','box_type'}, 0)), longSide*1000, shortSide*1000);
        state.cameraText.Position = [0 0 0];
        axis(ax, [-0.14 0.14 -0.055 0.055]);
        return;
    else
        state.cameraBox.Visible = 'off';
        state.cameraText.String = 'Target not selected';
        state.cameraText.Position = [0 0 0];
        return;
    end
end
longSide = rowLengthM(target, {'long_side','longSide','long'}, 0.12);
shortSide = rowLengthM(target, {'short_side','shortSide','short'}, 0.09);
w = min(0.23, max(0.045, longSide));
h = min(0.08, max(0.025, shortSide * 0.55));
state.cameraBox.XData = [-w/2 w/2 w/2 -w/2];
state.cameraBox.YData = [-h/2 -h/2 h/2 h/2];
state.cameraBox.FaceColor = packageColor(target);
state.cameraBox.Visible = 'on';
state.cameraText.String = sprintf('P%d Type %d   %.0f x %.0f mm', ...
    targetId, round(numberField(target, {'box_type','type'}, 0)), longSide*1000, shortSide*1000);
state.cameraText.Position = [0 0 0];
axis(ax, [-0.14 0.14 -0.055 0.055]);
end

function updateTofLabels(state, tof)
if isempty(state.tofText)
    return;
end
labels = ["CH0", "CH2", "CH4", "CH6"];
idx = [1 3 5 7];
for k = 1:4
    if numel(tof) >= idx(k)
        state.tofText(k).String = sprintf('%s %.0f', labels(k), tof(idx(k)));
    else
        state.tofText(k).String = labels(k) + " -";
    end
end
end

function p = patch2Rect(ax, rect, color, edge, lw)
x = [rect(1), rect(1)+rect(3), rect(1)+rect(3), rect(1)];
y = [rect(2), rect(2), rect(2)+rect(4), rect(2)+rect(4)];
p = patch(ax, x, y, color, 'EdgeColor', edge, 'LineWidth', lw);
end

function p = patch3Rect(ax, rect, z, color, edge, lw)
x = [rect(1), rect(1)+rect(3), rect(1)+rect(3), rect(1)];
y = [rect(2), rect(2), rect(2)+rect(4), rect(2)+rect(4)];
p = patch(ax, x, y, z * ones(1,4), color, 'EdgeColor', edge, 'LineWidth', lw);
end

function [xy, center] = packagePolygon(row, belt, posM, id, g)
longSide = rowLengthM(row, {'long_side','longSide','long'}, 0.12);
shortSide = rowLengthM(row, {'short_side','shortSide','short'}, 0.09);
% The original MATLAB simulator places the package center with beltXY(), but
% draws every package as an x/y-aligned footprint.  Keep that convention here;
% rotating by belt direction makes fixed DB rows look physically wrong.
[center, ~] = beltPose(belt, posM, longSide, shortSide, id, g);
xy = [
    center(1) - longSide/2, center(2) - shortSide/2
    center(1) + longSide/2, center(2) - shortSide/2
    center(1) + longSide/2, center(2) + shortSide/2
    center(1) - longSide/2, center(2) + shortSide/2
];
end

function [xy, center] = unloadPackagePolygon(row, g)
longSide = rowLengthM(row, {'long_side','longSide','long'}, 0.12);
shortSide = rowLengthM(row, {'short_side','shortSide','short'}, 0.09);
slotM = numberField(row, {'slot_m','slot'}, NaN);
if isnan(slotM)
    slotM = numberField(row, 'slot_mm', 0.0) / 1000.0;
end
rect = waitAreaRect(g);
pad = 0.012;
x = rect(1) + pad + slotM + longSide / 2;
y = rect(2) + rect(4) / 2;
center = [x, y];
xy = [
    center(1) - longSide/2, center(2) - shortSide/2
    center(1) + longSide/2, center(2) - shortSide/2
    center(1) + longSide/2, center(2) + shortSide/2
    center(1) - longSide/2, center(2) + shortSide/2
];
end

function [vertices, faces] = cuboidFromFootprint(xy, baseZ, heightM)
faces = cuboidFaces();
vertices = [
    xy, baseZ * ones(4,1)
    xy, (baseZ + heightM) * ones(4,1)
];
end

function faces = cuboidFaces()
faces = [
    1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
];
end

function [vertices, faces] = cuboidFromRect(rect, baseZ, heightM)
xy = [
    rect(1), rect(2)
    rect(1) + rect(3), rect(2)
    rect(1) + rect(3), rect(2) + rect(4)
    rect(1), rect(2) + rect(4)
];
[vertices, faces] = cuboidFromFootprint(xy, baseZ, heightM);
end

function [vertices, faces] = cuboidFromTiltedRect(rect, baseZ, heightM, tiltDeg)
xy = [
    rect(1), rect(2)
    rect(1) + rect(3), rect(2)
    rect(1) + rect(3), rect(2) + rect(4)
    rect(1), rect(2) + rect(4)
];
tiltDeg = max(-35.0, min(35.0, double(tiltDeg)));
centerY = rect(2) + rect(4) / 2.0;
zOffset = (xy(:,2) - centerY) * tan(tiltDeg * pi / 180.0);
zOffset = max(-0.060, min(0.060, zOffset));
faces = cuboidFaces();
vertices = [
    xy, baseZ + zOffset
    xy, baseZ + heightM + zOffset
];
end

function setPatch2Rect(handle, rect, visibleState)
if isempty(handle) || ~isvalid(handle)
    return;
end
x = [rect(1), rect(1)+rect(3), rect(1)+rect(3), rect(1)];
y = [rect(2), rect(2), rect(2)+rect(4), rect(2)+rect(4)];
set(handle, 'XData', x, 'YData', y, 'Visible', visibleState);
end

function s = structField(parent, name)
s = struct();
if isstruct(parent) && isfield(parent, name) && isstruct(parent.(name))
    s = parent.(name);
end
end

function tf = logicalField(s, names, defaultValue)
if nargin < 3
    defaultValue = false;
end
tf = logical(numberField(s, names, double(defaultValue)));
end

function z = platformZFromStatus(platform, platformFloor, g)
zM = numberField(platform, 'z_m', NaN);
if ~isnan(zM)
    z = zM;
    return;
end
zMm = numberField(platform, 'z_mm', NaN);
if ~isnan(zMm) && abs(zMm) > 1.0e-6
    z = zMm / 1000.0;
    return;
end
z = floorZ(platformFloor, g);
end

function active = pusherIsActive(pusher)
active = logicalField(pusher, {'main_active','pusher_active','side_active'}, false) ...
    || abs(numberField(pusher, {'main_mm','pusher_mm','main'}, 0.0)) > 0.5 ...
    || abs(numberField(pusher, {'side_mm','side'}, 0.0)) > 0.5;
end

function txt = machineActorText(status)
platform = structField(status, 'platform');
pusher = structField(status, 'pusher');
unload = structField(status, 'unload');
digitalTwin = structField(status, 'digital_twin');
floorNo = round(numberField(platform, 'floor', 1));
zMm = numberField(platform, 'z_mm', 0.0);
zUncMm = numberField(platform, 'z_uncertainty_mm', NaN);
tiltDeg = numberField(platform, {'tilt_deg','target_tilt_deg','servo_deg','angle_deg'}, 0.0);
tiltUncDeg = numberField(platform, 'tilt_uncertainty_deg', NaN);
mainMm = numberField(pusher, {'main_mm','pusher_mm','main'}, 0.0);
sideMm = numberField(pusher, {'side_mm','side'}, 0.0);
mainUncMm = numberField(pusher, 'main_uncertainty_mm', NaN);
sideUncMm = numberField(pusher, 'side_uncertainty_mm', NaN);
unloadUncMm = numberField(unload, 'layout_uncertainty_mm', NaN);
waitOcc = numericArrayField(unload, 'wait_occupied');
unloadCount = 0;
if isstruct(unload) && isfield(unload, 'packages') && isstruct(unload.packages)
    unloadCount = numel(unload.packages);
end
waitStr = "-";
if ~isempty(waitOcc)
    flags = waitOcc(1:min(3, numel(waitOcc))) > 0;
    waitStr = sprintf('%d', flags);
end
txt = sprintf('PF%d %.0f%s %.0f%sdeg | P%.0f%s/S%.0f%s | U%d%s W%s', ...
    floorNo, zMm, uncertaintySuffix(zUncMm), tiltDeg, uncertaintySuffix(tiltUncDeg), ...
    mainMm, uncertaintySuffix(mainUncMm), sideMm, uncertaintySuffix(sideUncMm), ...
    unloadCount, uncertaintySuffix(unloadUncMm), waitStr);
warningSuffix = warningText(digitalTwin);
if warningSuffix ~= ""
    txt = txt + " | " + warningSuffix;
end
end

function suffix = uncertaintySuffix(value)
if isnan(value)
    suffix = "";
else
    suffix = sprintf('\\x00B1%.0f', value);
    suffix = strrep(suffix, '\x00B1', char(177));
end
end

function txt = warningText(digitalTwin)
txt = "";
if ~isstruct(digitalTwin) || ~isfield(digitalTwin, 'warnings') || isempty(digitalTwin.warnings)
    return;
end
try
    warnings = string(digitalTwin.warnings);
    warnings = warnings(warnings ~= "");
    if ~isempty(warnings)
        short = strings(size(warnings));
        for ii = 1:numel(warnings)
            short(ii) = shortWarningToken(warnings(ii));
        end
        txt = "W:" + strjoin(short(1:min(2,end)), ",");
    end
catch
end
end

function token = shortWarningToken(warning)
warning = string(warning);
if contains(warning, "platform_not_homed")
    token = "pf_home";
elseif contains(warning, "platform")
    token = "pf";
elseif contains(warning, "pusher")
    token = "push";
elseif contains(warning, "unload")
    token = "unload";
else
    token = extractBefore(warning + "_", "_");
end
end

function [center, theta] = beltPose(belt, posM, longSide, shortSide, id, g)
unusedId = id; %#ok<NASGU>
aligned = 1.0;
displayPad = max([longSide, shortSide, 0.10]) + 0.05;
posM = min(max(posM, -displayPad), beltLengthM(belt, g) + displayPad);
switch belt
    case 4
        if aligned > 0.5
            x = g.b4Right - longSide/2;
        else
            x = lateralCenter(g.b4Left, g.b4Right, longSide, 0.5);
        end
        y = g.b4Start + posM;
        theta = pi/2;
    case 1
        x = g.b1Left + posM;
        if aligned > 0.5
            y = g.b1y - g.beltWidth/2 + shortSide/2;
        else
            y = lateralCenter(g.b1y - g.beltWidth/2, g.b1y + g.beltWidth/2, shortSide, 0.5);
        end
        theta = 0;
    case 2
        if aligned > 0.5
            x = g.b2x - g.beltWidth/2 + longSide/2;
        else
            x = lateralCenter(g.b2x - g.beltWidth/2, g.b2x + g.beltWidth/2, longSide, 0.5);
        end
        y = g.b2Top - posM;
        theta = -pi/2;
    case 3
        x = g.b3Right - posM;
        y = g.b3y + g.beltWidth/2 - shortSide/2;
        theta = pi;
    otherwise
        x = 0;
        y = 0;
        theta = 0;
end
center = [x, y];
end

function rect = beltRect(belt, g)
switch belt
    case 4
        rect = [g.b4Left, g.b4Start, g.beltWidth, g.b4End - g.b4Start];
    case 1
        rect = [g.b1Left, g.b1y - g.beltWidth/2, g.b1Right - g.b1Left, g.beltWidth];
    case 2
        rect = [g.b2x - g.beltWidth/2, g.b2Bottom, g.beltWidth, g.b2Top - g.b2Bottom];
    otherwise
        rect = [g.b3Left, g.b3y - g.beltWidth/2, g.b3Right - g.b3Left, g.beltWidth];
end
end

function rect = gapRect(belt, g)
gap = min(0.25, beltLengthM(belt, g));
switch belt
    case 4
        rect = [g.b4Left, g.b4Start, g.beltWidth, gap];
    case 1
        rect = [g.b1Right - gap, g.b1y - g.beltWidth/2, gap, g.beltWidth];
    case 2
        rect = [g.b2x - g.beltWidth/2, g.b2Bottom, g.beltWidth, gap];
    otherwise
        rect = [g.b3Left, g.b3y - g.beltWidth/2, gap, g.beltWidth];
end
end

function rect = waitAreaRect(g)
rect = [g.waitLeft, g.waitBottom, g.waitLength, g.waitWidth];
end

function rect = platformRect(g)
rect = [g.platformX - g.platformW/2, g.platformY - g.platformH/2, g.platformW, g.platformH];
end

function g = geom()
cfg = parcel_manual_config();
beltWidth = cfg.beltWidthM;
g.beltWidth = beltWidth;
g.beltThick = 0.015;
L1 = cfg.beltLengthM(1);
L2 = cfg.beltLengthM(2);
L3 = cfg.beltLengthM(3);
L4 = cfg.beltLengthM(4);
verticalSpan = max(L2, L4);
platformY = -0.35;
g.platformX = 0.0;
g.platformY = platformY;
g.platformW = beltWidth;
g.platformH = beltWidth;
stopperX = -0.15;
stopperY = platformY + 0.30;
Ax = stopperX - 0.03/2;
Ay = stopperY;
g.b3Right = Ax;
g.b3Left = g.b3Right - L3;
g.b3y = Ay;
g.b3Connect = g.b3y + beltWidth/2;
g.b4Left = Ax;
g.b4Right = Ax + beltWidth;
g.b4x = (g.b4Left + g.b4Right)/2;
g.b4Start = g.b3Connect - beltWidth;
g.b4End = g.b4Start + L4;
g.b1Right = g.b4Right;
g.b1Left = g.b1Right - L1;
g.b1y = g.b3Connect + verticalSpan - beltWidth/2;
g.b2x = g.b1Left - beltWidth/2;
g.b2Top = g.b3Connect + verticalSpan;
g.b2Bottom = g.b2Top - L2;
mirrorCenterX = (g.b4x + g.b2x) / 2;
g.platformX = mirrorX(g.platformX, mirrorCenterX);
[g.b3Left, g.b3Right] = mirrorRange(g.b3Left, g.b3Right, mirrorCenterX);
[g.b4Left, g.b4Right] = mirrorRange(g.b4Left, g.b4Right, mirrorCenterX);
[g.b1Left, g.b1Right] = mirrorRange(g.b1Left, g.b1Right, mirrorCenterX);
g.b4x = mirrorX(g.b4x, mirrorCenterX);
g.b2x = mirrorX(g.b2x, mirrorCenterX);
g.platformX = g.b4x;
g.platformY = g.b4Start - g.platformH / 2;
g.waitLength = cfg.waitAreaLengthM;
g.waitWidth = beltWidth;
g.waitLeft = g.b3Left;
g.waitBottom = g.b3y - beltWidth/2 - beltWidth;
rects = geometryRects(g);
pad = 0.14;
g.xMin3 = min(rects(:,1)) - pad;
g.xMax3 = max(rects(:,1) + rects(:,3)) + pad;
g.yMin3 = min(rects(:,2)) - pad;
g.yMax3 = max(rects(:,2) + rects(:,4)) + pad;
zs = cfg.floorHeightsM(1:min(cfg.floorCount, numel(cfg.floorHeightsM)));
g.zMin = -0.03;
g.zMax3 = max(zs) + 0.30;
[xs, ys] = rotatedBounds(rects);
g.xMin2 = min(xs) - pad;
g.xMax2 = max(xs) + pad;
g.yMin2 = min(ys) - pad;
g.yMax2 = max(ys) + pad;
g.infoX = g.xMin3;
g.infoY = g.yMax3 + 0.04;
g.infoZ = g.zMax3 - 0.08;
end

function rects = geometryRects(g)
rects = [
    beltRect(1, g)
    beltRect(2, g)
    beltRect(3, g)
    beltRect(4, g)
    waitAreaRect(g)
    platformRect(g)
];
end

function [xs, ys] = rotatedBounds(rects)
xs = [];
ys = [];
for i = 1:size(rects, 1)
    x0 = rects(i,1);
    y0 = rects(i,2);
    w = rects(i,3);
    h = rects(i,4);
    px = [x0; x0 + w; x0 + w; x0];
    py = [y0; y0; y0 + h; y0 + h];
    [rx, ry] = rotate2DPoint(px, py);
    xs = [xs; rx(:)]; %#ok<AGROW>
    ys = [ys; ry(:)]; %#ok<AGROW>
end
end

function rows = sortRows(rows)
if isempty(rows)
    return;
end
seq = zeros(numel(rows),1);
ids = zeros(numel(rows),1);
for ii = 1:numel(rows)
    seq(ii) = numberField(rows(ii), 'seq', 9999);
    ids(ii) = numberField(rows(ii), 'id', 0);
end
[~, order] = sortrows([seq ids]);
rows = rows(order);
end

function floorNo = rowFloor(row)
floorNo = round(numberField(row, 'floor', 1));
if floorNo < 1 || floorNo > 3
    floorNo = 1;
end
end

function belt = dbBeltToDisplay(raw)
% /refuge/db and /refuge/status.pending_move use the supervisor's internal
% zero-based belt index.  Convert only at the renderer boundary; logs and
% Arduino commands remain one-based elsewhere.
belt = round(raw);
if isnan(belt)
    return;
end
if belt >= 0 && belt <= 3
    belt = belt + 1;
end
if belt < 1 || belt > 4
    belt = NaN;
end
end

function posM = rowPosM(row)
posM = numberField(row, 'pos', NaN);
if ~isnan(posM) && abs(posM) > 5
    posM = posM / 1000.0;
end
end

function z = floorZ(floorNo, g)
try
    cfg = parcel_manual_config();
    z = cfg.floorHeightsM(floorNo);
catch
    z = (floorNo - 1) * 0.22;
end
z = z + g.beltThick;
end

function len = beltLengthM(belt, g)
switch belt
    case 1
        len = g.b1Right - g.b1Left;
    case 2
        len = g.b2Top - g.b2Bottom;
    case 3
        len = g.b3Right - g.b3Left;
    otherwise
        len = g.b4End - g.b4Start;
end
end

function color = packageColor(row)
boxType = round(numberField(row, {'box_type','type'}, 0));
palette = [
    0.94 0.45 0.12
    0.18 0.62 0.85
    0.20 0.68 0.32
    0.75 0.20 0.68
    0.93 0.76 0.20
];
if boxType >= 1 && boxType <= 4
    color = palette(boxType, :);
else
    id = max(1, round(numberField(row, 'id', 1)));
    color = palette(mod(id - 1, size(palette, 1)) + 1, :);
end
end

function value = rowLengthM(row, names, defaultM)
value = numberField(row, names, defaultM);
if abs(value) > 5
    value = value / 1000.0;
end
value = max(0.005, value);
end

function value = numberField(s, names, defaultValue)
if nargin < 3
    defaultValue = 0;
end
value = defaultValue;
if ~isstruct(s)
    return;
end
if ischar(names) || isstring(names)
    names = cellstr(string(names));
end
for ii = 1:numel(names)
    name = char(names{ii});
    if isfield(s, name)
        raw = s.(name);
        if isempty(raw)
            continue;
        end
        try
            value = double(raw(1));
            return;
        catch
            num = str2double(string(raw(1)));
            if ~isnan(num)
                value = num;
                return;
            end
        end
    end
end
end

function value = textField(s, names, defaultValue)
if nargin < 3
    defaultValue = "";
end
value = string(defaultValue);
if ~isstruct(s)
    return;
end
if ischar(names) || isstring(names)
    names = cellstr(string(names));
end
for ii = 1:numel(names)
    name = char(names{ii});
    if isfield(s, name)
        raw = s.(name);
        if isempty(raw)
            continue;
        end
        try
            if ischar(raw)
                value = string(raw);
            elseif isstring(raw)
                value = string(raw(1));
            elseif iscell(raw)
                value = string(raw{1});
            else
                value = string(raw(1));
            end
            return;
        catch
        end
    end
end
end

function values = numericArrayField(s, name)
values = [];
if ~isstruct(s) || ~isfield(s, name)
    return;
end
raw = s.(name);
try
    values = double(raw(:)');
catch
end
end

function [rx, ry] = rotate2DPoint(x, y)
rx = y;
ry = -x;
end

function rectOut = rotate2DRect(rectIn)
x0 = rectIn(1);
y0 = rectIn(2);
w = rectIn(3);
h = rectIn(4);
rectOut = [y0, -(x0 + w), h, w];
end

function label2D(ax, x, y, str, color)
if nargin < 5
    color = [0 0 0];
end
[tx, ty] = rotate2DPoint(x, y);
text(ax, tx, ty, str, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', 'FontSize', 8, 'Color', color, 'Clipping', 'on');
end

function label3D(ax, x, y, z, str)
text(ax, x, y, z, str, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', 'FontSize', 8, 'Color', [0 0 0], 'Clipping', 'on');
end

function x = mirrorX(x, centerX)
x = 2 * centerX - x;
end

function [left, right] = mirrorRange(left, right, centerX)
a = mirrorX(left, centerX);
b = mirrorX(right, centerX);
left = min(a, b);
right = max(a, b);
end

function center = lateralCenter(lowEdge, highEdge, crossSize, lane)
lo = lowEdge + crossSize/2;
hi = highEdge - crossSize/2;
if hi < lo
    center = (lowEdge + highEdge) / 2;
else
    center = lo + lane * (hi - lo);
end
end

function value = mid(a, b)
value = (a + b) / 2;
end

function pos = visiblePosition(w, h)
screen = get(0, 'ScreenSize');
margin = 40;
w = min(w, max(900, screen(3) - 2 * margin));
h = min(h, max(620, screen(4) - 2 * margin));
left = screen(1) + margin;
bottom = screen(2) + margin;
pos = [left bottom w h];
end
