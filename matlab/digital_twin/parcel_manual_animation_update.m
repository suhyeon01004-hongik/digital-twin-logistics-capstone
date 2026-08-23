function parcel_manual_animation_update(S)
%PARCEL_MANUAL_ANIMATION_UPDATE Live 3D HMI for the manual parcel simulator.

persistent fig ax3d axGauge axInfo axCamera axF1 axF2 axF3 frameCounter

if nargin == 0 || isempty(S)
    if ~isempty(fig) && isvalid(fig)
        close(fig);
    end
    fig = [];
    frameCounter = [];
    return;
end

if isempty(fig) || ~isvalid(fig)
    fig = figure('Name', 'Parcel Conveyor Manual 3D Simulation', ...
        'Color', [1 1 1], ...
        'MenuBar', 'none', ...
        'ToolBar', 'none', ...
        'NumberTitle', 'off', ...
        'Renderer', 'opengl', ...
        'GraphicsSmoothing', 'off', ...
        'Position', visibleMainPosition(1180, 680));
    ax3d = axes(fig, 'Position', [0.035 0.17 0.550 0.78]);
    axGauge = axes(fig, 'Position', [0.030 0.770 0.175 0.180]);
    axInfo = axes(fig, 'Position', [0.035 0.035 0.550 0.115]);
    axF3 = axes(fig, 'Position', [0.610 0.735 0.365 0.220]);
    axF2 = axes(fig, 'Position', [0.610 0.500 0.365 0.220]);
    axF1 = axes(fig, 'Position', [0.610 0.265 0.365 0.220]);
    axCamera = axes(fig, 'Position', [0.610 0.035 0.365 0.215]);
    configurePassiveAxes([ax3d axGauge axInfo axF3 axF2 axF1 axCamera]);
    frameCounter = 0;
end

if isempty(frameCounter)
    frameCounter = 0;
end
frameCounter = frameCounter + 1;
livePreview = isfield(S, 'livePreview') && S.livePreview > 0.5;

draw3DPanel(ax3d, S);
if livePreview || frameCounter <= 2 || mod(frameCounter, 12) == 0 || S.platformBoxActive > 0.5
    drawCameraPanel(axCamera, S);
end
if livePreview || frameCounter <= 2 || mod(frameCounter, 15) == 0 || (S.statusCode == 0 && mod(frameCounter, 6) == 0)
    drawCapacityGaugePanel(axGauge, S);
    drawInfoPanel(axInfo, S);
    drawFloorPanel(axF3, 3, S);
    drawFloorPanel(axF2, 2, S);
    drawFloorPanel(axF1, 1, S);
end
drawnow limitrate nocallbacks;
end

function configurePassiveAxes(axList)
for idx = 1:numel(axList)
    ax = axList(idx);
    try
        ax.Toolbar.Visible = 'off';
    catch
    end
    try
        disableDefaultInteractivity(ax);
    catch
    end
    try
        ax.Interactions = [];
    catch
    end
    try
        ax.ContextMenu = [];
    catch
    end
end
end

function pos = visibleMainPosition(preferredW, preferredH)
screen = get(0, 'ScreenSize');
margin = 40;
w = min(preferredW, max(760, screen(3) - 2 * margin));
h = min(preferredH, max(520, screen(4) - 2 * margin));
left = screen(1) + margin;
bottom = screen(2) + max(margin, screen(4) - h - margin);
pos = [left bottom w h];
end

function draw3DPanel(ax, S)
state = getappdata(ax, 'fast3dState');
if isempty(state) || ~isstruct(state) || ~isfield(state, 'pkgPatch') || ...
        ~isfield(state, 'barrierPatch') || ~isvalid(state.pkgPatch)
    state = initFast3DPanel(ax, numel(S.ids));
end
updateFast3DPanel(ax, state, S);
end

function state = initFast3DPanel(ax, maxPkg)
cla(ax);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
ax.GridAlpha = 0.18;
ax.Color = [0.98 0.98 0.98];
ax.SortMethod = 'depth';
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
zlabel(ax, 'floor z [m]');
title(ax, 'Manual 3D Simulation', 'Color', [0 0 0]);
view(ax, 38, 24);

g = geom();
set3DAxisLimits(ax, g);
state.beltPatch = gobjects(3,4);
state.waitPatch = gobjects(3,1);
state.barrierPatch = gobjects(3,1);
for floor = 1:3
    z = floorHeight(floor);
    rects = beltRects3D(g);
    for belt = 1:4
        [v, f] = cuboidVF(rects{belt}, z, g.beltThick);
        state.beltPatch(floor,belt) = patch(ax, 'Vertices', v, 'Faces', f, ...
            'FaceColor', [0.62 0.62 0.62], 'EdgeColor', [0.35 0.35 0.35], ...
            'LineWidth', 1.0, 'FaceAlpha', 1.0, 'Clipping', 'on');
        text(ax, rects{belt}(1) + rects{belt}(3)/2, rects{belt}(2) + rects{belt}(4)/2, z + 0.02, ...
            sprintf('B%d', belt), 'HorizontalAlignment', 'center', 'FontSize', 7, ...
            'FontWeight', 'bold', 'Color', [0 0 0], 'Clipping', 'on');
    end
    [vWait, fWait] = cuboidVF(waitAreaRect(g), z, g.beltThick * 0.8);
    state.waitPatch(floor) = patch(ax, 'Vertices', vWait, 'Faces', fWait, ...
        'FaceColor', [0.98 0.82 0.82], 'EdgeColor', [0.70 0.00 0.00], ...
        'LineWidth', 1.2, 'FaceAlpha', 1.0, 'Clipping', 'on');
    text(ax, g.waitLeft + g.waitLength/2, g.waitBottom + g.waitWidth/2, z + 0.02, ...
        'WAIT', 'HorizontalAlignment', 'center', 'FontSize', 6, ...
        'FontWeight', 'bold', 'Color', [0.55 0.00 0.00], 'Clipping', 'on');
    [vBar, fBar] = b4TofBarrierVF(g, z, 0);
    state.barrierPatch(floor) = patch(ax, 'Vertices', vBar, 'Faces', fBar, ...
        'FaceColor', b4TofBarrierColor(0), 'EdgeColor', [0.03 0.03 0.03], ...
        'LineWidth', 0.8, 'FaceAlpha', 1.0, 'Clipping', 'on');
    text(ax, 0.25, g.b1y, z + 0.025, sprintf('F%d', floor), ...
        'FontWeight', 'bold', 'Color', [0 0 0], 'Clipping', 'on');
    draw3DInnerWalls(ax, floorHeight(floor) + g.beltThick + 0.003);
end

[v, f] = cuboidVF([g.platformX - g.platformW/2, g.platformY - g.platformH/2, g.platformW, g.platformH], -0.02, 0.008);
state.platformPatch = patch(ax, 'Vertices', v, 'Faces', f, ...
    'FaceColor', [0.86 0.88 0.90], 'EdgeColor', [0.45 0.48 0.52], ...
    'LineWidth', 1.0, 'FaceAlpha', 1.0, 'Clipping', 'on');

state.pkgPatch = patch(ax, 'Vertices', zeros(0,3), 'Faces', zeros(0,4), ...
    'FaceVertexCData', zeros(0,3), 'FaceColor', 'flat', ...
    'EdgeColor', 'none', 'LineWidth', 0.8, 'FaceAlpha', 1.0, 'Clipping', 'on');
state.platformBoxPatch = patch(ax, 'Vertices', zeros(8,3), 'Faces', cuboidFaces(), ...
    'FaceVertexCData', zeros(6,3), 'FaceColor', 'flat', 'EdgeColor', 'none', ...
    'LineWidth', 1.0, 'FaceAlpha', 1.0, 'Visible', 'off', 'Clipping', 'on');
state.pusherPatch = patch(ax, 'Vertices', zeros(8,3), 'Faces', cuboidFaces(), ...
    'FaceColor', [0.18 0.22 0.26], 'EdgeColor', [0.05 0.06 0.07], ...
    'LineWidth', 1.0, 'FaceAlpha', 1.0, 'Visible', 'off', 'Clipping', 'on');
state.labelText = gobjects(0,1);
state.labelsShown = false;
hold(ax, 'off');
setappdata(ax, 'fast3dState', state);
end

function updateFast3DPanel(ax, state, S)
g = geom();
inactiveColor = [0.62 0.62 0.62];
inactiveEdge = [0.35 0.35 0.35];
activeColor = [0 0.42 1.0];
activeEdge = [0 0.20 0.55];
for floor = 1:3
    for belt = 1:4
        if beltMotorActive(floor, belt, S.dtMotorCmd)
            state.beltPatch(floor,belt).FaceColor = activeColor;
            state.beltPatch(floor,belt).EdgeColor = activeEdge;
        else
            state.beltPatch(floor,belt).FaceColor = inactiveColor;
            state.beltPatch(floor,belt).EdgeColor = inactiveEdge;
        end
    end
    barrierPos = b4TofBarrierPosition(S, floor);
    [vBar, ~] = b4TofBarrierVF(g, floorHeight(floor), barrierPos);
    state.barrierPatch(floor).Vertices = vBar;
    state.barrierPatch(floor).FaceColor = b4TofBarrierColor(barrierPos);
end

[vPlatform, ~] = cuboidVF([g.platformX - g.platformW/2, g.platformY - g.platformH/2, g.platformW, g.platformH], S.platformZ - 0.02, 0.008);
state.platformPatch.Vertices = vPlatform;

if S.platformBoxActive > 0.5
    [cx, cy] = platformPackageXY(g, S);
    [boxW, boxD, boxYaw] = platformPackageFootprint(S);
    [vBox, fBox] = rotatedCuboidVF(cx, cy, S.platformZ + 0.008, ...
        boxW, boxD, max(S.currentHeight, 0.035), boxYaw);
    state.platformBoxPatch.Vertices = vBox;
    state.platformBoxPatch.Faces = fBox;
    state.platformBoxPatch.FaceVertexCData = shadedFaceColors(packageTypeColor(S.currentLong));
    state.platformBoxPatch.Visible = 'on';
else
    state.platformBoxPatch.Visible = 'off';
end

if isfield(S, 'waitSidePusherActive') && S.waitSidePusherActive > 0.5
    [vPusher, fPusher] = waitSidePusherVF(g, S);
    state.pusherPatch.FaceColor = [0.55 0.05 0.05];
    state.pusherPatch.EdgeColor = [0.22 0.00 0.00];
    state.pusherPatch.Vertices = vPusher;
    state.pusherPatch.Faces = fPusher;
    state.pusherPatch.Visible = 'on';
elseif S.pusherActive > 0.5
    [vPusher, fPusher] = pusherVF(g, S);
    state.pusherPatch.FaceColor = [0.18 0.22 0.26];
    state.pusherPatch.EdgeColor = [0.05 0.06 0.07];
    state.pusherPatch.Vertices = vPusher;
    state.pusherPatch.Faces = fPusher;
    state.pusherPatch.Visible = 'on';
else
    state.pusherPatch.Visible = 'off';
end

[drawX, drawY] = applyB1EntryGapVisual(S.x, S.y, S.ids, S.floors, S.belts, S.boxLong);
[vertices, faces, faceColors, labelData] = packageSceneVF(S, drawX, drawY, g);
state.pkgPatch.Vertices = vertices;
state.pkgPatch.Faces = faces;
state.pkgPatch.FaceVertexCData = faceColors;
state.pkgPatch.EdgeColor = 'none';
if state.labelsShown && ~isempty(state.labelText)
    set(state.labelText, 'Visible', 'off');
    state.labelsShown = false;
end
setappdata(ax, 'fast3dState', state);
end

function rects = beltRects3D(g)
rects = cell(4,1);
rects{4} = [g.b4Left, g.b4Start, g.beltWidth, g.b4End - g.b4Start];
rects{1} = [g.b1Left, g.b1y - g.beltWidth/2, g.b1Right - g.b1Left, g.beltWidth];
rects{2} = [g.b2x - g.beltWidth/2, g.b2Bottom, g.beltWidth, g.b2Top - g.b2Bottom];
rects{3} = [g.b3Left, g.b3y - g.beltWidth/2, g.b3Right - g.b3Left, g.beltWidth];
end

function [vertices, faces, faceColors, labelData] = packageSceneVF(S, drawX, drawY, g)
vertices = zeros(0,3);
faces = zeros(0,4);
faceColors = zeros(0,3);
labelData.visible = false(numel(S.ids),1);
labelData.position = zeros(numel(S.ids),3);
for i = 1:numel(S.ids)
    if S.ids(i) > 0 && S.floors(i) > 0 && S.belts(i) > 0 && drawY(i) > -9
        floor = S.floors(i);
        localY = drawY(i) - floorOffset(floor);
        baseZ = floorHeight(floor) + g.beltThick;
        w = max(S.boxLong(i), 0.06);
        d = max(S.boxShort(i), 0.05);
        h = max(S.boxHeight(i), 0.035);
        [v, f] = cuboidVF([drawX(i)-w/2, localY-d/2, w, d], baseZ, h);
        offset = size(vertices, 1);
        vertices = [vertices; v]; %#ok<AGROW>
        faces = [faces; f + offset]; %#ok<AGROW>
        faceColors = [faceColors; shadedFaceColors(packageTypeColor(S.boxLong(i)))]; %#ok<AGROW>
        labelData.visible(i) = true;
        labelData.position(i,:) = [drawX(i), localY, baseZ + h + 0.025];
    end
end
if isfield(S, 'waitIds')
    for i = 1:numel(S.waitIds)
        if S.waitIds(i) > 0 && S.waitActive(i) > 0.5 && S.waitFloors(i) > 0
            floor = S.waitFloors(i);
            visualPos = S.waitPos(i) + waitAreaDynamicShift(g, floor, S);
            [cx, cy] = waitAreaPackageXY(g, visualPos, S.waitLong(i), S.waitShort(i));
            baseZ = floorHeight(floor) + g.beltThick;
            w = max(S.waitShort(i), 0.05);
            d = max(S.waitLong(i), 0.06);
            h = max(S.waitHeight(i), 0.035);
            [v, f] = cuboidVF([cx-w/2, cy-d/2, w, d], baseZ, h);
            offset = size(vertices, 1);
            vertices = [vertices; v]; %#ok<AGROW>
            faces = [faces; f + offset]; %#ok<AGROW>
            faceColors = [faceColors; shadedFaceColors(packageTypeColor(S.waitLong(i)))]; %#ok<AGROW>
        end
    end
end
end

function [v, f] = pusherVF(g, S)
[~, cy] = platformPackageXY(g, S);
barY = cy - max(S.currentShort, 0.05) / 2 - 0.030;
barY = max(barY, g.platformY - g.platformH/2 + 0.020);
rect = [g.platformX - g.platformW/2 + 0.015, barY, g.platformW - 0.030, 0.022];
[v, f] = cuboidVF(rect, S.platformZ + 0.012, 0.035);
end

function [v, f] = waitSidePusherVF(g, S)
[cx, cy] = platformPackageXY(g, S);
boxW = platformPackageXSize(S);
barX = cx - boxW / 2 - 0.030;
barX = min(barX, g.waitLeft - 0.012);
barX = max(barX, g.platformX - g.platformW/2 + 0.012);
rect = [barX, cy - g.platformH/2 + 0.020, 0.022, g.platformH - 0.040];
[v, f] = cuboidVF(rect, S.platformZ + 0.012, 0.035);
end

function [v, f] = cuboidVF(rect, z, thickness)
x0 = rect(1); y0 = rect(2); w = rect(3); h = rect(4);
v = [x0 y0 z; x0+w y0 z; x0+w y0+h z; x0 y0+h z; ...
    x0 y0 z+thickness; x0+w y0 z+thickness; x0+w y0+h z+thickness; x0 y0+h z+thickness];
f = cuboidFaces();
end

function [v, f] = rotatedCuboidVF(cx, cy, baseZ, w, d, h, yaw)
corners = rotatedBoxCorners(cx, cy, w, d, yaw);
v = [corners, baseZ * ones(4,1); corners, (baseZ + h) * ones(4,1)];
f = cuboidFaces();
end

function colors = shadedFaceColors(baseColor)
scale = [0.70; 1.00; 0.82; 0.88; 0.76; 0.92];
colors = min(max(scale .* baseColor, 0), 1);
end

function f = cuboidFaces()
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function drawInfoPanel(ax, S)
cla(ax);
axis(ax, 'off');
txt = [0 0 0];
moving = movingTextFromMotors(S.dtMotorCmd);
target = "None";
if S.targetId > 0
    target = sprintf("P%d: F%d B%d", S.targetId, S.targetFloor, S.targetBelt);
elseif isfield(S, 'circCompleteTargetId') && S.circCompleteTargetId > 0
    target = sprintf("Done P%d F%d", S.circCompleteTargetId, S.circCompleteFloor);
end
text(ax, 0.01, 0.75, sprintf('Status: %s', statusText(S.statusCode)), ...
    'FontWeight', 'bold', 'FontSize', 14, 'Color', txt);
text(ax, 0.30, 0.75, sprintf('Phase %d', S.phase), 'FontSize', 11, 'Color', txt);
text(ax, 0.42, 0.75, sprintf('Moving %s', moving), 'FontSize', 11, 'Color', txt);
text(ax, 0.60, 0.75, sprintf('Platform F%d', S.platformFloor), 'FontSize', 11, 'Color', txt);
text(ax, 0.76, 0.75, sprintf('Target %s', target), 'FontSize', 11, 'Color', txt);
waitTotal = 0;
if isfield(S, 'waitTotal')
    waitTotal = S.waitTotal;
end
text(ax, 0.01, 0.30, sprintf('Loaded %d / Unloaded %d / Wait %d', S.loadedCount, S.unloadedCount, waitTotal), ...
    'FontSize', 10, 'Color', txt);
text(ax, 0.36, 0.30, sprintf('Collision %d', S.collisionFlag), 'FontSize', 10, 'Color', txt);
text(ax, 0.51, 0.30, sprintf('Rotation %d', S.rotationFlag), 'FontSize', 10, 'Color', txt);
refugeCount = 0;
if isfield(S, 'tempUnloadCount')
    refugeCount = S.tempUnloadCount;
end
text(ax, 0.66, 0.30, sprintf('Refuge %d', refugeCount), 'FontSize', 10, 'Color', txt);
text(ax, 0.79, 0.30, sprintf('Gap %d', S.compactGapFlag), 'FontSize', 10, 'Color', txt);
text(ax, 0.01, 0.02, sprintf('%s | TOF empty %d/12 | motor channels %d', ...
    S.message, sum(S.dtTofEmpty > 0.5), sum(abs(S.dtMotorCmd) > 0.5)), ...
    'FontSize', 9, 'Color', [0.25 0.25 0.25]);
sideStep = 0;
if isfield(S, 'dtWaitSidePusherStepCmd')
    sideStep = S.dtWaitSidePusherStepCmd;
end
text(ax, 0.55, 0.02, sprintf('belt %.2f m/min | platform %+d | pusher %+d | side %+d | %s', ...
    S.beltSpeedMps * 60, S.dtPlatformStepCmd, S.dtPusherStepCmd, sideStep, ...
    b4TofBarrierInfoText(S)), ...
    'FontSize', 9, 'Color', [0.25 0.25 0.25]);
end

function txt = b4TofBarrierInfoText(S)
if ~isfield(S, 'b4TofBarrierPos')
    txt = "bar n/a";
    return;
end
state = strings(numel(S.b4TofBarrierPos),1);
for i = 1:numel(S.b4TofBarrierPos)
    if S.b4TofBarrierPos(i) >= 0.99
        state(i) = sprintf("F%d UP", i);
    elseif S.b4TofBarrierPos(i) <= 0.01
        state(i) = sprintf("F%d DN", i);
    else
        state(i) = sprintf("F%d %.0f%%", i, 100 * S.b4TofBarrierPos(i));
    end
end
fault = 0;
if isfield(S, 'b4TofBarrierFault')
    fault = S.b4TofBarrierFault;
end
if fault > 0.5
    txt = "bar FAULT";
else
    txt = "bar " + strjoin(state', "/");
end
end

function drawCapacityGaugePanel(ax, S)
[~, ~, percent, floorCounts, ~, ~, floorPercent] = capacityMetrics(S);
cla(ax);
hold(ax, 'on');
axis(ax, 'off');
xlim(ax, [0 1]);
ylim(ax, [0 1]);
rectangle(ax, 'Position', [0 0 1 1], 'FaceColor', [1 1 1], ...
    'EdgeColor', [0.75 0.78 0.82], 'LineWidth', 1.0);
text(ax, 0.04, 0.91, 'Belt Load Gauge', 'FontWeight', 'bold', ...
    'FontSize', 8, 'Color', [0 0 0]);
drawGaugeRow(ax, 0.04, 0.74, 0.92, 0.10, percent, [0.10 0.58 0.95], ...
    sprintf('Total %.0f%%  %d boxes', percent * 100, sum(floorCounts)));
for floor = 1:3
    y = 0.74 - 0.18 * floor;
    drawGaugeRow(ax, 0.04, y, 0.92, 0.10, floorPercent(floor), floorGaugeColor(floor), ...
        sprintf('F%d %.0f%%  %d boxes', floor, floorPercent(floor) * 100, floorCounts(floor)));
end
hold(ax, 'off');
end

function drawGaugeRow(ax, x, y, w, h, percent, color, label)
rectangle(ax, 'Position', [x y w h], 'FaceColor', [0.93 0.94 0.96], ...
    'EdgeColor', [0.45 0.48 0.52], 'LineWidth', 0.8);
rectangle(ax, 'Position', [x y w * min(max(percent, 0), 1) h], ...
    'FaceColor', color, 'EdgeColor', 'none');
text(ax, x, y + h + 0.020, label, 'FontWeight', 'bold', ...
    'FontSize', 6, 'Color', [0 0 0]);
end

function drawCameraPanel(ax, S)
cla(ax);
hold(ax, 'on');
axis(ax, 'off');
xlim(ax, [0 1]);
ylim(ax, [0 1]);
rectangle(ax, 'Position', [0 0 1 1], 'FaceColor', [0.08 0.09 0.10], ...
    'EdgeColor', [0.15 0.17 0.19], 'LineWidth', 1.2);
for gx = 0.12:0.16:0.92
    plot(ax, [gx gx], [0.12 0.88], '-', 'Color', [0.16 0.17 0.18], 'LineWidth', 0.6);
end
for gy = 0.16:0.16:0.84
    plot(ax, [0.08 0.92], [gy gy], '-', 'Color', [0.16 0.17 0.18], 'LineWidth', 0.6);
end
text(ax, 0.05, 0.93, 'Platform Camera / YOLO', 'Color', [0.95 0.98 1.00], ...
    'FontWeight', 'bold', 'FontSize', 8, 'HorizontalAlignment', 'left');
if S.currentPackageId <= 0 || S.currentLong <= 0
    text(ax, 0.50, 0.52, 'WAITING FOR BOX', 'Color', [0.75 0.80 0.85], ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 10);
    hold(ax, 'off');
    return;
end
liveText = 'HOLD';
liveColor = [1.00 0.72 0.18];
if S.statusCode == 1
    liveText = 'LIVE';
    liveColor = [0.10 1.00 0.25];
end
text(ax, 0.93, 0.93, liveText, 'Color', liveColor, 'FontWeight', 'bold', ...
    'FontSize', 8, 'HorizontalAlignment', 'right');
yawDeg = S.currentYaw * 180 / pi;
theta = yawDeg * pi / 180;
boxW = min(0.66, max(0.30, S.currentLong / 0.24 * 0.58));
boxH = min(0.48, max(0.20, S.currentShort / 0.19 * 0.38));
cx = 0.34 + 0.32 * platformLaneFactor(S.currentPackageId);
cy = 0.50;
[px, py] = rotatedRect(cx, cy, boxW, boxH, theta);
face = packageTypeColor(S.currentLong);
patch(ax, px, py, face, 'EdgeColor', [0.02 0.02 0.02], 'LineWidth', 1.4, 'FaceAlpha', 0.88);
plot(ax, [px; px(1)], [py; py(1)], '-', 'Color', [0.20 1.00 0.25], 'LineWidth', 2.0);
plot(ax, cx, cy, '+', 'Color', [1 1 1], 'LineWidth', 1.0, 'MarkerSize', 8);
text(ax, 0.05, 0.08, sprintf('P%d  Type %d', S.currentPackageId, packageTypeFromLongSide(S.currentLong)), ...
    'Color', [0.95 0.98 1.00], 'FontWeight', 'bold', 'FontSize', 7, 'HorizontalAlignment', 'left');
text(ax, 0.95, 0.08, sprintf('%.0f x %.0f mm  %.1f deg', S.currentLong*1000, S.currentShort*1000, yawDeg), ...
    'Color', [0.95 0.98 1.00], 'FontWeight', 'bold', 'FontSize', 7, 'HorizontalAlignment', 'right');
text(ax, 0.05, 0.84, sprintf('h %.0f mm', S.currentHeight * 1000), ...
    'Color', [0.72 0.82 0.90], 'FontSize', 7, 'HorizontalAlignment', 'left');
if S.platformBoxActive > 0.5
    text(ax, 0.95, 0.84, sprintf('pusher %.0f%%', 100 * S.pusherPosition / max(S.pusherTravel, 1.0e-6)), ...
        'Color', [0.72 0.82 0.90], 'FontSize', 7, 'HorizontalAlignment', 'right');
end
hold(ax, 'off');
end

function [px, py] = rotatedRect(cx, cy, w, h, theta)
corners = [-w/2 -h/2; w/2 -h/2; w/2 h/2; -w/2 h/2];
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
rot = corners * R';
px = rot(:,1) + cx;
py = rot(:,2) + cy;
end

function drawFloorPanel(ax, floor, S)
cla(ax);
hold(ax, 'on');
axis(ax, 'equal');
ax.Color = [1 1 1];
g = geom();
set2DAxisLimits(ax, g);
grid(ax, 'on');
ax.GridAlpha = 0.18;
ax.XColor = [0 0 0];
ax.YColor = [0 0 0];
set(ax, 'XTick', [], 'YTick', []);
text(ax, 0.50, 0.900, sprintf('Floor %d 2D Belt State', floor), ...
    'Units', 'normalized', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontSize', 10, ...
    'Color', [0 0 0], 'Clipping', 'off');

labelFontSize = packageLabelFontSize(S.ids);
[drawX, drawY] = applyB1EntryGapVisual(S.x, S.y, S.ids, S.floors, S.belts, S.boxLong);

draw2DBelts(ax, floor, S.dtMotorCmd);
drawWaitArea2D(ax, g, floor, S);
if S.platformFloor == floor
    draw2DPlatform(ax, g);
end
if S.platformBoxActive > 0.5 && S.platformBoxFloor == floor
    drawPlatformPackage2D(ax, g, S);
elseif isfield(S, 'waitSidePusherActive') && S.waitSidePusherActive > 0.5 && S.platformBoxFloor == floor
    drawWaitSidePusher2D(ax, g, S);
elseif S.pusherActive > 0.5 && S.platformBoxFloor == floor
    drawPusher2D(ax, g, S);
end
drawZones(ax, floor, S.dtTofGap, S.dtTofEmpty);
drawWaitAreaPackages2D(ax, g, floor, S, labelFontSize);

for i = 1:numel(S.ids)
    if S.ids(i) > 0 && S.floors(i) == floor && S.belts(i) > 0 && drawY(i) > -9
        localY = drawY(i) - floorOffset(floor);
        face = packageTypeColor(S.boxLong(i));
        edge = [0 0 0];
        lw = 1.1;
        if S.ids(i) == S.targetId && S.targetId > 0
            edge = [1 0 0];
            lw = 2.4;
        end
        w = max(S.boxLong(i), 0.06);
        h = max(S.boxShort(i), 0.05);
        rect = rotate2DRect([drawX(i)-w/2 localY-h/2 w h]);
        [tx, ty] = rotate2DPoint(drawX(i), localY);
        rectangle(ax, 'Position', rect, 'Curvature', 0.03, ...
            'FaceColor', face, 'EdgeColor', edge, 'LineWidth', lw);
        text(ax, tx, ty, sprintf('%d', S.ids(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontWeight', 'bold', 'FontSize', labelFontSize, ...
            'Color', [0 0 0], 'Clipping', 'on');
    end
end
draw2DInnerWalls(ax);
drawB4TofBarrier2D(ax, g, floor, S);
hold(ax, 'off');
end

function drawWaitArea2D(ax, g, floor, S)
rect = rotate2DRect(waitAreaRect(g));
used = 0;
if isfield(S, 'waitFloors')
    for i = 1:numel(S.waitFloors)
        if S.waitActive(i) > 0.5 && S.waitFloors(i) == floor
            used = used + minAxisForWait(S.waitLong(i), S.waitShort(i));
        end
    end
end
used = used + waitAreaDynamicShift(g, floor, S);
fillFrac = min(max(used / g.waitLength, 0), 1);
rectangle(ax, 'Position', rect, 'FaceColor', [1.00 0.90 0.90], ...
    'EdgeColor', [0.70 0.00 0.00], 'LineWidth', 1.4);
if fillFrac > 0
    fillRect = waitAreaRect(g);
    fillRect(3) = fillRect(3) * fillFrac;
    rectangle(ax, 'Position', rotate2DRect(fillRect), 'FaceColor', [0.98 0.35 0.35], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.18);
end
[tx, ty] = rotate2DPoint(g.waitLeft + g.waitLength/2, g.waitBottom + g.waitWidth/2);
text(ax, tx, ty, sprintf('Wait F%d', floor), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontSize', 6, 'FontWeight', 'bold', 'Color', [0.55 0.00 0.00], 'Clipping', 'on');
end

function drawWaitAreaPackages2D(ax, g, floor, S, labelFontSize)
if ~isfield(S, 'waitIds')
    return;
end
for i = 1:numel(S.waitIds)
    if S.waitIds(i) > 0 && S.waitActive(i) > 0.5 && S.waitFloors(i) == floor
        visualPos = S.waitPos(i) + waitAreaDynamicShift(g, floor, S);
        [cx, cy] = waitAreaPackageXY(g, visualPos, S.waitLong(i), S.waitShort(i));
        w = max(S.waitShort(i), 0.05);
        h = max(S.waitLong(i), 0.06);
        rect = rotate2DRect([cx-w/2 cy-h/2 w h]);
        [tx, ty] = rotate2DPoint(cx, cy);
        rectangle(ax, 'Position', rect, 'Curvature', 0.03, ...
            'FaceColor', packageTypeColor(S.waitLong(i)), ...
            'EdgeColor', [0.55 0.00 0.00], 'LineWidth', 1.4);
        text(ax, tx, ty, sprintf('%d', S.waitIds(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontWeight', 'bold', 'FontSize', labelFontSize, ...
            'Color', [0 0 0], 'Clipping', 'on');
    end
end
end

function draw2DBelts(ax, floor, motorCmd)
g = geom();
draw2DBeltRect(ax, [g.b4Left, g.b4Start, g.beltWidth, g.b4End - g.b4Start], beltMotorActive(floor, 4, motorCmd), 4);
draw2DBeltRect(ax, [g.b1Left, g.b1y - g.beltWidth/2, g.b1Right - g.b1Left, g.beltWidth], beltMotorActive(floor, 1, motorCmd), 1);
draw2DBeltRect(ax, [g.b2x - g.beltWidth/2, g.b2Bottom, g.beltWidth, g.b2Top - g.b2Bottom], beltMotorActive(floor, 2, motorCmd), 2);
draw2DBeltRect(ax, [g.b3Left, g.b3y - g.beltWidth/2, g.b3Right - g.b3Left, g.beltWidth], beltMotorActive(floor, 3, motorCmd), 3);
end

function draw2DPlatform(ax, g)
rect = rotate2DRect([g.platformX - g.platformW/2, g.platformY - g.platformH/2, g.platformW, g.platformH]);
[tx, ty] = rotate2DPoint(g.platformX, g.platformY);
rectangle(ax, 'Position', rect, ...
    'FaceColor', [0.86 0.88 0.90], 'EdgeColor', [0.45 0.48 0.52], 'LineWidth', 1.0);
text(ax, tx, ty, 'Platform', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontSize', 7, 'FontWeight', 'bold', 'Color', [0.20 0.22 0.25], 'Clipping', 'on');
end

function drawPlatformPackage2D(ax, g, S)
[cx, cy] = platformPackageXY(g, S);
[w, h, yaw] = platformPackageFootprint(S);
corners = rotatedBoxCorners(cx, cy, w, h, yaw);
[rx, ry] = rotate2DPoint(corners(:,1), corners(:,2));
face = packageTypeColor(S.currentLong);
patch(ax, rx, ry, face, 'EdgeColor', [0.02 0.02 0.02], ...
    'LineWidth', 1.3, 'FaceAlpha', 0.88, 'Clipping', 'on');
[tx, ty] = rotate2DPoint(cx, cy);
text(ax, tx, ty, sprintf('%d', S.currentPackageId), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontWeight', 'bold', 'FontSize', 8, 'Color', [0 0 0], 'Clipping', 'on');
if isfield(S, 'waitSidePusherActive') && S.waitSidePusherActive > 0.5
    drawWaitSidePusher2D(ax, g, S);
elseif S.pusherActive > 0.5
    drawPusher2D(ax, g, S);
end
end

function draw2DBeltRect(ax, rect, isActive, belt)
if isActive
    color = [0 0.42 1.0];
    edge = [0 0.20 0.55];
else
    color = [0.72 0.72 0.72];
    edge = [0.45 0.45 0.45];
end
drawRect = rotate2DRect(rect);
[tx, ty] = rotate2DPoint(rect(1) + rect(3)/2, rect(2) + rect(4)/2);
rectangle(ax, 'Position', drawRect, 'FaceColor', color, 'EdgeColor', edge, 'LineWidth', 1.2);
text(ax, tx, ty, sprintf('B%d', belt), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontSize', 8, 'FontWeight', 'bold', 'Color', [0 0 0], 'Clipping', 'on');
end

function draw2DInnerWalls(ax)
g = geom();
wallColor = [0.04 0.05 0.06];
wallWidth = 2.0;
draw2DLine(ax, [g.b4Right g.b4Start], [g.b4Right g.b4End], wallColor, wallWidth);
draw2DLine(ax, [g.b1Left g.b1y-g.beltWidth/2], [g.b1Right g.b1y-g.beltWidth/2], wallColor, wallWidth);
draw2DLine(ax, [g.b2x-g.beltWidth/2 g.b2Bottom], [g.b2x-g.beltWidth/2 g.b2Top], wallColor, wallWidth);
draw2DLine(ax, [g.b3Left g.b3y+g.beltWidth/2], [g.b3Right g.b3y+g.beltWidth/2], wallColor, wallWidth);
end

function draw2DLine(ax, p1, p2, color, lw)
[x1, y1] = rotate2DPoint(p1(1), p1(2));
[x2, y2] = rotate2DPoint(p2(1), p2(2));
plot(ax, [x1 x2], [y1 y2], '-', 'Color', color, 'LineWidth', lw, 'Clipping', 'on');
end

function drawZones(ax, floor, dtTofGap, dtTofEmpty)
g = geom();
zones = [
    g.b4Left, g.b4Start, g.beltWidth, g.beltWidth, 4;
    g.b1Left, g.b1y-g.beltWidth/2, g.beltWidth, g.beltWidth, 1;
    g.b2x-g.beltWidth/2, g.b2Top-g.beltWidth, g.beltWidth, g.beltWidth, 2;
    g.b3Right-g.beltWidth, g.b3y-g.beltWidth/2, g.beltWidth, g.beltWidth, 3
];
for i = 1:size(zones,1)
    belt = zones(i,5);
    idx = sensorIndex(floor, belt);
    if dtTofEmpty(idx) > 0.5
        zoneColor = [0.20 0.85 0.35];
        edgeColor = [0.00 0.45 0.10];
    else
        zoneColor = [1.0 0.90 0.25];
        edgeColor = [0.80 0.55 0.00];
    end
    rect = rotate2DRect(zones(i,1:4));
    rectangle(ax, 'Position', rect, 'EdgeColor', edgeColor, ...
        'FaceColor', zoneColor, 'FaceAlpha', 0.20, 'LineStyle', '--');
end
end

function drawB4TofBarrier2D(ax, g, floor, S)
pos = b4TofBarrierPosition(S, floor);
color = b4TofBarrierColor(pos);
lineWidth = 2.8;
if pos > 0.5
    lineWidth = 1.8;
end
y = g.b4Start - 0.004 - 0.030 * min(max(pos, 0), 1);
[x1, y1] = rotate2DPoint(g.b4Left, y);
[x2, y2] = rotate2DPoint(g.b4Right, y);
plot(ax, [x1 x2], [y1 y2], '-', 'Color', color, ...
    'LineWidth', lineWidth, 'Clipping', 'on');
end

function pos = b4TofBarrierPosition(S, floor)
pos = 0;
if isfield(S, 'b4TofBarrierPos') && numel(S.b4TofBarrierPos) >= floor
    pos = S.b4TofBarrierPos(floor);
end
pos = min(max(double(pos), 0), 1);
end

function color = b4TofBarrierColor(pos)
if pos > 0.5
    color = [0.12 0.48 0.95];
else
    color = [0.08 0.08 0.08];
end
end

function [v, f] = b4TofBarrierVF(g, floorZ, pos)
rect = b4TofBarrierRect(g);
baseZ = floorZ + g.beltThick + 0.004 + 0.115 * min(max(pos, 0), 1);
[v, f] = cuboidVF(rect, baseZ, 0.050);
end

function rect = b4TofBarrierRect(g)
rect = [g.b4Left, g.b4Start - 0.006, g.beltWidth, 0.012];
end

function idx = sensorIndex(floor, belt)
idx = (floor - 1) * 4 + belt;
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

function draw3DBelts(ax, floor, z, motorCmd)
g = geom();
draw3DBeltSurface(ax, [g.b4Left, g.b4Start, g.beltWidth, g.b4End - g.b4Start], z, beltMotorActive(floor, 4, motorCmd), 4);
draw3DBeltSurface(ax, [g.b1Left, g.b1y - g.beltWidth/2, g.b1Right - g.b1Left, g.beltWidth], z, beltMotorActive(floor, 1, motorCmd), 1);
draw3DBeltSurface(ax, [g.b2x - g.beltWidth/2, g.b2Bottom, g.beltWidth, g.b2Top - g.b2Bottom], z, beltMotorActive(floor, 2, motorCmd), 2);
draw3DBeltSurface(ax, [g.b3Left, g.b3y - g.beltWidth/2, g.b3Right - g.b3Left, g.beltWidth], z, beltMotorActive(floor, 3, motorCmd), 3);
text(ax, 0.25, g.b1y, z + 0.025, sprintf('F%d', floor), 'FontWeight', 'bold', ...
    'Color', [0 0 0], 'Clipping', 'on');
end

function draw3DBeltSurface(ax, rect, z, isActive, belt)
g = geom();
if isActive
    color = [0 0.42 1.0];
    edge = [0 0.20 0.55];
else
    color = [0.62 0.62 0.62];
    edge = [0.35 0.35 0.35];
end
draw3DPlate(ax, rect, z, g.beltThick, color, edge, 1.0);
text(ax, rect(1) + rect(3)/2, rect(2) + rect(4)/2, z + 0.02, ...
    sprintf('B%d', belt), 'HorizontalAlignment', 'center', 'FontSize', 7, ...
    'FontWeight', 'bold', 'Color', [0 0 0], 'Clipping', 'on');
end

function drawPlatformPackage3D(ax, g, S)
[cx, cy] = platformPackageXY(g, S);
baseZ = S.platformZ + 0.008;
[w, d, yaw] = platformPackageFootprint(S);
h = max(S.currentHeight, 0.035);
face = packageTypeColor(S.currentLong);
drawPackage3DRotated(ax, cx, cy, baseZ, w, d, h, yaw, face, [0 0 0], 1.2);
drawPusher3D(ax, g, S);
end

function draw3DPlatform(ax, g, z)
rect = [g.platformX - g.platformW/2, g.platformY - g.platformH/2, g.platformW, g.platformH];
draw3DPlate(ax, rect, z, 0.008, [0.86 0.88 0.90], [0.45 0.48 0.52], 1.0);
end

function draw3DInnerWalls(ax, z)
g = geom();
wallColor = [0.04 0.05 0.06];
wallWidth = 2.2;
plot3(ax, [g.b4Right g.b4Right], [g.b4Start g.b4End], [z z], '-', 'Color', wallColor, 'LineWidth', wallWidth, 'Clipping', 'on');
plot3(ax, [g.b1Left g.b1Right], [g.b1y-g.beltWidth/2 g.b1y-g.beltWidth/2], [z z], '-', 'Color', wallColor, 'LineWidth', wallWidth, 'Clipping', 'on');
plot3(ax, [g.b2x-g.beltWidth/2 g.b2x-g.beltWidth/2], [g.b2Bottom g.b2Top], [z z], '-', 'Color', wallColor, 'LineWidth', wallWidth, 'Clipping', 'on');
plot3(ax, [g.b3Left g.b3Right], [g.b3y+g.beltWidth/2 g.b3y+g.beltWidth/2], [z z], '-', 'Color', wallColor, 'LineWidth', wallWidth, 'Clipping', 'on');
end

function drawPackage3D(ax, cx, cy, baseZ, w, d, h, face, edge, lw)
rect = [cx - w/2, cy - d/2, w, d];
draw3DPlate(ax, rect, baseZ, h, face, edge, lw);
end

function drawPackage3DRotated(ax, cx, cy, baseZ, w, d, h, yaw, face, edge, lw)
corners = rotatedBoxCorners(cx, cy, w, d, yaw);
z0 = baseZ;
z1 = baseZ + h;
vertices = [corners, z0 * ones(4,1); corners, z1 * ones(4,1)];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', vertices, 'Faces', faces, 'FaceColor', face, ...
    'EdgeColor', 'none', 'LineWidth', lw, 'FaceAlpha', 1.0);
end

function corners = rotatedBoxCorners(cx, cy, w, h, yaw)
base = [-w/2 -h/2; w/2 -h/2; w/2 h/2; -w/2 h/2];
R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
corners = base * R' + [cx cy];
end

function [cx, cy] = platformPackageXY(g, S)
lane = min(max(S.platformBoxLane, 0.08), 0.92);
if isfield(S, 'waitTransferActive') && S.waitTransferActive > 0.5
    cx = lateralCenter(g.platformX - g.platformW/2, g.platformX + g.platformW/2, max(S.currentShort, 0.05), lane);
    pushFrac = min(max(S.waitSidePusherPosition / max(S.waitSidePusherTravel, 1.0e-6), 0), 1);
    endX = g.waitLeft + max(S.currentShort, 0.05) / 2;
    endY = waitAreaCrossCenter(g, max(S.currentLong, 0.06));
    cx = cx + (endX - cx) * pushFrac;
    cy = g.platformY + (endY - g.platformY) * pushFrac;
else
    cx = lateralCenter(g.platformX - g.platformW/2, g.platformX + g.platformW/2, max(S.currentLong, 0.06), lane);
    pushFrac = min(max(S.pusherPosition / max(S.pusherTravel, 1.0e-6), 0), 1);
    startY = g.platformY;
    endY = g.b4Start - max(S.currentShort, 0.05) / 2;
    cy = startY + (endY - startY) * pushFrac;
end
end

function drawPusher3D(ax, g, S)
[~, cy] = platformPackageXY(g, S);
barY = cy - max(S.currentShort, 0.05) / 2 - 0.030;
barY = max(barY, g.platformY - g.platformH/2 + 0.020);
rect = [g.platformX - g.platformW/2 + 0.015, barY, g.platformW - 0.030, 0.022];
draw3DPlate(ax, rect, S.platformZ + 0.012, 0.035, [0.18 0.22 0.26], [0.05 0.06 0.07], 1.0);
end

function drawPusher2D(ax, g, S)
[~, cy] = platformPackageXY(g, S);
barY = cy - max(S.currentShort, 0.05) / 2 - 0.030;
barY = max(barY, g.platformY - g.platformH/2 + 0.020);
rect = rotate2DRect([g.platformX - g.platformW/2 + 0.015, barY, g.platformW - 0.030, 0.022]);
rectangle(ax, 'Position', rect, 'FaceColor', [0.18 0.22 0.26], ...
    'EdgeColor', [0.05 0.06 0.07], 'LineWidth', 1.0);
end

function drawWaitSidePusher2D(ax, g, S)
[cx, cy] = platformPackageXY(g, S);
boxW = platformPackageXSize(S);
barX = cx - boxW / 2 - 0.030;
barX = min(barX, g.waitLeft - 0.012);
barX = max(barX, g.platformX - g.platformW/2 + 0.012);
rect = rotate2DRect([barX, cy - g.platformH/2 + 0.020, 0.022, g.platformH - 0.040]);
rectangle(ax, 'Position', rect, 'FaceColor', [0.55 0.05 0.05], ...
    'EdgeColor', [0.22 0.00 0.00], 'LineWidth', 1.0);
end

function draw3DPlate(ax, rect, z, thickness, face, edge, lw)
x0 = rect(1); y0 = rect(2); w = rect(3); h = rect(4);
z0 = z; z1 = z + thickness;
vertices = [x0 y0 z0; x0+w y0 z0; x0+w y0+h z0; x0 y0+h z0; ...
    x0 y0 z1; x0+w y0 z1; x0+w y0+h z1; x0 y0+h z1];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', vertices, 'Faces', faces, 'FaceColor', face, ...
    'EdgeColor', edge, 'LineWidth', lw, 'FaceAlpha', 1.0);
end

function drawCuboidEdges3D(ax, rect, z, thickness, color, lw)
x0 = rect(1); y0 = rect(2); w = rect(3); h = rect(4);
vertices = [x0 y0 z; x0+w y0 z; x0+w y0+h z; x0 y0+h z; ...
    x0 y0 z+thickness; x0+w y0 z+thickness; x0+w y0+h z+thickness; x0 y0+h z+thickness];
edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
for i = 1:size(edges, 1)
    p = vertices(edges(i,:), :);
    plot3(ax, p(:,1), p(:,2), p(:,3), '-', 'Color', color, 'LineWidth', lw, 'Clipping', 'on');
end
end

function [drawX, drawY] = applyB1EntryGapVisual(x, y, ids, floors, belts, boxLong)
drawX = x;
drawY = y;
% Keep the display tied to the physical simulation coordinates. Earlier builds
% visually re-spaced B1 parcels here, which made valid 250 mm top gaps look
% much larger than the actual TOF gap.
unused = {ids, floors, belts, boxLong}; %#ok<NASGU>
end

function [used, capacity, percent, floorCounts, floorUsed, floorCapacity, floorPercent] = capacityMetrics(S)
cfg = parcel_manual_config();
nFloors = cfg.floorCount;
used = 0;
floorCounts = zeros(nFloors,1);
floorUsed = zeros(nFloors,1);
floorCapacity = beltLengthForDisplay(1) + beltLengthForDisplay(2) + beltLengthForDisplay(3) + beltLengthForDisplay(4);
for i = 1:numel(S.ids)
    if S.ids(i) > 0 && S.floors(i) >= 1 && S.floors(i) <= nFloors && S.belts(i) >= 1 && S.belts(i) <= 4
        len = axisLengthForBelt(S.belts(i), S.boxLong(i), S.boxShort(i));
        used = used + len;
        floorUsed(S.floors(i)) = floorUsed(S.floors(i)) + len;
        floorCounts(S.floors(i)) = floorCounts(S.floors(i)) + 1;
    end
end
capacity = nFloors * floorCapacity;
percent = used / capacity;
floorPercent = floorUsed / floorCapacity;
end

function rect = waitAreaRect(g)
rect = [g.waitLeft, g.waitBottom, g.waitLength, g.waitWidth];
end

function [cx, cy] = waitAreaPackageXY(g, pos, longSide, shortSide)
unused = shortSide; %#ok<NASGU>
cx = g.waitLeft + pos;
cy = waitAreaCrossCenter(g, max(longSide, 0.06));
end

function shift = waitAreaDynamicShift(g, floor, S)
shift = 0;
if ~isfield(S, 'waitTransferActive') || S.waitTransferActive <= 0.5
    return;
end
if waitTransferDestinationFloor(S) ~= floor
    return;
end
if ~isfield(S, 'currentShort') || S.currentShort <= 0
    return;
end
[cx, ~] = platformPackageXY(g, S);
incomingAxis = max(S.currentShort, 0.05);
incomingFront = (cx - g.waitLeft) + incomingAxis / 2;
shift = min(max(incomingFront, 0), incomingAxis);
end

function floor = waitTransferDestinationFloor(S)
floor = 0;
if isfield(S, 'platformTargetFloor') && S.platformTargetFloor > 0
    floor = round(S.platformTargetFloor);
elseif isfield(S, 'targetFloor') && S.targetFloor > 0
    floor = round(S.targetFloor);
elseif isfield(S, 'platformFloor') && S.platformFloor > 0
    floor = round(S.platformFloor);
end
end

function [w, d, yaw] = platformPackageFootprint(S)
if isfield(S, 'waitTransferActive') && S.waitTransferActive > 0.5
    w = max(S.currentShort, 0.05);
    d = max(S.currentLong, 0.06);
    yaw = 0;
else
    w = max(S.currentLong, 0.06);
    d = max(S.currentShort, 0.05);
    yaw = S.currentYaw;
end
end

function w = platformPackageXSize(S)
if isfield(S, 'waitTransferActive') && S.waitTransferActive > 0.5
    w = max(S.currentShort, 0.05);
else
    w = max(S.currentLong, 0.06);
end
end

function cy = waitAreaCrossCenter(g, crossSide)
lo = g.waitBottom + crossSide / 2;
hi = g.waitBottom + g.waitWidth - crossSide / 2;
if hi < lo
    cy = g.waitBottom + g.waitWidth / 2;
else
    cy = (lo + hi) / 2;
end
end

function len = minAxisForWait(longSide, shortSide)
unused = longSide; %#ok<NASGU>
len = shortSide;
end

function len = axisLengthForBelt(belt, longSide, shortSide)
if belt == 1 || belt == 3
    len = longSide;
else
    len = shortSide;
end
end

function len = beltLengthForDisplay(belt)
cfg = parcel_manual_config();
idx = min(max(1, round(belt)), numel(cfg.beltLengthM));
len = cfg.beltLengthM(idx);
end

function color = floorGaugeColor(floor)
if floor == 1
    color = [0.20 0.65 0.35];
elseif floor == 2
    color = [0.95 0.62 0.15];
else
    color = [0.55 0.36 0.90];
end
end

function fontSize = packageLabelFontSize(ids)
activeCount = sum(ids > 0);
if activeCount >= 55
    fontSize = 5;
elseif activeCount >= 35
    fontSize = 6;
else
    fontSize = 8;
end
end

function color = packageTypeColor(longSide)
palette = [0.20 0.58 0.95; 0.95 0.48 0.16; 0.22 0.68 0.30; 0.82 0.20 0.70; 0.95 0.73 0.18];
color = palette(packageTypeFromLongSide(longSide), :);
end

function boxType = packageTypeFromLongSide(longSide)
cfg = parcel_manual_config();
scaledLong = max(cfg.packageSizeM(:,1), cfg.packageSizeM(:,2)) * cfg.packageScale;
[~, boxType] = min(abs(scaledLong - longSide));
end

function s = statusText(code)
if code == 0
    s = 'IDLE';
elseif code == 1
    s = 'LOADING';
elseif code == 2
    s = 'CIRCULATION';
elseif code == 3
    s = 'UNLOADING';
else
    s = 'COMPLETE';
end
end

function flag = beltMotorActive(floor, belt, motorCmd)
idx = sensorIndex(floor, belt);
flag = idx <= numel(motorCmd) && abs(motorCmd(idx)) > 0.5;
end

function moving = movingTextFromMotors(motorCmd)
cfg = parcel_manual_config();
parts = strings(0,1);
for floor = 1:cfg.floorCount
    for belt = 1:4
        idx = sensorIndex(floor, belt);
        if idx <= numel(motorCmd) && abs(motorCmd(idx)) > 0.5
            parts(end+1,1) = sprintf("F%d/B%d", floor, belt); %#ok<AGROW>
        end
    end
end
if isempty(parts)
    moving = "None";
else
    moving = strjoin(parts, ", ");
end
end

function r = platformLaneFactor(packageId)
seed = mod(packageId * 1664525 + 1013904223, 10000);
r = seed / 9999;
r = min(max(r, 0.08), 0.92);
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

function v = floorOffset(floor)
v = (floor - 2) * 1.7;
end

function z = floorHeight(floor)
cfg = parcel_manual_config();
idx = min(max(1, round(floor)), numel(cfg.floorHeightsM));
z = cfg.floorHeightsM(idx);
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
end

function set3DAxisLimits(ax, g)
rects = geometryRects(g);
pad = 0.12;
xlim(ax, [min(rects(:,1)) - pad, max(rects(:,1) + rects(:,3)) + pad]);
ylim(ax, [min(rects(:,2)) - pad, max(rects(:,2) + rects(:,4)) + pad]);
cfg = parcel_manual_config();
topZ = max(cfg.floorHeightsM(1:min(cfg.floorCount, numel(cfg.floorHeightsM)))) + 0.24;
zlim(ax, [-0.03 topZ]);
end

function set2DAxisLimits(ax, g)
rects = geometryRects(g);
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
pad = 0.12;
xlim(ax, [min(xs) - pad, max(xs) + pad]);
ylim(ax, [min(ys) - pad, max(ys) + pad]);
end

function rects = geometryRects(g)
beltRects = beltRects3D(g);
platformRect = [g.platformX - g.platformW/2, g.platformY - g.platformH/2, g.platformW, g.platformH];
rects = [
    beltRects{1}
    beltRects{2}
    beltRects{3}
    beltRects{4}
    waitAreaRect(g)
    platformRect
];
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
