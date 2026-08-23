function render_parcel_split_dashboard()
%RENDER_PARCEL_SPLIT_DASHBOARD Render split digital-twin dashboard preview.
%
% Left side: 3D-style whole-system view and current HMI status.
% Right side: three floor-specific 2D belt views, similar to the Python viewer.

rootDir = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(rootDir, 'outputs');
modelFile = fullfile(outDir, 'parcel_belt_algorithm_revised.slx');
gifFile = fullfile(outDir, 'parcel_split_dashboard_preview.gif');
pngFile = fullfile(outDir, 'parcel_split_dashboard_unload_snapshot.png');

if exist(gifFile, 'file')
    delete(gifFile);
end
if exist(pngFile, 'file')
    delete(pngFile);
end

load_system(modelFile);
out = sim('parcel_belt_algorithm_revised');

ids = squeeze(out.ids);
floors = squeeze(out.floors);
belts = squeeze(out.belts);
x = squeeze(out.x);
y = squeeze(out.y);
boxLong = squeeze(out.box_long);
boxShort = squeeze(out.box_short);
boxHeight = squeeze(out.box_height);
if isvector(ids)
    ids = reshape(ids, [], 1);
    floors = reshape(floors, [], 1);
    belts = reshape(belts, [], 1);
    x = reshape(x, [], 1);
    y = reshape(y, [], 1);
    boxLong = reshape(boxLong, [], 1);
    boxShort = reshape(boxShort, [], 1);
    boxHeight = reshape(boxHeight, [], 1);
end

statusCode = out.status_code(:);
phase = out.phase(:);
activeFloor = out.active_floor(:);
activeBelt = out.active_belt(:);
targetId = out.target_id(:);
targetFloor = out.target_floor(:);
targetBelt = out.target_belt(:);
loadedCount = out.loaded_count(:);
unloadedCount = out.unloaded_count(:);
collisionFlag = out.collision_violation(:);
rotationFlag = out.rotation_violation(:);
gapFlag = out.same_belt_gap_violation(:);
dtEncoder = squeeze(out.dt_encoder_m);
dtTofGap = squeeze(out.dt_tof_gap_m);
dtTofEmpty = squeeze(out.dt_tof_empty);
dtMotorCmd = squeeze(out.dt_motor_cmd);
dtPlatformFloor = out.dt_platform_floor(:);
dtPlatformZ = out.dt_platform_z(:);
dtCurrentId = out.dt_current_package_id(:);
dtCurrentLong = out.dt_current_box_long(:);
dtCurrentShort = out.dt_current_box_short(:);
dtCurrentHeight = out.dt_current_box_height(:);
if isvector(dtEncoder)
    dtEncoder = reshape(dtEncoder, [], 1);
    dtTofGap = reshape(dtTofGap, [], 1);
    dtTofEmpty = reshape(dtTofEmpty, [], 1);
    dtMotorCmd = reshape(dtMotorCmd, [], 1);
end

frameCount = numel(statusCode);
sampleIdx = unique(round(linspace(1, frameCount, min(60, frameCount))));
snapshotIdx = find(statusCode == 3, 1, 'first');
if isempty(snapshotIdx)
    snapshotIdx = find(statusCode == 2, 1, 'first');
end
if isempty(snapshotIdx)
    snapshotIdx = frameCount;
end

colorCount = max(72, max(ids(:)));
colors = lines(colorCount);
fig = figure('Visible', 'off', 'Color', [1 1 1], 'Position', [80 80 1500 850]);

ax3d = axes(fig, 'Position', [0.035 0.18 0.540 0.76]);
axGauge = axes(fig, 'Position', [0.020 0.790 0.160 0.170]);
axInfo = axes(fig, 'Position', [0.035 0.03 0.540 0.12]);
axF3 = axes(fig, 'Position', [0.600 0.735 0.380 0.220]);
axF2 = axes(fig, 'Position', [0.600 0.500 0.380 0.220]);
axF1 = axes(fig, 'Position', [0.600 0.265 0.380 0.220]);
axCamera = axes(fig, 'Position', [0.686 0.015 0.265 0.250]);

for n = 1:numel(sampleIdx)
    k = sampleIdx(n);
    drawDashboard(ax3d, axGauge, axCamera, axInfo, axF1, axF2, axF3, ...
        ids(:,k), floors(:,k), belts(:,k), x(:,k), y(:,k), ...
        boxLong(:,k), boxShort(:,k), boxHeight(:,k), ...
        phase(k), statusCode(k), activeFloor(k), activeBelt(k), ...
        targetId(k), targetFloor(k), targetBelt(k), ...
        loadedCount(k), unloadedCount(k), collisionFlag(k), rotationFlag(k), gapFlag(k), ...
        dtEncoder(:,k), dtTofGap(:,k), dtTofEmpty(:,k), dtMotorCmd(:,k), dtPlatformFloor(k), dtPlatformZ(k), ...
        dtCurrentId(k), dtCurrentLong(k), dtCurrentShort(k), dtCurrentHeight(k), colors);
    drawnow;
    frame = getframe(fig);
    [im, map] = rgb2ind(frame2im(frame), 256);
    if n == 1
        imwrite(im, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.08);
    else
        imwrite(im, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.08);
    end
end

drawDashboard(ax3d, axGauge, axCamera, axInfo, axF1, axF2, axF3, ...
    ids(:,snapshotIdx), floors(:,snapshotIdx), belts(:,snapshotIdx), x(:,snapshotIdx), y(:,snapshotIdx), ...
    boxLong(:,snapshotIdx), boxShort(:,snapshotIdx), boxHeight(:,snapshotIdx), ...
    phase(snapshotIdx), statusCode(snapshotIdx), activeFloor(snapshotIdx), activeBelt(snapshotIdx), ...
    targetId(snapshotIdx), targetFloor(snapshotIdx), targetBelt(snapshotIdx), ...
    loadedCount(snapshotIdx), unloadedCount(snapshotIdx), collisionFlag(snapshotIdx), rotationFlag(snapshotIdx), gapFlag(snapshotIdx), ...
    dtEncoder(:,snapshotIdx), dtTofGap(:,snapshotIdx), dtTofEmpty(:,snapshotIdx), dtMotorCmd(:,snapshotIdx), dtPlatformFloor(snapshotIdx), dtPlatformZ(snapshotIdx), ...
    dtCurrentId(snapshotIdx), dtCurrentLong(snapshotIdx), dtCurrentShort(snapshotIdx), dtCurrentHeight(snapshotIdx), colors);
exportgraphics(fig, pngFile, 'Resolution', 160);

close(fig);
bdclose('all');
fprintf('Created %s\n', gifFile);
fprintf('Created %s\n', pngFile);
end

function drawDashboard(ax3d, axGauge, axCamera, axInfo, axF1, axF2, axF3, ids, floors, belts, x, y, boxLong, boxShort, boxHeight, phase, statusCode, activeFloor, activeBelt, targetId, targetFloor, targetBelt, loadedCount, unloadedCount, collisionFlag, rotationFlag, gapFlag, dtEncoder, dtTofGap, dtTofEmpty, dtMotorCmd, dtPlatformFloor, dtPlatformZ, dtCurrentId, dtCurrentLong, dtCurrentShort, dtCurrentHeight, colors)
[displayActiveFloor, displayActiveBelt] = displayActive(statusCode, activeFloor, activeBelt, targetFloor, targetBelt);
platformFloor = inferPlatformFloor(statusCode, displayActiveFloor, targetFloor);

draw3DPanel(ax3d, ids, floors, belts, x, y, boxLong, boxShort, boxHeight, targetId, displayActiveFloor, displayActiveBelt, platformFloor, colors);
drawCapacityGaugePanel(axGauge, ids, floors, belts, boxLong, boxShort);
drawYoloCameraPanel(axCamera, statusCode, dtCurrentId, dtCurrentLong, dtCurrentShort, dtCurrentHeight);
drawInfoPanel(axInfo, phase, statusCode, displayActiveFloor, displayActiveBelt, platformFloor, targetId, targetFloor, targetBelt, loadedCount, unloadedCount, collisionFlag, rotationFlag, gapFlag, dtEncoder, dtTofEmpty, dtMotorCmd, dtPlatformFloor, dtPlatformZ);
drawFloorPanel(axF3, 3, ids, floors, belts, x, y, boxLong, boxShort, targetId, displayActiveFloor, displayActiveBelt, platformFloor, dtTofGap, dtTofEmpty, colors);
drawFloorPanel(axF2, 2, ids, floors, belts, x, y, boxLong, boxShort, targetId, displayActiveFloor, displayActiveBelt, platformFloor, dtTofGap, dtTofEmpty, colors);
drawFloorPanel(axF1, 1, ids, floors, belts, x, y, boxLong, boxShort, targetId, displayActiveFloor, displayActiveBelt, platformFloor, dtTofGap, dtTofEmpty, colors);
end

function [f, b] = displayActive(statusCode, activeFloor, activeBelt, targetFloor, targetBelt)
f = activeFloor;
b = activeBelt;
if b <= 0 && statusCode == 3 && targetFloor > 0
    f = targetFloor;
    b = 4;
elseif b <= 0 && statusCode == 2 && targetFloor > 0 && targetBelt > 0
    f = targetFloor;
    b = targetBelt;
end
end

function floor = inferPlatformFloor(statusCode, activeFloor, targetFloor)
if targetFloor >= 1 && targetFloor <= 3
    floor = targetFloor;
elseif activeFloor >= 1 && activeFloor <= 3
    floor = activeFloor;
elseif statusCode == 4
    floor = 1;
else
    floor = 1;
end
end

function draw3DPanel(ax, ids, floors, belts, x, y, boxLong, boxShort, boxHeight, targetId, activeFloor, activeBelt, platformFloor, colors)
cla(ax);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
ax.GridAlpha = 0.18;
ax.Color = [0.98 0.98 0.98];
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
zlabel(ax, 'floor z [m]');
title(ax, '3D Simulation', 'Color', [0 0 0]);
ax.XColor = [0 0 0];
ax.YColor = [0 0 0];
ax.ZColor = [0 0 0];
view(ax, 38, 24);
xlim(ax, [-0.95 0.25]);
ylim(ax, [-0.55 1.55]);
zlim(ax, [-0.03 0.78]);
g = geom();
labelFontSize = packageLabelFontSize(ids);
[drawX, drawY] = applyB1EntryGapVisual(x, y, ids, floors, belts, boxLong, boxShort);

for floor = 1:3
    z = floorHeight(floor);
    draw3DBelts(ax, floor, z, activeFloor, activeBelt);
end
draw3DPlatform(ax, g, floorHeight(platformFloor) - 0.02);

for i = 1:numel(ids)
    if ids(i) > 0 && floors(i) > 0 && belts(i) > 0 && drawY(i) > -9
        floor = floors(i);
        localY = drawY(i) - floorOffset(floor);
        baseZ = floorHeight(floor) + g.beltThick;
        c = packageTypeColor(boxLong(i));
        w = max(boxLong(i), 0.06);
        d = max(boxShort(i), 0.05);
        h = max(boxHeight(i), 0.035);
        edge = [0 0 0];
        lw = 1.0;
        face = c;
        if ids(i) == targetId
            edge = [1 0 0];
            lw = 2.2;
        end
        drawPackage3D(ax, drawX(i), localY, baseZ, w, d, h, face, edge, lw);
        text(ax, drawX(i), localY, baseZ + h + 0.025, sprintf('%d', ids(i)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
            'FontSize', labelFontSize, 'Color', [0 0 0], 'Clipping', 'on');
    end
end
for floor = 1:3
    draw3DInnerWalls(ax, floorHeight(floor) + g.beltThick + 0.003);
end
hold(ax, 'off');
end

function drawInfoPanel(ax, phase, statusCode, activeFloor, activeBelt, platformFloor, targetId, targetFloor, targetBelt, loadedCount, unloadedCount, collisionFlag, rotationFlag, gapFlag, dtEncoder, dtTofEmpty, dtMotorCmd, dtPlatformFloor, dtPlatformZ)
cla(ax);
axis(ax, 'off');
txt = [0 0 0];
statusName = statusText(statusCode);
activeSensor = find(abs(dtMotorCmd) > 0.5, 1, 'first');
activeEncoder = 0;
if ~isempty(activeSensor)
    activeEncoder = dtEncoder(activeSensor);
end
emptyCount = sum(dtTofEmpty > 0.5);
text(ax, 0.01, 0.78, sprintf('Status: %s', statusName), 'FontWeight', 'bold', 'FontSize', 13, 'Color', txt);
text(ax, 0.28, 0.78, sprintf('Phase %.0f', phase), 'FontSize', 11, 'Color', txt);
text(ax, 0.40, 0.78, sprintf('Moving F%.0f / B%.0f', activeFloor, activeBelt), 'FontSize', 11, 'Color', txt);
text(ax, 0.60, 0.78, sprintf('Platform F%.0f', platformFloor), 'FontSize', 11, 'Color', txt);
text(ax, 0.76, 0.78, sprintf('Target P%.0f: F%.0f B%.0f', targetId, targetFloor, targetBelt), 'FontSize', 11, 'Color', txt);
text(ax, 0.01, 0.30, sprintf('Loaded %.0f / Unloaded %.0f', loadedCount, unloadedCount), 'FontSize', 11, 'Color', txt);
text(ax, 0.28, 0.30, sprintf('Collision %.0f', collisionFlag), 'FontSize', 11, 'Color', txt);
text(ax, 0.44, 0.30, sprintf('Rotation %.0f', rotationFlag), 'FontSize', 11, 'Color', txt);
text(ax, 0.60, 0.30, sprintf('Compact gap %.0f', gapFlag), 'FontSize', 11, 'Color', txt);
text(ax, 0.01, 0.02, sprintf('DT sensors: encoder %.3f m | TOF empty %.0f/12 | motor channels %.0f | platform F%.0f z=%.2f m', ...
    activeEncoder, emptyCount, sum(abs(dtMotorCmd) > 0.5), dtPlatformFloor, dtPlatformZ), 'FontSize', 9, 'Color', [0.25 0.25 0.25]);
end

function drawFloorPanel(ax, floor, ids, floors, belts, x, y, boxLong, boxShort, targetId, activeFloor, activeBelt, platformFloor, dtTofGap, dtTofEmpty, colors)
cla(ax);
hold(ax, 'on');
axis(ax, 'equal');
ax.Color = [1 1 1];
xlim(ax, [-0.62 1.68]);
ylim(ax, [-0.33 1.05]);
grid(ax, 'on');
ax.GridAlpha = 0.18;
ax.XColor = [0 0 0];
ax.YColor = [0 0 0];
set(ax, 'XTick', [], 'YTick', []);
text(ax, 0.50, 0.915, sprintf('Floor %d 2D Belt State', floor), ...
    'Units', 'normalized', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontSize', 11, ...
    'Color', [0 0 0], 'Clipping', 'off');
labelFontSize = packageLabelFontSize(ids);
[drawX, drawY] = applyB1EntryGapVisual(x, y, ids, floors, belts, boxLong, boxShort);

draw2DBelts(ax, activeFloor == floor, activeBelt);
if platformFloor == floor
    draw2DPlatform(ax, geom());
end
drawZones(ax, floor, dtTofGap, dtTofEmpty);

for i = 1:numel(ids)
    if ids(i) > 0 && floors(i) == floor && belts(i) > 0 && drawY(i) > -9
        localY = drawY(i) - floorOffset(floor);
        c = packageTypeColor(boxLong(i));
        edge = [0 0 0];
        lw = 1.1;
        face = c;
        if ids(i) == targetId
            edge = [1 0 0];
            lw = 2.4;
        end
        w = max(boxLong(i), 0.06);
        h = max(boxShort(i), 0.05);
        rect = rotate2DRect([drawX(i)-w/2 localY-h/2 w h]);
        [tx, ty] = rotate2DPoint(drawX(i), localY);
        rectangle(ax, 'Position', rect, 'Curvature', 0.03, 'FaceColor', face, 'EdgeColor', edge, 'LineWidth', lw);
        text(ax, tx, ty, sprintf('%d', ids(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontWeight', 'bold', 'FontSize', labelFontSize, 'Color', [0 0 0], ...
            'Clipping', 'on');
    end
end
draw2DInnerWalls(ax);

text(ax, 1.48, 0.90, sprintf('F%d', floor), ...
    'FontWeight', 'bold', 'FontSize', 11, 'Color', [0 0 0], 'Clipping', 'on');
hold(ax, 'off');
end

function draw2DBelts(ax, floorIsActive, activeBelt)
[g] = geom();
draw2DBeltRect(ax, [g.b4Left, g.b4Start, g.beltWidth, g.b4End - g.b4Start], floorIsActive, activeBelt, 4);
draw2DBeltRect(ax, [g.b1Left, g.b1y - g.beltWidth/2, g.b1Right - g.b1Left, g.beltWidth], floorIsActive, activeBelt, 1);
draw2DBeltRect(ax, [g.b2x - g.beltWidth/2, g.b2Bottom, g.beltWidth, g.b2Top - g.b2Bottom], floorIsActive, activeBelt, 2);
draw2DBeltRect(ax, [g.b3Left, g.b3y - g.beltWidth/2, g.b3Right - g.b3Left, g.beltWidth], floorIsActive, activeBelt, 3);
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

function draw2DBeltRect(ax, rect, floorIsActive, activeBelt, belt)
if floorIsActive && activeBelt == belt
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

function [drawX, drawY] = applyB1EntryGapVisual(x, y, ids, floors, belts, boxLong, boxShort)
drawX = x;
drawY = y;
g = geom();
for floor = 1:3
    idx = find(ids > 0 & floors == floor & belts == 1 & y > -9);
    if isempty(idx)
        continue;
    end
    desired = zeros(numel(idx), 1);
    lens = zeros(numel(idx), 1);
    for n = 1:numel(idx)
        i = idx(n);
        lens(n) = max(boxLong(i), 0.06);
        desired(n) = x(i) + b1EntrySlack(ids(i), boxLong(i));
    end
    [~, order] = sort(desired, 'ascend');
    centers = desired;
    prevEnd = g.b1Left;
    for n = 1:numel(order)
        oi = order(n);
        len = lens(oi);
        center = min(max(desired(oi), g.b1Left + len/2), g.b1Right - len/2);
        center = max(center, prevEnd + len/2);
        center = min(center, g.b1Right - len/2);
        centers(oi) = center;
        prevEnd = center + len/2;
    end
    nextStart = g.b1Right;
    for n = numel(order):-1:1
        oi = order(n);
        len = lens(oi);
        center = min(centers(oi), nextStart - len/2);
        center = max(center, g.b1Left + len/2);
        centers(oi) = center;
        nextStart = center - len/2;
    end
    for n = 1:numel(idx)
        drawX(idx(n)) = centers(n);
    end
end
end

function dx = b1EntrySlack(packageId, longSide)
g = geom();
available = max(0, g.beltWidth - max(longSide, 0.06));
dx = (platformLaneFactor(packageId) - 0.5) * available;
end

function drawZones(ax, floor, dtTofGap, dtTofEmpty)
[g] = geom();
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
    [tx, ty] = rotate2DPoint(zones(i,1) + zones(i,3)/2, zones(i,2) + zones(i,4)/2);
    rectangle(ax, 'Position', rect, 'EdgeColor', edgeColor, 'FaceColor', zoneColor, 'FaceAlpha', 0.20, 'LineStyle', '--');
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

function draw3DBelts(ax, floor, z, activeFloor, activeBelt)
[g] = geom();
draw3DBeltSurface(ax, [g.b4Left, g.b4Start, g.beltWidth, g.b4End - g.b4Start], z, activeFloor == floor, activeBelt, 4);
draw3DBeltSurface(ax, [g.b1Left, g.b1y - g.beltWidth/2, g.b1Right - g.b1Left, g.beltWidth], z, activeFloor == floor, activeBelt, 1);
draw3DBeltSurface(ax, [g.b2x - g.beltWidth/2, g.b2Bottom, g.beltWidth, g.b2Top - g.b2Bottom], z, activeFloor == floor, activeBelt, 2);
draw3DBeltSurface(ax, [g.b3Left, g.b3y - g.beltWidth/2, g.b3Right - g.b3Left, g.beltWidth], z, activeFloor == floor, activeBelt, 3);
text(ax, 0.25, g.b1y, z + 0.025, sprintf('F%d', floor), ...
    'FontWeight', 'bold', 'Color', [0 0 0], 'Clipping', 'on');
end

function draw3DInnerWalls(ax, z)
g = geom();
wallColor = [0.04 0.05 0.06];
wallWidth = 2.2;
plot3(ax, [g.b4Right g.b4Right], [g.b4Start g.b4End], [z z], '-', ...
    'Color', wallColor, 'LineWidth', wallWidth, 'Clipping', 'on');
plot3(ax, [g.b1Left g.b1Right], [g.b1y-g.beltWidth/2 g.b1y-g.beltWidth/2], [z z], '-', ...
    'Color', wallColor, 'LineWidth', wallWidth, 'Clipping', 'on');
plot3(ax, [g.b2x-g.beltWidth/2 g.b2x-g.beltWidth/2], [g.b2Bottom g.b2Top], [z z], '-', ...
    'Color', wallColor, 'LineWidth', wallWidth, 'Clipping', 'on');
plot3(ax, [g.b3Left g.b3Right], [g.b3y+g.beltWidth/2 g.b3y+g.beltWidth/2], [z z], '-', ...
    'Color', wallColor, 'LineWidth', wallWidth, 'Clipping', 'on');
end

function draw3DPlatform(ax, g, z)
rect = [g.platformX - g.platformW/2, g.platformY - g.platformH/2, g.platformW, g.platformH];
draw3DPlate(ax, rect, z, 0.008, [0.86 0.88 0.90], [0.45 0.48 0.52], 1.0);
end

function draw3DBeltSurface(ax, rect, z, floorIsActive, activeBelt, belt)
g = geom();
if floorIsActive && activeBelt == belt
    color = [0 0.42 1.0];
    edge = [0 0.20 0.55];
else
    color = [0.62 0.62 0.62];
    edge = [0.35 0.35 0.35];
end
draw3DPlate(ax, rect, z, g.beltThick, color, edge, 1.0);
text(ax, rect(1) + rect(3)/2, rect(2) + rect(4)/2, z + 0.02, sprintf('B%d', belt), ...
    'HorizontalAlignment', 'center', 'FontSize', 7, 'FontWeight', 'bold', 'Color', [0 0 0], 'Clipping', 'on');
end

function draw3DPlate(ax, rect, z, thickness, face, edge, lw)
x0 = rect(1); y0 = rect(2); w = rect(3); h = rect(4);
z0 = z;
z1 = z + thickness;
vertices = [
    x0,   y0,   z0;
    x0+w, y0,   z0;
    x0+w, y0+h, z0;
    x0,   y0+h, z0;
    x0,   y0,   z1;
    x0+w, y0,   z1;
    x0+w, y0+h, z1;
    x0,   y0+h, z1
];
faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];
patch(ax, 'Vertices', vertices, 'Faces', faces, ...
    'FaceColor', face, 'EdgeColor', edge, 'LineWidth', lw, 'FaceAlpha', 0.95);
end

function drawPackage3D(ax, cx, cy, baseZ, w, d, h, face, edge, lw)
rect = [cx - w/2, cy - d/2, w, d];
draw3DPlate(ax, rect, baseZ, h, face, edge, lw);
drawCuboidEdges3D(ax, rect, baseZ, h, edge, max(lw, 1.4));
end

function drawCapacityGaugePanel(ax, ids, floors, belts, boxLong, boxShort)
[used, capacity, percent, floorCounts, floorUsed, floorCapacity, floorPercent] = capacityMetrics(ids, floors, belts, boxLong, boxShort);
cla(ax);
hold(ax, 'on');
axis(ax, 'off');
xlim(ax, [0 1]);
ylim(ax, [0 1]);
rectangle(ax, 'Position', [0.00 0.00 1.00 1.00], ...
    'FaceColor', [1 1 1], 'EdgeColor', [0.75 0.78 0.82], 'LineWidth', 1.0);
text(ax, 0.04, 0.91, 'Load Gauge', ...
    'FontWeight', 'bold', 'FontSize', 8, 'Color', [0 0 0], 'HorizontalAlignment', 'left');
drawGaugeRow2D(ax, 0.04, 0.74, 0.92, 0.10, percent, [0.10 0.58 0.95], ...
    sprintf('Total %.0f%%  %d boxes', percent * 100, sum(floorCounts)));
for floor = 1:3
    y = 0.74 - 0.18 * floor;
    drawGaugeRow2D(ax, 0.04, y, 0.92, 0.10, floorPercent(floor), floorGaugeColor(floor), ...
        sprintf('F%d %.0f%%  %d boxes', floor, floorPercent(floor) * 100, floorCounts(floor)));
end
hold(ax, 'off');
end

function drawGaugeRow2D(ax, x, y, w, h, percent, color, label)
rectangle(ax, 'Position', [x y w h], ...
    'FaceColor', [0.93 0.94 0.96], 'EdgeColor', [0.45 0.48 0.52], 'LineWidth', 0.8);
rectangle(ax, 'Position', [x y w * min(max(percent, 0), 1) h], ...
    'FaceColor', color, 'EdgeColor', 'none');
text(ax, x, y + h + 0.020, label, ...
    'FontWeight', 'bold', 'FontSize', 6, 'Color', [0 0 0], 'HorizontalAlignment', 'left');
end

function drawYoloCameraPanel(ax, statusCode, packageId, longSide, shortSide, boxHeight)
cla(ax);
hold(ax, 'on');
axis(ax, 'off');
xlim(ax, [0 1]);
ylim(ax, [0 1]);
rectangle(ax, 'Position', [0 0 1 1], ...
    'FaceColor', [0.08 0.09 0.10], 'EdgeColor', [0.15 0.17 0.19], 'LineWidth', 1.2);
for gx = 0.12:0.16:0.92
    plot(ax, [gx gx], [0.12 0.88], '-', 'Color', [0.16 0.17 0.18], 'LineWidth', 0.6);
end
for gy = 0.16:0.16:0.84
    plot(ax, [0.08 0.92], [gy gy], '-', 'Color', [0.16 0.17 0.18], 'LineWidth', 0.6);
end

if packageId <= 0 || longSide <= 0 || shortSide <= 0
    text(ax, 0.50, 0.52, 'YOLO CAMERA', 'Color', [0.75 0.80 0.85], ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 10);
    text(ax, 0.50, 0.42, 'WAITING FOR BOX', 'Color', [0.45 0.50 0.55], ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 8);
    hold(ax, 'off');
    return;
end

yawDeg = yoloYawDeg(packageId, statusCode);
theta = yawDeg * pi / 180;
boxW = min(0.66, max(0.30, longSide / 0.24 * 0.58));
boxH = min(0.48, max(0.20, shortSide / 0.19 * 0.38));
cx = 0.34 + 0.32 * platformLaneFactor(packageId);
cy = 0.50;
[px, py] = rotatedRect(cx, cy, boxW, boxH, theta);
face = packageTypeColor(longSide);
patch(ax, px, py, face, 'EdgeColor', [0.02 0.02 0.02], 'LineWidth', 1.4, 'FaceAlpha', 0.88);
plot(ax, [px; px(1)], [py; py(1)], '-', 'Color', [0.20 1.00 0.25], 'LineWidth', 2.0);
plot(ax, cx, cy, '+', 'Color', [1 1 1], 'LineWidth', 1.0, 'MarkerSize', 8);
for i = 1:4
    plot(ax, px(i), py(i), 'o', 'Color', [0.20 1.00 0.25], ...
        'MarkerFaceColor', [0.20 1.00 0.25], 'MarkerSize', 3);
end

if statusCode == 1
    liveText = 'LIVE';
    liveColor = [0.10 1.00 0.25];
else
    liveText = 'HOLD';
    liveColor = [1.00 0.72 0.18];
end
text(ax, 0.05, 0.93, 'Platform Camera / YOLO', ...
    'Color', [0.95 0.98 1.00], 'FontWeight', 'bold', 'FontSize', 8, 'HorizontalAlignment', 'left');
text(ax, 0.93, 0.93, liveText, ...
    'Color', liveColor, 'FontWeight', 'bold', 'FontSize', 8, 'HorizontalAlignment', 'right');
text(ax, 0.05, 0.08, sprintf('P%d  Type %d', packageId, packageTypeFromLongSide(longSide)), ...
    'Color', [0.95 0.98 1.00], 'FontWeight', 'bold', 'FontSize', 7, 'HorizontalAlignment', 'left');
text(ax, 0.95, 0.08, sprintf('%.0f x %.0f mm  %.1f deg', longSide*1000, shortSide*1000, yawDeg), ...
    'Color', [0.95 0.98 1.00], 'FontWeight', 'bold', 'FontSize', 7, 'HorizontalAlignment', 'right');
text(ax, 0.05, 0.84, sprintf('h %.0f mm', boxHeight * 1000), ...
    'Color', [0.72 0.82 0.90], 'FontSize', 7, 'HorizontalAlignment', 'left');
hold(ax, 'off');
end

function yawDeg = yoloYawDeg(packageId, statusCode)
yawDeg = 0.9 * sin(packageId * 1.7 + statusCode * 0.3);
end

function r = platformLaneFactor(packageId)
seed = mod(packageId * 1664525 + 1013904223, 10000);
r = seed / 9999;
r = min(max(r, 0.08), 0.92);
end

function [px, py] = rotatedRect(cx, cy, w, h, theta)
corners = [
    -w/2, -h/2;
     w/2, -h/2;
     w/2,  h/2;
    -w/2,  h/2
];
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
rot = corners * R';
px = rot(:,1) + cx;
py = rot(:,2) + cy;
end

function drawCapacityGauge3D(ax, ids, floors, belts, boxLong, boxShort)
[used, capacity, percent, floorCounts, floorUsed, floorCapacity, floorPercent] = capacityMetrics(ids, floors, belts, boxLong, boxShort);
x0 = -0.92;
y0 = 1.44;
z0 = 0.665;
w = 0.54;
h = 0.032;
rowGap = 0.065;
drawGaugeRow3D(ax, x0, y0, z0, w, h, percent, [0.10 0.58 0.95], ...
    sprintf('Total %.2f / %.2f m  %.0f%%', used, capacity, percent * 100));
for floor = 1:3
    yy = y0 - rowGap * floor;
    drawGaugeRow3D(ax, x0, yy, z0, w, h, floorPercent(floor), floorGaugeColor(floor), ...
        sprintf('F%d %.2f / %.2f m  %.0f%%   %d boxes', floor, floorUsed(floor), floorCapacity, floorPercent(floor) * 100, floorCounts(floor)));
end
end

function drawGaugeRow3D(ax, x0, y0, z0, w, h, percent, color, label)
fill3(ax, [x0 x0+w x0+w x0], [y0 y0 y0+h y0+h], [z0 z0 z0 z0], ...
    [0.95 0.96 0.97], 'EdgeColor', [0.30 0.32 0.35], 'LineWidth', 0.8, 'FaceAlpha', 0.93);
fillW = w * min(max(percent, 0), 1);
fill3(ax, [x0 x0+fillW x0+fillW x0], [y0 y0 y0+h y0+h], [z0+0.002 z0+0.002 z0+0.002 z0+0.002], ...
    color, 'EdgeColor', 'none', 'FaceAlpha', 0.88);
text(ax, x0, y0 + h + 0.006, z0 + 0.004, label, ...
    'FontWeight', 'bold', 'FontSize', 7, 'Color', [0 0 0], 'Clipping', 'on');
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

function [used, capacity, percent, floorCounts, floorUsed, floorCapacity, floorPercent] = capacityMetrics(ids, floors, belts, boxLong, boxShort)
used = 0;
floorCounts = zeros(3,1);
floorUsed = zeros(3,1);
floorCapacity = beltLengthForDisplay(1) + beltLengthForDisplay(2) + beltLengthForDisplay(3) + beltLengthForDisplay(4);
for i = 1:numel(ids)
    if ids(i) > 0 && floors(i) >= 1 && floors(i) <= 3 && belts(i) >= 1 && belts(i) <= 4
        len = axisLengthForBelt(belts(i), boxLong(i), boxShort(i));
        used = used + len;
        floorUsed(floors(i)) = floorUsed(floors(i)) + len;
        floorCounts(floors(i)) = floorCounts(floors(i)) + 1;
    end
end
capacity = 3 * floorCapacity;
percent = used / capacity;
floorPercent = floorUsed / floorCapacity;
end

function occupancy = floorBeltOccupancy(floor, ids, floors, belts, boxLong, boxShort)
occupancy = zeros(4,1);
for b = 1:4
    used = 0;
    for i = 1:numel(ids)
        if ids(i) > 0 && floors(i) == floor && belts(i) == b
            used = used + axisLengthForBelt(b, boxLong(i), boxShort(i));
        end
    end
    occupancy(b) = min(1, used / beltLengthForDisplay(b));
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
boxType = packageTypeFromLongSide(longSide);
palette = [
    0.20 0.58 0.95;  % Type 1
    0.95 0.48 0.16;  % Type 2
    0.22 0.68 0.30;  % Type 3
    0.82 0.20 0.70;  % Type 4
    0.95 0.73 0.18   % Type 5
];
color = palette(boxType, :);
end

function boxType = packageTypeFromLongSide(longSide)
longMm = round(longSide * 1000);
if longMm <= 122
    boxType = 1;
elseif longMm <= 152
    boxType = 2;
elseif longMm <= 188
    boxType = 3;
elseif longMm <= 222
    boxType = 4;
else
    boxType = 5;
end
end

function len = axisLengthForBelt(belt, longSide, shortSide)
if belt == 1 || belt == 3
    len = longSide;
else
    len = shortSide;
end
end

function len = beltLengthForDisplay(belt)
if belt == 1
    len = 0.51;
elseif belt == 2
    len = 1.36;
elseif belt == 3
    len = 0.51;
else
    len = 1.11;
end
end

function drawCuboidEdges3D(ax, rect, z, thickness, color, lw)
x0 = rect(1); y0 = rect(2); w = rect(3); h = rect(4);
z0 = z;
z1 = z + thickness;
vertices = [
    x0,   y0,   z0;
    x0+w, y0,   z0;
    x0+w, y0+h, z0;
    x0,   y0+h, z0;
    x0,   y0,   z1;
    x0+w, y0,   z1;
    x0+w, y0+h, z1;
    x0,   y0+h, z1
];
edges = [
    1 2; 2 3; 3 4; 4 1;
    5 6; 6 7; 7 8; 8 5;
    1 5; 2 6; 3 7; 4 8
];
for i = 1:size(edges, 1)
    p = vertices(edges(i,:), :);
    plot3(ax, p(:,1), p(:,2), p(:,3), '-', ...
        'Color', color, 'LineWidth', lw, 'Clipping', 'on');
end
end

function g = geom()
beltWidth = 0.25;
g.beltWidth = beltWidth;
g.beltThick = 0.015;
L1 = 0.51;
L3 = 0.51;
L4 = 1.11;
platformY = -0.35;
g.platformX = 0.0;
g.platformY = platformY;
g.platformW = 0.34;
g.platformH = 0.35;
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
g.b4Start = g.b3Connect - 0.25;
g.b4End = g.b3Connect + L4;
g.b1Right = g.b4Right;
g.b1Left = g.b1Right - L1;
g.b1y = g.b4End + beltWidth/2;
g.b2x = g.b1Left - beltWidth/2;
g.b2Top = g.b1y + beltWidth/2;
g.b2Bottom = g.b3Connect;

mirrorCenterX = (g.b4x + g.b2x) / 2;
g.platformX = mirrorX(g.platformX, mirrorCenterX);
[g.b3Left, g.b3Right] = mirrorRange(g.b3Left, g.b3Right, mirrorCenterX);
[g.b4Left, g.b4Right] = mirrorRange(g.b4Left, g.b4Right, mirrorCenterX);
[g.b1Left, g.b1Right] = mirrorRange(g.b1Left, g.b1Right, mirrorCenterX);
g.b4x = mirrorX(g.b4x, mirrorCenterX);
g.b2x = mirrorX(g.b2x, mirrorCenterX);
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

function idx = sensorIndex(floor, belt)
idx = (floor - 1) * 4 + belt;
end

function v = floorOffset(floor)
v = (floor - 2) * 1.7;
end

function z = floorHeight(floor)
if floor == 1
    z = 0.00;
elseif floor == 2
    z = 0.27;
else
    z = 0.54;
end
end

function s = statusText(code)
if code == 1
    s = 'LOADING';
elseif code == 2
    s = 'CIRCULATION';
elseif code == 3
    s = 'UNLOADING';
else
    s = 'COMPLETE';
end
end
