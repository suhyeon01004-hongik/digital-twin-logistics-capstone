function render_parcel_algorithm_preview()
%RENDER_PARCEL_ALGORITHM_PREVIEW Render a numbered-box HMI preview from logs.

rootDir = fileparts(mfilename('fullpath'));
outDir = fullfile(rootDir, 'outputs');
modelFile = fullfile(outDir, 'parcel_belt_algorithm_revised.slx');
gifFile = fullfile(outDir, 'parcel_belt_algorithm_preview.gif');
pngFile = fullfile(outDir, 'parcel_belt_algorithm_final.png');
snapshotFile = fullfile(outDir, 'parcel_belt_algorithm_unload_snapshot.png');

if exist(gifFile, 'file')
    delete(gifFile);
end
if exist(pngFile, 'file')
    delete(pngFile);
end
if exist(snapshotFile, 'file')
    delete(snapshotFile);
end

load_system(modelFile);
out = sim('parcel_belt_algorithm_revised');

ids = squeeze(out.ids);
floors = squeeze(out.floors);
belts = squeeze(out.belts);
x = squeeze(out.x);
y = squeeze(out.y);
phase = out.phase(:);
statusCode = out.status_code(:);
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

if isvector(ids)
    ids = reshape(ids, [], 1);
    floors = reshape(floors, [], 1);
    belts = reshape(belts, [], 1);
    x = reshape(x, [], 1);
    y = reshape(y, [], 1);
end

frameCount = numel(phase);
sampleCount = min(180, frameCount);
sampleIdx = unique(round(linspace(1, frameCount, sampleCount)));
colors = lines(18);

fig = figure('Visible', 'off', 'Color', [1 1 1], 'Position', [100 100 1180 760]);
ax = axes(fig, 'Position', [0.05 0.08 0.70 0.86]);
panel = axes(fig, 'Position', [0.78 0.08 0.20 0.86]);

for n = 1:numel(sampleIdx)
    k = sampleIdx(n);
    drawFrame(ax, panel, ids(:,k), floors(:,k), belts(:,k), x(:,k), y(:,k), ...
        phase(k), statusCode(k), activeFloor(k), activeBelt(k), ...
        targetId(k), targetFloor(k), targetBelt(k), loadedCount(k), unloadedCount(k), ...
        collisionFlag(k), rotationFlag(k), gapFlag(k), colors);

    drawnow;
    frame = getframe(fig);
    [im, map] = rgb2ind(frame2im(frame), 256);
    if n == 1
        imwrite(im, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.08);
    else
        imwrite(im, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.08);
    end
end

drawFrame(ax, panel, ids(:,end), floors(:,end), belts(:,end), x(:,end), y(:,end), ...
    phase(end), statusCode(end), activeFloor(end), activeBelt(end), ...
    targetId(end), targetFloor(end), targetBelt(end), loadedCount(end), unloadedCount(end), ...
    collisionFlag(end), rotationFlag(end), gapFlag(end), colors);
exportgraphics(fig, pngFile, 'Resolution', 160);

snapshotIdx = find(statusCode == 3, 1, 'first');
if isempty(snapshotIdx)
    snapshotIdx = find(statusCode == 2, 1, 'first');
end
if isempty(snapshotIdx)
    snapshotIdx = frameCount;
end
drawFrame(ax, panel, ids(:,snapshotIdx), floors(:,snapshotIdx), belts(:,snapshotIdx), x(:,snapshotIdx), y(:,snapshotIdx), ...
    phase(snapshotIdx), statusCode(snapshotIdx), activeFloor(snapshotIdx), activeBelt(snapshotIdx), ...
    targetId(snapshotIdx), targetFloor(snapshotIdx), targetBelt(snapshotIdx), loadedCount(snapshotIdx), unloadedCount(snapshotIdx), ...
    collisionFlag(snapshotIdx), rotationFlag(snapshotIdx), gapFlag(snapshotIdx), colors);
exportgraphics(fig, snapshotFile, 'Resolution', 160);
close(fig);
bdclose('all');

fprintf('Created %s\n', gifFile);
fprintf('Created %s\n', pngFile);
fprintf('Created %s\n', snapshotFile);
end

function drawFrame(ax, panel, ids, floors, belts, x, y, phase, statusCode, activeFloor, activeBelt, targetId, targetFloor, targetBelt, loadedCount, unloadedCount, collisionFlag, rotationFlag, gapFlag, colors)
displayActiveFloor = activeFloor;
displayActiveBelt = activeBelt;
if displayActiveBelt <= 0 && statusCode == 3 && targetFloor > 0
    displayActiveFloor = targetFloor;
    displayActiveBelt = 4;
end
if displayActiveBelt <= 0 && statusCode == 2 && targetFloor > 0 && targetBelt > 0
    displayActiveFloor = targetFloor;
    displayActiveBelt = targetBelt;
end

cla(ax);
hold(ax, 'on');
axis(ax, 'equal');
ax.Color = [1 1 1];
ax.XColor = [0 0 0];
ax.YColor = [0 0 0];
xlim(ax, [-1.0 0.65]);
ylim(ax, [-2.15 2.35]);
grid(ax, 'on');
ax.GridColor = [0.82 0.82 0.82];
ax.GridAlpha = 0.55;
title(ax, 'Digital Twin Preview: numbered parcels and belt state', 'Color', [0 0 0]);
xlabel(ax, 'x [m]', 'Color', [0 0 0]);
ylabel(ax, 'layout y [m]', 'Color', [0 0 0]);

for floor = 1:3
    drawBelts(ax, floor, displayActiveFloor, displayActiveBelt);
end

for i = 1:numel(ids)
    if ids(i) > 0 && floors(i) > 0 && belts(i) > 0 && y(i) > -9
        c = colors(max(1, min(size(colors,1), ids(i))), :);
        edgeColor = [0.12 0.12 0.12];
        lineWidth = 1.0;
        if ids(i) == targetId
            edgeColor = [1.0 0.0 0.0];
            lineWidth = 2.5;
        end
        rectangle(ax, 'Position', [x(i)-0.035 y(i)-0.035 0.07 0.07], ...
            'FaceColor', c, 'EdgeColor', edgeColor, 'LineWidth', lineWidth);
        text(ax, x(i), y(i), sprintf('%d', ids(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontWeight', 'bold', 'FontSize', 8, 'Color', [0 0 0]);
    end
end

hold(ax, 'off');

cla(panel);
panel.Color = [1 1 1];
axis(panel, 'off');
statusName = statusText(statusCode);
lineY = 0.96;
txt = [0 0 0];
text(panel, 0.02, lineY, 'Current System State', 'FontWeight', 'bold', 'FontSize', 13, 'Color', txt);
lineY = lineY - 0.08;
text(panel, 0.02, lineY, sprintf('Status: %s', statusName), 'FontSize', 11, 'Color', txt);
lineY = lineY - 0.07;
text(panel, 0.02, lineY, sprintf('Phase code: %.0f', phase), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.07;
text(panel, 0.02, lineY, sprintf('Moving: Floor %.0f / Belt %.0f', displayActiveFloor, displayActiveBelt), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.10;
text(panel, 0.02, lineY, 'Target Parcel', 'FontWeight', 'bold', 'FontSize', 12, 'Color', txt);
lineY = lineY - 0.07;
text(panel, 0.02, lineY, sprintf('ID: %.0f', targetId), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.07;
text(panel, 0.02, lineY, sprintf('Floor: %.0f', targetFloor), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.07;
text(panel, 0.02, lineY, sprintf('Belt: %.0f', targetBelt), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.10;
text(panel, 0.02, lineY, sprintf('Loaded: %.0f / Unloaded: %.0f', loadedCount, unloadedCount), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.10;
text(panel, 0.02, lineY, 'Safety Flags', 'FontWeight', 'bold', 'FontSize', 12, 'Color', txt);
lineY = lineY - 0.07;
text(panel, 0.02, lineY, sprintf('Collision: %.0f', collisionFlag), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.07;
text(panel, 0.02, lineY, sprintf('Rotation: %.0f', rotationFlag), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.07;
text(panel, 0.02, lineY, sprintf('Same-belt gap: %.0f', gapFlag), 'FontSize', 10, 'Color', txt);
lineY = lineY - 0.12;
text(panel, 0.02, lineY, 'Status map', 'FontWeight', 'bold', 'FontSize', 11, 'Color', txt);
lineY = lineY - 0.06;
text(panel, 0.02, lineY, '1 Loading', 'FontSize', 9, 'Color', txt);
lineY = lineY - 0.05;
text(panel, 0.02, lineY, '2 Circulation', 'FontSize', 9, 'Color', txt);
lineY = lineY - 0.05;
text(panel, 0.02, lineY, '3 Unloading', 'FontSize', 9, 'Color', txt);
lineY = lineY - 0.05;
text(panel, 0.02, lineY, '4 Complete', 'FontSize', 9, 'Color', txt);
end

function drawBelts(ax, floor, activeFloor, activeBelt)
belt_width = 0.25;
L1 = 0.51;
L3 = 0.51;
L4 = 1.11;
platform_y = 0.0;
stopper_x = -0.15;
stopper_y = platform_y + 0.30;
A_x = stopper_x - 0.03/2;
A_y = stopper_y;
floor_offset = (floor - 2) * 1.7;
belt3_right_x = A_x;
belt3_left_x = belt3_right_x - L3;
belt3_y = A_y + floor_offset;
belt3_connect_y = belt3_y + belt_width/2;
belt4_left_x = A_x;
belt4_right_x = A_x + belt_width;
belt4_x = (belt4_left_x + belt4_right_x)/2;
belt4_start_y = belt3_connect_y - 0.25;
belt4_end_y = belt3_connect_y + L4;
belt1_right_x = belt4_right_x;
belt1_left_x = belt1_right_x - L1;
belt1_y = belt4_end_y + belt_width/2;
belt2_x = belt1_left_x - belt_width/2;
belt2_top_y = belt1_y + belt_width/2;
belt2_bottom_y = belt3_connect_y;

drawSegment(ax, [belt4_x belt4_x], [belt4_start_y belt4_end_y], floor, 4, activeFloor, activeBelt);
drawSegment(ax, [belt1_right_x belt1_left_x], [belt1_y belt1_y], floor, 1, activeFloor, activeBelt);
drawSegment(ax, [belt2_x belt2_x], [belt2_top_y belt2_bottom_y], floor, 2, activeFloor, activeBelt);
drawSegment(ax, [belt3_left_x belt3_right_x], [belt3_y belt3_y], floor, 3, activeFloor, activeBelt);

text(ax, 0.42, belt3_y, sprintf('F%d', floor), 'FontWeight', 'bold', 'Color', [0 0 0]);
end

function drawSegment(ax, xs, ys, floor, belt, activeFloor, activeBelt)
if floor == activeFloor && belt == activeBelt
    color = [0.0 0.45 1.0];
    lw = 8;
else
    color = [0.68 0.68 0.68];
    lw = 5;
end
plot(ax, xs, ys, '-', 'Color', color, 'LineWidth', lw);
midX = mean(xs);
midY = mean(ys);
text(ax, midX, midY, sprintf('B%d', belt), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8, 'Color', [0 0 0]);
end

function name = statusText(code)
if code == 1
    name = 'LOADING';
elseif code == 2
    name = 'CIRCULATION';
elseif code == 3
    name = 'UNLOADING';
else
    name = 'COMPLETE';
end
end
