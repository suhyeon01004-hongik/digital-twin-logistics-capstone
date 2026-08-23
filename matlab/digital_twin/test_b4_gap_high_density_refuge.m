function result = test_b4_gap_high_density_refuge()
%TEST_B4_GAP_HIGH_DENSITY_REFUGE Dense SEQ max-load refuge/wait-area check.
rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

outDir = fullfile(rootDir, 'outputs');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

parcel_manual_core_step("reset", 0);
for floor = 1:3
    for n = 1:80
        S = parcel_manual_core_step("seq_package", [floor 115 85 80]);
        if startsWith(string(S.message), "SEQ BLOCKED")
            break;
        end
    end
end
S0 = parcel_manual_core_step("snapshot", 0);
targetId = 16;
S = parcel_manual_core_step("unload", targetId);
steps = 0;
while steps < 26000
    S = parcel_manual_core_step("stepn", 20);
    steps = steps + 20;
    if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        break;
    end
end

result.loaded = S0.loadedCount;
result.loadFraction = activeAxisUsed(S0) / (3 * S0.floorCapacityM);
result.minB4GapMm = minB4Gap(S0) * 1000;
result.target = targetId;
result.steps = steps;
result.success = S.statusCode == 0 && S.collisionFlag <= 0.5 && ...
    S.rotationFlag <= 0.5 && S.circCompleteTargetId == targetId && ...
    S.tempUnloadCount >= 1 && S.waitTotal >= 1;
result.collision = S.collisionFlag;
result.rotation = S.rotationFlag;
result.refuge = S.tempUnloadCount;
result.waitTotal = S.waitTotal;
result.message = string(S.message);

save(fullfile(outDir, 'b4_gap_high_density_refuge_result.mat'), 'result');
writeSummary(result, fullfile(outDir, 'b4_gap_high_density_refuge_summary.txt'));
renderSnapshot(S0, fullfile(outDir, 'b4_gap_high_density_before.png'));
renderSnapshot(S, fullfile(outDir, 'b4_gap_high_density_wait_complete.png'));
end

function used = activeAxisUsed(S)
used = 0;
for i = 1:numel(S.ids)
    if S.ids(i) > 0 && S.floors(i) > 0 && S.belts(i) > 0
        used = used + axisLengthLocal(S.belts(i), S.boxLong(i), S.boxShort(i));
    end
end
end

function gap = minB4Gap(S)
gap = inf;
for floor = 1:3
    floorGap = 1.105;
    idxs = find(S.ids > 0 & S.floors == floor & S.belts == 4);
    for k = 1:numel(idxs)
        i = idxs(k);
        len = axisLengthLocal(4, S.boxLong(i), S.boxShort(i));
        floorGap = min(floorGap, S.pos(i) - len / 2);
    end
    gap = min(gap, floorGap);
end
end

function len = axisLengthLocal(belt, longSide, shortSide)
if belt == 1 || belt == 3
    len = longSide;
else
    len = shortSide;
end
end

function renderSnapshot(S, filePath)
set(0, 'DefaultFigureVisible', 'off');
parcel_manual_animation_update([]);
parcel_manual_animation_update(S);
figs = findall(0, 'Type', 'figure');
if ~isempty(figs)
    exportgraphics(figs(1), filePath, 'Resolution', 150);
end
parcel_manual_animation_update([]);
end

function writeSummary(result, filePath)
fid = fopen(filePath, 'w');
if fid < 0
    return;
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "Dense B4 250mm gap refuge/wait-area test\n");
fprintf(fid, "loaded %d | load %.1f%% | min B4 gap %.0f mm | target %d | success %d | refuge %d | wait %d | collision %d | rotation %d | steps %d | %s\n", ...
    result.loaded, 100 * result.loadFraction, result.minB4GapMm, result.target, ...
    result.success, result.refuge, result.waitTotal, result.collision, ...
    result.rotation, result.steps, string(result.message));
end
