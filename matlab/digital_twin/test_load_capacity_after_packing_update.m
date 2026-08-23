function results = test_load_capacity_after_packing_update()
%TEST_LOAD_CAPACITY_AFTER_PACKING_UPDATE Load-only capacity check after packing update.

rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

outDir = fullfile(rootDir, 'outputs', 'packing_update_20260615');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

stamp = datestr(now, 'yyyymmdd_HHMMSS');
summaryPath = fullfile(outDir, sprintf('packing_update_capacity_%s.txt', stamp));
csvPath = fullfile(outDir, sprintf('packing_update_capacity_%s.csv', stamp));

cases = [
    struct('name', "baseline", 'preset', "baseline", 'startId', 1)
    struct('name', "small_dominant", 'preset', "small_dominant", 'startId', 4)
    struct('name', "large_stress", 'preset', "large_stress", 'startId', 2)
];

results = struct('name', {}, 'preset', {}, 'startId', {}, 'loaded', {}, ...
    'loadFraction', {}, 'collision', {}, 'rotation', {}, 'steps', {}, 'message', {});

for ci = 1:numel(cases)
    c = cases(ci);
    parcel_manual_config("reset");
    parcel_manual_config("preset", c.preset);
    parcel_manual_core_step("reset", 0);
    parcel_manual_core_step("set_next_id", c.startId);
    [S, steps] = runMeasuredLoads(c.startId);
    r = struct();
    r.name = c.name;
    r.preset = c.preset;
    r.startId = c.startId;
    r.loaded = S.loadedCount;
    r.loadFraction = activeAxisUsed(S) / (parcel_manual_config().floorCount * S.floorCapacityM);
    r.collision = S.collisionFlag;
    r.rotation = S.rotationFlag;
    r.steps = steps;
    r.message = string(S.message);
    results(end+1) = r; %#ok<AGROW>
    fprintf("%s loaded=%d load=%.1f%% col=%d rot=%d steps=%d msg=%s\n", ...
        r.name, r.loaded, 100*r.loadFraction, r.collision, r.rotation, r.steps, r.message);
end

writetable(struct2table(results), csvPath);
writeSummary(results, summaryPath, csvPath);
parcel_manual_config("reset");
end

function [S, totalSteps] = runMeasuredLoads(startId)
totalSteps = 0;
S = parcel_manual_core_step("snapshot", 0);
staleAttempts = 0;
for attempt = 1:S.maxPkg
    beforeLoaded = S.loadedCount;
    id = startId + attempt - 1;
    yolo = measuredYoloForId(id);
    S = parcel_digital_twin_io_adapter("apply_yolo_load", yolo, "normal");
    for stepCount = 0:20:18000
        S = parcel_manual_core_step("stepn", 20);
        totalSteps = totalSteps + 20;
        if S.isIdle || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
            break;
        end
    end
    if S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        return;
    end
    if S.loadedCount <= beforeLoaded
        staleAttempts = staleAttempts + 1;
    else
        staleAttempts = 0;
    end
    if staleAttempts >= 1 || startsWith(string(S.message), "LOAD FINALIZE BLOCKED")
        break;
    end
end
for stepCount = 0:20:60000
    if S.isIdle && S.collisionFlag <= 0.5 && S.rotationFlag <= 0.5
        return;
    end
    S = parcel_manual_core_step("stepn", 20);
    totalSteps = totalSteps + 20;
end
end

function yolo = measuredYoloForId(id)
cfg = parcel_manual_config();
boxType = deterministicBoxType(id, cfg.boxTypeWeights);
dimsM = cfg.packageSizeM(boxType, :) * cfg.packageScale;
yolo = parcel_digital_twin_io_adapter("yolo_template");
yolo.parcelId = id;
yolo.longMm = 1000 * max(dimsM(1), dimsM(2));
yolo.shortMm = 1000 * min(dimsM(1), dimsM(2));
yolo.heightMm = 1000 * dimsM(3);
yolo.yawDeg = -28 + 56 * mod(id * 22695477 + 1, 20001) / 20000;
yolo.confidence = 0.95;
end

function k = deterministicBoxType(id, weights)
weights = reshape(double(weights), 1, []);
weights = weights / sum(weights);
u = mod(id * 1103515245 + 12345, 100000) / 100000;
c = cumsum(weights);
k = find(u <= c, 1, 'first');
if isempty(k)
    k = numel(weights);
end
end

function used = activeAxisUsed(S)
used = 0;
for i = 1:numel(S.ids)
    if S.ids(i) > 0 && S.floors(i) > 0 && S.belts(i) > 0
        used = used + axisLengthLocal(S.belts(i), S.boxLong(i), S.boxShort(i));
    end
end
end

function len = axisLengthLocal(belt, longSide, shortSide)
if belt == 1 || belt == 3
    len = longSide;
else
    len = shortSide;
end
end

function writeSummary(results, summaryPath, csvPath)
fid = fopen(summaryPath, 'w');
if fid < 0
    return;
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, "Packing update load-only capacity check\n\n");
fprintf(fid, "CSV: %s\n\n", csvPath);
for i = 1:numel(results)
    r = results(i);
    fprintf(fid, "%s | loaded %d | load %.1f%% | collision %d | rotation %d | steps %d | %s\n", ...
        r.name, r.loaded, 100*r.loadFraction, r.collision, r.rotation, r.steps, string(r.message));
end
end
