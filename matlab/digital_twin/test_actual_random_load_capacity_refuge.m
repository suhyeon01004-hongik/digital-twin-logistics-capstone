function results = test_actual_random_load_capacity_refuge(startIds, maxStepsPerLoad, maxCirculationSteps)
%TEST_ACTUAL_RANDOM_LOAD_CAPACITY_REFUGE Repeated real load-action capacity tests.
% The load action uses the simulator's packageDims(id), target-floor choice,
% pusher/platform timing, background routing, and final B4 250 mm reserve.
rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

if nargin < 1 || isempty(startIds)
    startIds = 1;
end
if nargin < 2 || isempty(maxStepsPerLoad)
    maxStepsPerLoad = 16000;
end
if nargin < 3 || isempty(maxCirculationSteps)
    maxCirculationSteps = 28000;
end

outDir = fullfile(rootDir, 'outputs');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

results = struct('runId', {}, 'startId', {}, 'loaded', {}, 'loadFraction', {}, ...
    'minB4GapMm', {}, 'target', {}, 'stepsLoad', {}, 'stepsCirculation', {}, ...
    'success', {}, 'collision', {}, 'rotation', {}, 'refuge', {}, ...
    'waitTotal', {}, 'message', {});

best = [];
for runId = 1:numel(startIds)
    startId = startIds(runId);
    parcel_manual_core_step("reset", 0);
    if startId > 1
        parcel_manual_core_step("set_next_id", startId);
    end
    [S0, loadSteps] = runRealLoads(maxStepsPerLoad);
    parcel_manual_core_step("save_state", 0);
    targetId = chooseTargetForRefuge(S0);
    S = S0;
    circSteps = 0;
    if targetId > 0 && maxCirculationSteps > 0
        [S, circSteps] = runTarget(targetId, maxCirculationSteps);
    end
    success = targetId > 0 && S.statusCode == 0 && S.collisionFlag <= 0.5 && ...
        S.rotationFlag <= 0.5 && S.circCompleteTargetId == targetId && ...
        S.tempUnloadCount >= 1 && S.waitTotal >= 1;
    result = makeResult(runId, startId, S0, S, targetId, loadSteps, circSteps, success);
    results(end+1) = result; %#ok<AGROW>
    fprintf("actual run %02d start=%d loaded=%d load=%.1f%% b4gap=%.0fmm target=%d success=%d refuge=%d wait=%d col=%d rot=%d loadSteps=%d circSteps=%d msg=%s\n", ...
        runId, startId, result.loaded, 100 * result.loadFraction, result.minB4GapMm, ...
        targetId, success, result.refuge, result.waitTotal, result.collision, ...
        result.rotation, loadSteps, circSteps, string(result.message));
    if isempty(best) || result.loaded > best.result.loaded || ...
            (result.loaded == best.result.loaded && result.success > best.result.success)
        best.result = result;
        best.before = S0;
        best.after = S;
    end
end

save(fullfile(outDir, 'actual_random_load_capacity_refuge_results.mat'), 'results');
writetable(struct2table(results), fullfile(outDir, 'actual_random_load_capacity_refuge_results.csv'));
writeSummary(results, fullfile(outDir, 'actual_random_load_capacity_refuge_summary.txt'));

if ~isempty(best)
    renderSnapshot(best.before, fullfile(outDir, 'actual_random_load_capacity_best_before.png'));
    renderSnapshot(best.after, fullfile(outDir, 'actual_random_load_capacity_best_after.png'));
end
end

function [S, totalSteps] = runRealLoads(maxStepsPerLoad)
totalSteps = 0;
S = parcel_manual_core_step("snapshot", 0);
staleAttempts = 0;
for attempt = 1:S.maxPkg
    beforeLoaded = S.loadedCount;
    S = parcel_manual_core_step("load", 0);
    for stepCount = 0:20:maxStepsPerLoad
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
S = settleAfterLoading(S, 50000);
end

function S = settleAfterLoading(S, maxSteps)
for stepCount = 0:20:maxSteps
    if S.isIdle && S.collisionFlag <= 0.5 && S.rotationFlag <= 0.5
        return;
    end
    S = parcel_manual_core_step("stepn", 20);
end
end

function targetId = chooseTargetForRefuge(S)
targetId = 0;
ids = chooseB4BlockedTargets(S);
if ~isempty(ids)
    targetId = ids(1);
    return;
end
ids = S.ids(S.ids > 0 & S.belts == 3);
if ~isempty(ids)
    targetId = min(ids);
    return;
end
ids = S.ids(S.ids > 0);
if ~isempty(ids)
    targetId = min(ids);
end
end

function ids = chooseB4BlockedTargets(S)
ids = [];
for floor = 1:3
    idxs = find(S.ids > 0 & S.floors == floor & S.belts == 4);
    if numel(idxs) < 2
        continue;
    end
    tails = zeros(numel(idxs),1);
    for k = 1:numel(idxs)
        i = idxs(k);
        len = axisLengthLocal(4, S.boxLong(i), S.boxShort(i));
        tails(k) = S.pos(i) - len / 2;
    end
    [~, order] = sort(tails, 'ascend');
    sorted = idxs(order);
    ids = [ids; S.ids(sorted(2:end))]; %#ok<AGROW>
end
ids = unique(ids, 'stable');
end

function [S, steps] = runTarget(targetId, maxSteps)
parcel_manual_core_step("load_state", 0);
S = parcel_manual_core_step("unload", targetId);
steps = 0;
while steps < maxSteps
    S = parcel_manual_core_step("stepn", 20);
    steps = steps + 20;
    if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        return;
    end
end
end

function result = makeResult(runId, startId, S0, S, targetId, loadSteps, circSteps, success)
result.runId = runId;
result.startId = startId;
result.loaded = S0.loadedCount;
result.loadFraction = activeAxisUsed(S0) / (3 * S0.floorCapacityM);
result.minB4GapMm = minB4Gap(S0) * 1000;
result.target = targetId;
result.stepsLoad = loadSteps;
result.stepsCirculation = circSteps;
result.success = success;
result.collision = max(S0.collisionFlag, S.collisionFlag);
result.rotation = max(S0.rotationFlag, S.rotationFlag);
result.refuge = S.tempUnloadCount;
result.waitTotal = S.waitTotal;
result.message = string(S.message);
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

function writeSummary(results, filePath)
fid = fopen(filePath, 'w');
if fid < 0
    return;
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "Actual load-action random-size capacity + refuge/wait-area test\n");
fprintf(fid, "success criterion: idle, no collision, no rotation, refuge >= 1, target stored in wait area\n\n");
for k = 1:numel(results)
    r = results(k);
    fprintf(fid, "run %02d | start %d | loaded %d | load %.1f%% | min B4 gap %.0f mm | target %d | success %d | refuge %d | wait %d | loadSteps %d | circSteps %d | %s\n", ...
        r.runId, r.startId, r.loaded, 100 * r.loadFraction, r.minB4GapMm, ...
        r.target, r.success, r.refuge, r.waitTotal, r.stepsLoad, ...
        r.stepsCirculation, string(r.message));
end
end
