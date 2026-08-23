function results = test_actual_random_target_scan(startId, maxCandidates, maxStepsPerLoad, maxCirculationSteps)
%TEST_ACTUAL_RANDOM_TARGET_SCAN Build one actual random-load layout and scan unload targets.
rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

if nargin < 1 || isempty(startId)
    startId = 1;
end
if nargin < 2 || isempty(maxCandidates)
    maxCandidates = 8;
end
if nargin < 3 || isempty(maxStepsPerLoad)
    maxStepsPerLoad = 16000;
end
if nargin < 4 || isempty(maxCirculationSteps)
    maxCirculationSteps = 32000;
end

outDir = fullfile(rootDir, 'outputs');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

parcel_manual_core_step("reset", 0);
if startId > 1
    parcel_manual_core_step("set_next_id", startId);
end
[S0, loadSteps] = runRealLoads(maxStepsPerLoad);
parcel_manual_core_step("save_state", 0);
candidates = chooseCandidateTargets(S0);
candidates = candidates(1:min(maxCandidates, numel(candidates)));

results = struct('startId', {}, 'loaded', {}, 'loadFraction', {}, ...
    'minB4GapMm', {}, 'target', {}, 'stepsLoad', {}, 'stepsCirculation', {}, ...
    'success', {}, 'collision', {}, 'rotation', {}, 'refuge', {}, ...
    'waitTotal', {}, 'message', {});

bestAfter = [];
for k = 1:numel(candidates)
    targetId = candidates(k);
    parcel_manual_core_step("load_state", 0);
    [S, circSteps] = runTarget(targetId, maxCirculationSteps);
    success = S.statusCode == 0 && S.collisionFlag <= 0.5 && ...
        S.rotationFlag <= 0.5 && S.circCompleteTargetId == targetId && ...
        S.waitTotal >= 1 && S.tempUnloadCount >= 1;
    results(end+1) = makeResult(startId, S0, S, targetId, loadSteps, circSteps, success); %#ok<AGROW>
    fprintf("scan start=%d cand=%02d target=%d loaded=%d load=%.1f%% success=%d refuge=%d wait=%d col=%d rot=%d circSteps=%d msg=%s\n", ...
        startId, k, targetId, S0.loadedCount, ...
        100 * activeAxisUsed(S0) / (3 * S0.floorCapacityM), success, ...
        S.tempUnloadCount, S.waitTotal, S.collisionFlag, S.rotationFlag, ...
        circSteps, string(S.message));
    if success && isempty(bestAfter)
        bestAfter = S;
    end
end

save(fullfile(outDir, sprintf('actual_random_target_scan_start_%d.mat', startId)), 'results');
writetable(struct2table(results), fullfile(outDir, sprintf('actual_random_target_scan_start_%d.csv', startId)));
writeSummary(results, fullfile(outDir, sprintf('actual_random_target_scan_start_%d_summary.txt', startId)));
renderSnapshot(S0, fullfile(outDir, sprintf('actual_random_target_scan_start_%d_before.png', startId)));
if ~isempty(bestAfter)
    renderSnapshot(bestAfter, fullfile(outDir, sprintf('actual_random_target_scan_start_%d_success.png', startId)));
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
end

function ids = chooseCandidateTargets(S)
ids = [];
ids = [ids; chooseB4BlockedTargets(S)]; %#ok<AGROW>
for belt = [3 2 1 4]
    beltIds = S.ids(S.ids > 0 & S.belts == belt);
    beltIds = sort(beltIds(:), 'descend');
    ids = [ids; beltIds]; %#ok<AGROW>
end
ids = unique(ids, 'stable');
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

function result = makeResult(startId, S0, S, targetId, loadSteps, circSteps, success)
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
fprintf(fid, "Actual random-load target scan\n\n");
for k = 1:numel(results)
    r = results(k);
    fprintf(fid, "target %d | loaded %d | load %.1f%% | B4 gap %.0f mm | success %d | refuge %d | wait %d | circSteps %d | %s\n", ...
        r.target, r.loaded, 100 * r.loadFraction, r.minB4GapMm, ...
        r.success, r.refuge, r.waitTotal, r.stepsCirculation, string(r.message));
end
end
