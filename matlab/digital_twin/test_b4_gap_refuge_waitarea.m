function results = test_b4_gap_refuge_waitarea(caseCount, maxCandidates, maxSteps)
%TEST_B4_GAP_REFUGE_WAITAREA Max-load circulation test with B4 250 mm reserve.
rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

if nargin < 1 || isempty(caseCount)
    caseCount = 6;
end
if nargin < 2 || isempty(maxCandidates)
    maxCandidates = 8;
end
if nargin < 3 || isempty(maxSteps)
    maxSteps = 14000;
end

outDir = fullfile(rootDir, 'outputs');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

results = struct('caseId', {}, 'loaded', {}, 'loadFraction', {}, ...
    'minB4GapMm', {}, 'target', {}, 'steps', {}, 'success', {}, ...
    'collision', {}, 'rotation', {}, 'refuge', {}, 'waitTotal', {}, ...
    'message', {});

best = [];
for caseId = 1:caseCount
    parcel_manual_core_step("reset", 0);
    S0 = buildMaxB4GapLayout(caseId);
    parcel_manual_core_step("save_state", 0);
    candidates = chooseB4BlockedTargets(S0);
    if isempty(candidates)
        candidates = chooseAnyTargets(S0);
    end
    candidates = candidates(1:min(numel(candidates), maxCandidates));

    trial = [];
    for k = 1:numel(candidates)
        parcel_manual_core_step("load_state", 0);
        [S, steps] = runTarget(candidates(k), maxSteps);
        success = S.statusCode == 0 && S.collisionFlag <= 0.5 && ...
            S.rotationFlag <= 0.5 && S.circCompleteTargetId == candidates(k) && ...
            S.waitTotal >= 1 && S.tempUnloadCount >= 1;
        trial = makeResult(caseId, S0, S, candidates(k), steps, success);
        fprintf("case %02d target=%d loaded=%d load=%.1f%% b4gap=%.0fmm success=%d refuge=%d wait=%d col=%d rot=%d steps=%d msg=%s\n", ...
            caseId, candidates(k), trial.loaded, 100 * trial.loadFraction, ...
            trial.minB4GapMm, success, trial.refuge, trial.waitTotal, ...
            trial.collision, trial.rotation, steps, string(trial.message));
        if success
            break;
        end
    end

    if isempty(trial)
        S = S0;
        trial = makeResult(caseId, S0, S, 0, 0, false);
    end
    results(end+1) = trial; %#ok<AGROW>
    if trial.success && isempty(best)
        best.caseId = caseId;
        best.target = trial.target;
        best.before = S0;
        best.after = S;
    end
end

save(fullfile(outDir, 'b4_gap_refuge_waitarea_results.mat'), 'results');
writetable(struct2table(results), fullfile(outDir, 'b4_gap_refuge_waitarea_results.csv'));
writeSummary(results, fullfile(outDir, 'b4_gap_refuge_waitarea_summary.txt'));

if ~isempty(best)
    renderSnapshot(best.before, fullfile(outDir, 'b4_gap_refuge_before.png'));
    renderSnapshot(best.after, fullfile(outDir, 'b4_gap_refuge_wait_complete.png'));
end
end

function S = buildMaxB4GapLayout(caseId)
for floor = 1:3
    for n = 1:40
        dims = stressDims(caseId, floor, n);
        S = parcel_manual_core_step("seq_package", [floor dims]);
        if startsWith(string(S.message), "SEQ BLOCKED")
            break;
        end
    end
end
S = parcel_manual_core_step("snapshot", 0);
end

function dims = stressDims(caseId, floor, n)
longSet = [115 135 155 175 205 230 245];
shortSet = [85 95 110 125 145 165 185];
k = 1 + mod(n * 5 + floor * 3 + caseId * 7, numel(longSet));
m = 1 + mod(n * 2 + floor * 5 + caseId * 3, numel(shortSet));
longSide = longSet(k);
shortSide = min(shortSet(m), longSide - 10);
height = 80 + 10 * mod(n + floor + caseId, 5);
dims = [longSide shortSide height];
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
        tails(k) = S.pos(i) - len/2;
    end
    [~, order] = sort(tails, 'ascend');
    sorted = idxs(order);
    ids = [ids; S.ids(sorted(2:end))]; %#ok<AGROW>
end
ids = unique(ids, 'stable');
end

function ids = chooseAnyTargets(S)
ids = S.ids(S.ids > 0);
ids = ids(:);
end

function [S, steps] = runTarget(targetId, maxSteps)
S = parcel_manual_core_step("unload", targetId);
chunk = 20;
steps = 0;
stale = 0;
lastKey = "";
while steps < maxSteps
    S = parcel_manual_core_step("stepn", chunk);
    steps = steps + chunk;
    if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        return;
    end
    key = sprintf("%d|%d|%d|%d|%.3f|%.3f|%s", S.phase, S.activeFloor, ...
        S.activeBelt, S.targetBelt, S.circLapPercent, S.pusherPosition, string(S.message));
    if key == lastKey && sum(abs(S.dtMotorCmd) > 0.5) == 0 && ...
            abs(S.dtPlatformStepCmd) < 0.5 && abs(S.dtPusherStepCmd) < 0.5 && ...
            abs(S.dtWaitSidePusherStepCmd) < 0.5
        stale = stale + 1;
    else
        stale = 0;
        lastKey = key;
    end
    if stale > 100
        return;
    end
end
end

function result = makeResult(caseId, S0, S, target, steps, success)
result.caseId = caseId;
result.loaded = S0.loadedCount;
result.loadFraction = activeAxisUsed(S0) / (3 * S0.floorCapacityM);
result.minB4GapMm = minB4Gap(S0) * 1000;
result.target = target;
result.steps = steps;
result.success = success;
result.collision = S.collisionFlag;
result.rotation = S.rotationFlag;
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
fprintf(fid, "B4 250mm gap + refuge + wait-area circulation test\n");
fprintf(fid, "success criterion: idle, no collision, no rotation, refuge >= 1, target stored in wait area\n\n");
for k = 1:numel(results)
    r = results(k);
    fprintf(fid, "case %02d | loaded %d | load %.1f%% | min B4 gap %.0f mm | target %d | success %d | refuge %d | wait %d | steps %d | %s\n", ...
        r.caseId, r.loaded, 100 * r.loadFraction, r.minB4GapMm, r.target, ...
        r.success, r.refuge, r.waitTotal, r.steps, string(r.message));
end
end
