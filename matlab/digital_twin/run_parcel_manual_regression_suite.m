function results = run_parcel_manual_regression_suite(profile)
%RUN_PARCEL_MANUAL_REGRESSION_SUITE Validate the fixed loading/circulation baseline.
%   results = run_parcel_manual_regression_suite()
%   results = run_parcel_manual_regression_suite("quick")
%
% The suite writes CSV/MAT/TXT outputs under this module's outputs directory.

rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

if nargin < 1 || strlength(string(profile)) == 0
    profile = "standard";
end
profile = lower(string(profile));

outDir = getenv('REFUGE_TEST_OUTPUT_DIR');
if strlength(string(outDir)) == 0
    outDir = fullfile(rootDir, 'outputs');
end
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

stamp = datestr(now, 'yyyymmdd_HHMMSS');
progressPath = fullfile(outDir, sprintf('parcel_regression_progress_%s.txt', stamp));
summaryPath = fullfile(outDir, sprintf('parcel_regression_summary_%s.txt', stamp));
csvPath = fullfile(outDir, sprintf('parcel_regression_results_%s.csv', stamp));
matPath = fullfile(outDir, sprintf('parcel_regression_results_%s.mat', stamp));

scenarios = regressionScenarios(profile);
results = emptyResult();
progress("Regression suite start profile=%s scenarios=%d", profile, numel(scenarios));

for si = 1:numel(scenarios)
    sc = scenarios(si);
    configureScenario(sc);
    cfg = parcel_manual_config();
    progress("SCENARIO %s config=%s belt=[%.3f %.3f %.3f %.3f] weights=%s", ...
        sc.name, cfg.versionName, cfg.beltLengthM(1), cfg.beltLengthM(2), ...
        cfg.beltLengthM(3), cfg.beltLengthM(4), mat2str(cfg.boxTypeWeights, 3));

    for startId = sc.startIds
        parcel_manual_core_step("reset", 0);
        if startId > 1
            parcel_manual_core_step("set_next_id", startId);
        end
        [S0, loadSteps] = runRealLoads(sc.maxStepsPerLoad);
        loadOk = S0.collisionFlag <= 0.5 && S0.rotationFlag <= 0.5 && S0.loadedCount > 0;
        results(end+1) = makeLoadResult(sc, startId, S0, loadSteps, loadOk); %#ok<AGROW>
        progressResult(results(end));
        if ~loadOk
            continue;
        end

        parcel_manual_core_step("save_state", 0);
        targets = chooseCandidateTargets(S0, sc.scanTargets);
        for ti = 1:numel(targets)
            targetId = targets(ti);
            parcel_manual_core_step("load_state", 0);
            [S, circSteps] = runTarget(targetId, sc.maxCirculationSteps);
            success = targetSuccess(S, targetId);
            results(end+1) = makeTargetResult(sc, startId, "scan", ti, targetId, ...
                S0.loadedCount, activeAxisUsed(S0) / (3 * S0.floorCapacityM), ...
                circSteps, S, success); %#ok<AGROW>
            progressResult(results(end));
            if ~success && sc.stopOnFailure
                finalize();
                return;
            end
        end
    end

    if sc.sequentialStartId > 0 && sc.sequentialDeliveries > 0
        parcel_manual_core_step("reset", 0);
        if sc.sequentialStartId > 1
            parcel_manual_core_step("set_next_id", sc.sequentialStartId);
        end
        [S, loadSteps] = runRealLoads(sc.maxStepsPerLoad);
        results(end+1) = makeLoadResult(sc, sc.sequentialStartId, S, loadSteps, ...
            S.collisionFlag <= 0.5 && S.rotationFlag <= 0.5 && S.loadedCount > 0); %#ok<AGROW>
        results(end).phase = "sequential_load";
        progressResult(results(end));
        for roundId = 1:sc.sequentialDeliveries
            targetId = chooseSequentialTarget(S, roundId);
            if targetId <= 0
                break;
            end
            activeBefore = activeCirculationCount(S);
            loadBefore = activeAxisUsed(S) / (3 * S.floorCapacityM);
            prevRefuge = S.tempUnloadCount;
            prevReinsert = S.tempReinsertCount;
            [S, circSteps] = runTarget(targetId, sc.maxCirculationSteps);
            success = targetSuccess(S, targetId);
            results(end+1) = makeTargetResult(sc, sc.sequentialStartId, "sequential", roundId, ...
                targetId, activeBefore, loadBefore, circSteps, S, success); %#ok<AGROW>
            results(end).refugeDelta = max(0, S.tempUnloadCount - prevRefuge);
            results(end).reinsertDelta = max(0, S.tempReinsertCount - prevReinsert);
            progressResult(results(end));
            if ~success && sc.stopOnFailure
                finalize();
                return;
            end
            S = parcel_manual_core_step("clear_wait", 0);
            S = parcel_manual_core_step("snapshot", 0);
        end
    end
end

finalize();
parcel_manual_config("reset");

    function progress(fmt, varargin)
        line = sprintf(fmt, varargin{:});
        fprintf('%s\n', line);
        fid = fopen(progressPath, 'a');
        if fid >= 0
            cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fprintf(fid, '%s\n', line);
        end
    end

    function progressResult(r)
        progress("%s | %s | start=%d | idx=%d | target=%d | loaded/active=%d | load=%.1f%% | success=%d | col=%d | rot=%d | refuge=%d(+%d) | rein=%d(+%d) | steps=%d | %s", ...
            r.scenario, r.phase, r.startId, r.caseIndex, r.targetId, ...
            r.activeBefore, 100 * r.loadFractionBefore, r.success, ...
            r.collision, r.rotation, r.refuge, r.refugeDelta, ...
            r.reinsert, r.reinsertDelta, r.steps, r.message);
    end

    function finalize()
        if isempty(results)
            return;
        end
        T = struct2table(results);
        writetable(T, csvPath);
        save(matPath, 'results', 'scenarios', 'profile');
        writeSummary(results, summaryPath, csvPath, matPath, progressPath);
        progress("Regression suite written summary=%s", summaryPath);
        failed = results([results.success] == 0);
        if ~isempty(failed)
            progress("Regression suite failed cases=%d", numel(failed));
            error("refuge:regressionFailed", ...
                "Regression suite has %d failed case(s). See %s", ...
                numel(failed), summaryPath);
        end
    end
end

function scenarios = regressionScenarios(profile)
base = struct('name', "", 'preset', "baseline", 'overrides', struct(), ...
    'startIds', [], 'scanTargets', 0, 'sequentialStartId', 0, ...
    'sequentialDeliveries', 0, 'maxStepsPerLoad', 16000, ...
    'maxCirculationSteps', 36000, 'stopOnFailure', true);

if profile == "quick"
    scenarios = repmat(base, 1, 2);
    scenarios(1).name = "baseline_quick";
    scenarios(1).preset = "baseline";
    scenarios(1).startIds = [3];
    scenarios(1).scanTargets = 4;
    scenarios(1).sequentialStartId = 3;
    scenarios(1).sequentialDeliveries = 5;

    scenarios(2).name = "small_dominant_quick";
    scenarios(2).preset = "small_dominant";
    scenarios(2).startIds = [3];
    scenarios(2).scanTargets = 4;
    scenarios(2).sequentialStartId = 3;
    scenarios(2).sequentialDeliveries = 5;
    return;
end

scenarios = repmat(base, 1, 4);
scenarios(1).name = "baseline";
scenarios(1).preset = "baseline";
scenarios(1).startIds = [1 3 6];
scenarios(1).scanTargets = 6;
scenarios(1).sequentialStartId = 3;
scenarios(1).sequentialDeliveries = 15;

scenarios(2).name = "small_dominant";
scenarios(2).preset = "small_dominant";
scenarios(2).startIds = [1 3 8];
scenarios(2).scanTargets = 8;
scenarios(2).sequentialStartId = 3;
scenarios(2).sequentialDeliveries = 18;

scenarios(3).name = "belt_plus_5pct";
scenarios(3).preset = "baseline";
scenarios(3).overrides = struct('beltLengthM', [0.525 1.160 0.525 1.160], ...
    'waitAreaLengthM', 0.525);
scenarios(3).startIds = [2 5];
scenarios(3).scanTargets = 5;
scenarios(3).sequentialStartId = 2;
scenarios(3).sequentialDeliveries = 8;

scenarios(4).name = "belt_minus_3pct_small_mix";
scenarios(4).preset = "small_dominant";
scenarios(4).overrides = struct('beltLengthM', [0.485 1.070 0.485 1.070], ...
    'waitAreaLengthM', 0.485);
scenarios(4).startIds = [4 9];
scenarios(4).scanTargets = 5;
scenarios(4).sequentialStartId = 4;
scenarios(4).sequentialDeliveries = 8;
end

function configureScenario(sc)
parcel_manual_config("reset");
parcel_manual_config("preset", sc.preset);
if ~isempty(fieldnames(sc.overrides))
    parcel_manual_config("set", sc.overrides);
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
S = settleAfterLoading(S, 60000);
end

function S = settleAfterLoading(S, maxSteps)
for stepCount = 0:20:maxSteps
    if S.isIdle && S.collisionFlag <= 0.5 && S.rotationFlag <= 0.5
        return;
    end
    S = parcel_manual_core_step("stepn", 20);
end
end

function targets = chooseCandidateTargets(S, maxTargets)
targets = [];
targets = [targets; chooseB4BlockedTargets(S)]; %#ok<AGROW>
for belt = [3 2 1 4]
    beltIds = S.ids(S.ids > 0 & S.floors > 0 & S.belts == belt);
    beltIds = sort(beltIds(:), 'descend');
    targets = [targets; beltIds]; %#ok<AGROW>
end
targets = unique(targets, 'stable');
targets = targets(1:min(maxTargets, numel(targets)));
end

function targetId = chooseSequentialTarget(S, roundId)
targets = chooseCandidateTargets(S, max(8, roundId + 3));
if isempty(targets)
    targetId = 0;
else
    targetId = targets(1 + mod(roundId - 1, numel(targets)));
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

function ok = targetSuccess(S, targetId)
ok = S.statusCode == 0 && S.collisionFlag <= 0.5 && ...
    S.rotationFlag <= 0.5 && S.circCompleteTargetId == targetId && S.waitTotal >= 1;
end

function r = makeLoadResult(sc, startId, S, loadSteps, success)
r = blankResult();
r.scenario = string(sc.name);
r.phase = "load";
r.startId = startId;
r.caseIndex = 0;
r.targetId = 0;
r.activeBefore = activeCirculationCount(S);
r.loadedCount = S.loadedCount;
r.loadFractionBefore = activeAxisUsed(S) / max(eps, 3 * S.floorCapacityM);
r.steps = loadSteps;
r.success = success;
r.collision = S.collisionFlag;
r.rotation = S.rotationFlag;
r.refuge = S.tempUnloadCount;
r.reinsert = S.tempReinsertCount;
r.waitTotal = S.waitTotal;
r.message = string(S.message);
end

function r = makeTargetResult(sc, startId, phase, caseIndex, targetId, activeBefore, loadBefore, circSteps, S, success)
r = blankResult();
r.scenario = string(sc.name);
r.phase = string(phase);
r.startId = startId;
r.caseIndex = caseIndex;
r.targetId = targetId;
r.activeBefore = activeBefore;
r.loadedCount = S.loadedCount;
r.loadFractionBefore = loadBefore;
r.steps = circSteps;
r.success = success;
r.collision = S.collisionFlag;
r.rotation = S.rotationFlag;
r.refuge = S.tempUnloadCount;
r.reinsert = S.tempReinsertCount;
r.refugeDelta = S.tempUnloadCount;
r.reinsertDelta = S.tempReinsertCount;
r.waitTotal = S.waitTotal;
r.message = string(S.message);
end

function r = blankResult()
r = struct('scenario', "", 'phase', "", 'startId', 0, 'caseIndex', 0, ...
    'targetId', 0, 'activeBefore', 0, 'loadedCount', 0, ...
    'loadFractionBefore', 0, 'steps', 0, 'success', false, ...
    'collision', 0, 'rotation', 0, 'refuge', 0, 'reinsert', 0, ...
    'refugeDelta', 0, 'reinsertDelta', 0, 'waitTotal', 0, 'message', "");
end

function results = emptyResult()
results = repmat(blankResult(), 0, 1);
end

function n = activeCirculationCount(S)
n = sum(S.ids > 0 & S.floors > 0 & S.belts > 0);
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

function writeSummary(results, summaryPath, csvPath, matPath, progressPath)
fid = fopen(summaryPath, 'w');
if fid < 0
    return;
end
cleanup = onCleanup(@() fclose(fid));
total = numel(results);
failures = sum(~[results.success]);
fprintf(fid, "Parcel manual regression suite\n\n");
fprintf(fid, "total %d\nfailures %d\ncsv %s\nmat %s\nprogress %s\n\n", ...
    total, failures, csvPath, matPath, progressPath);
scenarios = unique(string({results.scenario}));
for si = 1:numel(scenarios)
    sc = scenarios(si);
    idx = string({results.scenario}) == sc;
    scResults = results(idx);
    fprintf(fid, "[%s] total %d failures %d\n", sc, numel(scResults), sum(~[scResults.success]));
    for k = 1:numel(scResults)
        r = scResults(k);
        fprintf(fid, "  %s start %d idx %d target %d active %d load %.1f%% success %d col %d rot %d refuge %d rein %d steps %d | %s\n", ...
            r.phase, r.startId, r.caseIndex, r.targetId, r.activeBefore, ...
            100 * r.loadFractionBefore, r.success, r.collision, r.rotation, ...
            r.refuge, r.reinsert, r.steps, string(r.message));
    end
    fprintf(fid, "\n");
end
end
