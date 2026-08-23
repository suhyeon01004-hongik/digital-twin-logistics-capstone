function results = run_parcel_final_e2e_suite(profile)
%RUN_PARCEL_FINAL_E2E_SUITE Final scenario test from YOLO loading to wait-area unload.
% The suite validates:
%   measured YOLO loading -> circulation/refuge -> B4 reverse unload
%   -> B4 restore -> platform F1 camera/yaw alignment -> wait-area side push.

rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

if nargin < 1 || strlength(string(profile)) == 0
    profile = "standard";
end
profile = lower(string(profile));

outRoot = getenv('REFUGE_TEST_OUTPUT_DIR');
if strlength(string(outRoot)) == 0
    outRoot = fullfile(rootDir, 'outputs');
end
outDir = fullfile(outRoot, 'final_e2e_20260615');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

stamp = datestr(now, 'yyyymmdd_HHMMSS');
progressPath = fullfile(outDir, sprintf('final_e2e_progress_%s.txt', stamp));
summaryPath = fullfile(outDir, sprintf('final_e2e_summary_%s.txt', stamp));
csvPath = fullfile(outDir, sprintf('final_e2e_results_%s.csv', stamp));
matPath = fullfile(outDir, sprintf('final_e2e_results_%s.mat', stamp));

scenarios = finalScenarios(profile);
results = blankResult();
results(1) = [];
progress("FINAL E2E start profile=%s scenarios=%d", profile, numel(scenarios));

for si = 1:numel(scenarios)
    sc = scenarios(si);
    configureScenario(sc);
    cfg = parcel_manual_config();
    progress("SCENARIO %s config=%s floor=%d belt=%s weights=%s", ...
        sc.name, cfg.versionName, cfg.floorCount, mat2str(cfg.beltLengthM, 3), ...
        mat2str(cfg.boxTypeWeights, 3));

    for startId = sc.startIds
        parcel_manual_core_step("reset", 0);
        parcel_manual_core_step("set_next_id", startId);
        [S0, loadSteps, loadStopReason] = runMeasuredLoads(sc, startId);
        loadSuccess = S0.loadedCount > 0 && S0.collisionFlag <= 0.5 && S0.rotationFlag <= 0.5 && S0.isIdle;
        loadResult = makeLoadResult(sc, startId, S0, loadSteps, loadSuccess, loadStopReason);
        results(end+1) = loadResult; %#ok<AGROW>
        progressResult(loadResult);
        if ~loadSuccess
            if sc.stopOnFailure
                finalize();
                return;
            end
            continue;
        end

        parcel_manual_core_step("save_state", 0);
        targets = chooseFinalTargets(S0, sc.targetCount);
        if isempty(targets)
            progress("NO TARGETS scenario=%s start=%d", sc.name, startId);
            continue;
        end

        for ti = 1:numel(targets)
            targetId = targets(ti);
            parcel_manual_core_step("load_state", 0);
            waitBefore = S0.waitTotal;
            activeBefore = activeCirculationCount(S0);
            loadBefore = activeAxisUsed(S0) / max(eps, cfg.floorCount * S0.floorCapacityM);
            prevRefuge = S0.tempUnloadCount;
            prevReinsert = S0.tempReinsertCount;
            [S, trace] = runTargetWithTrace(targetId, sc.maxTargetSteps);
            success = targetSuccess(S, targetId, waitBefore, trace);
            r = makeTargetResult(sc, startId, "scan", ti, targetId, activeBefore, ...
                loadBefore, trace.steps, S, success, trace, prevRefuge, prevReinsert);
            results(end+1) = r; %#ok<AGROW>
            progressResult(r);
            if ~success && sc.stopOnFailure
                finalize();
                return;
            end
        end

        if sc.sequentialDeliveries > 0
            parcel_manual_core_step("load_state", 0);
            Sseq = parcel_manual_core_step("snapshot", 0);
            for di = 1:sc.sequentialDeliveries
                targetId = chooseSequentialTarget(Sseq, di);
                if targetId <= 0
                    break;
                end
                waitBefore = Sseq.waitTotal;
                activeBefore = activeCirculationCount(Sseq);
                loadBefore = activeAxisUsed(Sseq) / max(eps, cfg.floorCount * Sseq.floorCapacityM);
                prevRefuge = Sseq.tempUnloadCount;
                prevReinsert = Sseq.tempReinsertCount;
                [Sseq, trace] = runTargetWithTrace(targetId, sc.maxTargetSteps);
                success = targetSuccess(Sseq, targetId, waitBefore, trace);
                r = makeTargetResult(sc, startId, "sequential", di, targetId, ...
                    activeBefore, loadBefore, trace.steps, Sseq, success, trace, prevRefuge, prevReinsert);
                results(end+1) = r; %#ok<AGROW>
                progressResult(r);
                if ~success && sc.stopOnFailure
                    finalize();
                    return;
                end
                Sseq = parcel_manual_core_step("clear_wait", 0);
                Sseq = parcel_manual_core_step("snapshot", 0);
            end
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
        progress("%s | %s | start=%d case=%d target=%d loaded=%d active=%d load=%.1f%% success=%d col=%d rot=%d yoloF1=%d align=%d refuge=%d(+%d) rein=%d(+%d) wait=%d steps=%d | %s", ...
            r.scenario, r.phase, r.startId, r.caseIndex, r.targetId, ...
            r.loadedCount, r.activeBefore, 100 * r.loadFractionBefore, ...
            r.success, r.collision, r.rotation, r.sawMode27, r.sawYoloAligned, ...
            r.refuge, r.refugeDelta, r.reinsert, r.reinsertDelta, ...
            r.waitTotal, r.steps, r.message);
    end

    function finalize()
        if isempty(results)
            return;
        end
        T = struct2table(results);
        writetable(T, csvPath);
        save(matPath, 'results', 'scenarios', 'profile');
        writeFinalSummary(results, scenarios, summaryPath, csvPath, matPath, progressPath);
        progress("FINAL E2E written summary=%s", summaryPath);
        failed = results([results.success] == 0);
        if ~isempty(failed)
            progress("FINAL E2E failed cases=%d", numel(failed));
            error("refuge:finalE2EFailed", ...
                "Final E2E suite has %d failed case(s). See %s", ...
                numel(failed), summaryPath);
        end
    end
end

function scenarios = finalScenarios(profile)
base = struct('name', "", 'preset', "baseline", 'overrides', struct(), ...
    'startIds', [], 'targetCount', 0, 'sequentialDeliveries', 0, ...
    'maxLoads', 96, 'maxStepsPerLoad', 18000, 'maxTargetSteps', 42000, ...
    'stopOnFailure', true);

if profile == "quick"
    scenarios = repmat(base, 1, 3);
    scenarios(1).name = "baseline_measured_quick";
    scenarios(1).preset = "baseline";
    scenarios(1).startIds = [1];
    scenarios(1).targetCount = 4;
    scenarios(1).sequentialDeliveries = 3;

    scenarios(2).name = "small_dominant_measured_quick";
    scenarios(2).preset = "small_dominant";
    scenarios(2).startIds = [4];
    scenarios(2).targetCount = 4;
    scenarios(2).sequentialDeliveries = 3;

    scenarios(3).name = "large_stress_measured_quick";
    scenarios(3).preset = "large_stress";
    scenarios(3).startIds = [2];
    scenarios(3).targetCount = 3;
    scenarios(3).sequentialDeliveries = 2;
    return;
end

if profile == "floor_coverage"
    scenarios = repmat(base, 1, 1);
    scenarios(1).name = "small_dominant_floor_coverage";
    scenarios(1).preset = "small_dominant";
    scenarios(1).startIds = [4];
    scenarios(1).targetCount = 9;
    scenarios(1).sequentialDeliveries = 0;
    return;
end

scenarios = repmat(base, 1, 6);
scenarios(1).name = "baseline_measured_a";
scenarios(1).preset = "baseline";
scenarios(1).startIds = [1 5];
scenarios(1).targetCount = 6;
scenarios(1).sequentialDeliveries = 5;

scenarios(2).name = "small_dominant_measured_a";
scenarios(2).preset = "small_dominant";
scenarios(2).startIds = [1 7];
scenarios(2).targetCount = 7;
scenarios(2).sequentialDeliveries = 6;

scenarios(3).name = "small_dominant_measured_b";
scenarios(3).preset = "small_dominant";
scenarios(3).startIds = [13];
scenarios(3).targetCount = 8;
scenarios(3).sequentialDeliveries = 6;

scenarios(4).name = "large_stress_measured";
scenarios(4).preset = "large_stress";
scenarios(4).startIds = [2];
scenarios(4).targetCount = 6;
scenarios(4).sequentialDeliveries = 4;

scenarios(5).name = "belt_plus_5pct_small";
scenarios(5).preset = "small_dominant";
scenarios(5).overrides = struct('beltLengthM', [0.525 1.160 0.525 1.160], ...
    'waitAreaLengthM', 0.525);
scenarios(5).startIds = [3];
scenarios(5).targetCount = 6;
scenarios(5).sequentialDeliveries = 4;

scenarios(6).name = "belt_minus_3pct_small";
scenarios(6).preset = "small_dominant";
scenarios(6).overrides = struct('beltLengthM', [0.485 1.070 0.485 1.070], ...
    'waitAreaLengthM', 0.485);
scenarios(6).startIds = [4];
scenarios(6).targetCount = 6;
scenarios(6).sequentialDeliveries = 4;
end

function configureScenario(sc)
parcel_manual_config("reset");
parcel_manual_config("preset", sc.preset);
if ~isempty(fieldnames(sc.overrides))
    parcel_manual_config("set", sc.overrides);
end
end

function [S, totalSteps, stopReason] = runMeasuredLoads(sc, startId)
totalSteps = 0;
stopReason = "max_loads";
S = parcel_manual_core_step("snapshot", 0);
staleAttempts = 0;
for attempt = 1:sc.maxLoads
    beforeLoaded = S.loadedCount;
    id = startId + attempt - 1;
    yolo = measuredYoloForId(id);
    S = parcel_digital_twin_io_adapter("apply_yolo_load", yolo, "normal");
    [S, steps] = runUntilIdleOrFault(sc.maxStepsPerLoad);
    totalSteps = totalSteps + steps;
    if S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        stopReason = "fault";
        return;
    end
    if S.loadedCount <= beforeLoaded
        staleAttempts = staleAttempts + 1;
    else
        staleAttempts = 0;
    end
    if staleAttempts >= 1 || startsWith(string(S.message), "LOAD FINALIZE BLOCKED")
        stopReason = "capacity_finalized";
        break;
    end
end
[S, settleSteps] = settleAfterLoading(S, 60000);
totalSteps = totalSteps + settleSteps;
end

function [S, steps] = runUntilIdleOrFault(maxSteps)
steps = 0;
S = parcel_manual_core_step("snapshot", 0);
while steps < maxSteps
    S = parcel_manual_core_step("stepn", 20);
    steps = steps + 20;
    if S.isIdle || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        return;
    end
end
end

function [S, steps] = settleAfterLoading(S, maxSteps)
steps = 0;
while steps < maxSteps
    if S.isIdle && S.collisionFlag <= 0.5 && S.rotationFlag <= 0.5
        return;
    end
    S = parcel_manual_core_step("stepn", 20);
    steps = steps + 20;
end
end

function yolo = measuredYoloForId(id)
cfg = parcel_manual_config();
boxType = deterministicBoxType(id, cfg.boxTypeWeights);
dimsM = cfg.packageSizeM(boxType, :) * cfg.packageScale;
longMm = 1000 * max(dimsM(1), dimsM(2));
shortMm = 1000 * min(dimsM(1), dimsM(2));
yolo = parcel_digital_twin_io_adapter("yolo_template");
yolo.parcelId = id;
yolo.longMm = longMm;
yolo.shortMm = shortMm;
yolo.heightMm = 1000 * dimsM(3);
yolo.yawDeg = deterministicYawDeg(id);
yolo.confidence = 0.90 + 0.09 * mod(id * 37, 11) / 10;
yolo.qrText = sprintf("E2E-%04d", id);
yolo.destinationCode = sprintf("D%02d", 1 + mod(id - 1, 9));
yolo.isFirstDelivery = false;
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

function yaw = deterministicYawDeg(id)
yaw = -28 + 56 * mod(id * 22695477 + 1, 20001) / 20000;
end

function targets = chooseFinalTargets(S, maxTargets)
targets = [];
floorCount = max(S.floors);
for floor = 1:floorCount
    floorIds = S.ids(S.ids > 0 & S.floors == floor);
    if isempty(floorIds)
        continue;
    end
    for belt = [3 2 1 4]
        beltIds = S.ids(S.ids > 0 & S.floors == floor & S.belts == belt);
        if ~isempty(beltIds)
            targets = [targets; max(beltIds(:))]; %#ok<AGROW>
            break;
        end
    end
    targets = [targets; min(floorIds(:)); max(floorIds(:))]; %#ok<AGROW>
end
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
targets = chooseFinalTargets(S, max(8, roundId + 4));
if isempty(targets)
    targetId = 0;
else
    targetId = targets(1 + mod(roundId - 1, numel(targets)));
end
end

function ids = chooseB4BlockedTargets(S)
ids = [];
floorCount = max(S.floors);
for floor = 1:floorCount
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

function [S, trace] = runTargetWithTrace(targetId, maxSteps)
S = parcel_manual_core_step("unload", targetId);
trace = struct();
trace.steps = 0;
trace.sawMode27 = false;
trace.sawF1Camera = false;
trace.sawYoloAligned = false;
trace.sawWaitSidePush = false;
trace.sawB4Restore = false;
trace.lastNonzeroMode = S.phase;
while trace.steps < maxSteps
    S = parcel_manual_core_step("stepn", 20);
    trace.steps = trace.steps + 20;
    trace.sawB4Restore = trace.sawB4Restore || S.phase == 23 || S.targetYoloModeCount > 0;
    trace.sawMode27 = trace.sawMode27 || S.targetYoloModeCount > 0;
    trace.sawF1Camera = trace.sawF1Camera || S.targetYoloF1Seen > 0.5;
    trace.sawYoloAligned = trace.sawYoloAligned || S.targetYoloAlignedSeen > 0.5;
    trace.sawWaitSidePush = trace.sawWaitSidePush || S.phase == 24 || ...
        S.phase == 25 || S.circCompleteTargetId == targetId;
    if S.phase ~= 0
        trace.lastNonzeroMode = S.phase;
    end
    if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
        return;
    end
end
end

function ok = targetSuccess(S, targetId, waitBefore, trace)
ok = S.statusCode == 0 && S.isIdle && S.collisionFlag <= 0.5 && ...
    S.rotationFlag <= 0.5 && S.circCompleteTargetId == targetId && ...
    S.waitTotal >= waitBefore + 1 && trace.sawMode27 && ...
    trace.sawF1Camera && trace.sawYoloAligned && trace.sawWaitSidePush;
end

function r = makeLoadResult(sc, startId, S, loadSteps, success, stopReason)
r = blankResult();
r.scenario = string(sc.name);
r.phase = "load";
r.startId = startId;
r.caseIndex = 0;
r.targetId = 0;
r.activeBefore = activeCirculationCount(S);
r.loadedCount = S.loadedCount;
r.loadFractionBefore = activeAxisUsed(S) / max(eps, parcel_manual_config().floorCount * S.floorCapacityM);
r.steps = loadSteps;
r.success = success;
r.collision = S.collisionFlag;
r.rotation = S.rotationFlag;
r.refuge = S.tempUnloadCount;
r.reinsert = S.tempReinsertCount;
r.waitTotal = S.waitTotal;
r.message = sprintf("%s | %s", stopReason, string(S.message));
end

function r = makeTargetResult(sc, startId, phase, caseIndex, targetId, activeBefore, ...
        loadBefore, steps, S, success, trace, prevRefuge, prevReinsert)
r = blankResult();
r.scenario = string(sc.name);
r.phase = string(phase);
r.startId = startId;
r.caseIndex = caseIndex;
r.targetId = targetId;
r.activeBefore = activeBefore;
r.loadedCount = S.loadedCount;
r.loadFractionBefore = loadBefore;
r.steps = steps;
r.success = success;
r.collision = S.collisionFlag;
r.rotation = S.rotationFlag;
r.refuge = S.tempUnloadCount;
r.reinsert = S.tempReinsertCount;
r.refugeDelta = max(0, S.tempUnloadCount - prevRefuge);
r.reinsertDelta = max(0, S.tempReinsertCount - prevReinsert);
r.waitTotal = S.waitTotal;
r.sawMode27 = double(trace.sawMode27);
r.sawF1Camera = double(trace.sawF1Camera);
r.sawYoloAligned = double(trace.sawYoloAligned);
r.sawWaitSidePush = double(trace.sawWaitSidePush);
r.message = string(S.message);
end

function r = blankResult()
r = struct('scenario', "", 'phase', "", 'startId', 0, 'caseIndex', 0, ...
    'targetId', 0, 'activeBefore', 0, 'loadedCount', 0, ...
    'loadFractionBefore', 0, 'steps', 0, 'success', 0, ...
    'collision', 0, 'rotation', 0, 'refuge', 0, 'reinsert', 0, ...
    'refugeDelta', 0, 'reinsertDelta', 0, 'waitTotal', 0, ...
    'sawMode27', 0, 'sawF1Camera', 0, 'sawYoloAligned', 0, ...
    'sawWaitSidePush', 0, 'message', "");
end

function count = activeCirculationCount(S)
count = sum(S.ids > 0 & S.floors > 0 & S.belts > 0);
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

function writeFinalSummary(results, scenarios, summaryPath, csvPath, matPath, progressPath)
fid = fopen(summaryPath, 'w');
if fid < 0
    return;
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, "Final parcel end-to-end scenario suite\n\n");
fprintf(fid, "Success criterion:\n");
fprintf(fid, "- measured YOLO loading reaches idle without collision/rotation\n");
fprintf(fid, "- target unload reaches wait area without collision/rotation\n");
fprintf(fid, "- mode 27 F1 camera/yaw alignment is observed before wait-area side push\n\n");
fprintf(fid, "Scenarios: %d\n", numel(scenarios));
fprintf(fid, "CSV: %s\nMAT: %s\nProgress: %s\n\n", csvPath, matPath, progressPath);

success = [results.success] > 0.5;
fprintf(fid, "Overall: %d/%d success\n\n", sum(success), numel(results));

names = unique(string({results.scenario}), 'stable');
for si = 1:numel(names)
    name = names(si);
    idx = string({results.scenario}) == name;
    subset = results(idx);
    ok = [subset.success] > 0.5;
    fprintf(fid, "[%s] %d/%d success\n", name, sum(ok), numel(subset));
    for k = 1:numel(subset)
        r = subset(k);
        fprintf(fid, "  %s start %d case %d target %d loaded %d active %d load %.1f%% success %d yoloF1 %d align %d refuge +%d rein +%d steps %d | %s\n", ...
            r.phase, r.startId, r.caseIndex, r.targetId, r.loadedCount, ...
            r.activeBefore, 100 * r.loadFractionBefore, r.success, ...
            r.sawMode27, r.sawYoloAligned, r.refugeDelta, r.reinsertDelta, ...
            r.steps, string(r.message));
    end
    fprintf(fid, "\n");
end
end
