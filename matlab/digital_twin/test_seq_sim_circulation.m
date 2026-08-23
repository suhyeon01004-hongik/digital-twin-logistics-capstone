function results = test_seq_sim_circulation()
%TEST_SEQ_SIM_CIRCULATION Verify UI-style SEQ layout and circulation.
rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

caseCount = 6;
results = struct('caseId', {}, 'loaded', {}, 'target', {}, 'steps', {}, ...
    'success', {}, 'collision', {}, 'rotation', {}, 'refuge', {}, 'message', {});

for caseId = 1:caseCount
    parcel_manual_core_step("reset", 0);
    for floor = 1:3
        for n = 1:22
            dims = seqDims(caseId, floor, n);
            S = parcel_manual_core_step("seq_package", [floor dims]);
            if startsWith(string(S.message), "SEQ BLOCKED")
                break;
            end
        end
    end
    S = parcel_manual_core_step("snapshot", 0);
    targetFloor = 1 + mod(caseId - 1, 3);
    targetId = chooseOldestTargetOnFloor(S, targetFloor);
    if targetId <= 0
        targetId = min(S.ids(S.ids > 0));
    end
    S = parcel_manual_core_step("unload", targetId);
    for chunk = 1:9000
        S = parcel_manual_core_step("stepn", 20);
        if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
            break;
        end
    end
    success = S.statusCode == 0 && S.collisionFlag <= 0.5 && S.rotationFlag <= 0.5 && ...
        isfield(S, 'circCompleteTargetId') && S.circCompleteTargetId == targetId;
    results(caseId).caseId = caseId;
    results(caseId).loaded = S.loadedCount;
    results(caseId).target = targetId;
    results(caseId).steps = chunk * 20;
    results(caseId).success = success;
    results(caseId).collision = S.collisionFlag;
    results(caseId).rotation = S.rotationFlag;
    results(caseId).refuge = S.tempUnloadCount;
    results(caseId).message = string(S.message);
    fprintf("seq case %02d loaded=%d target=%d success=%d steps=%d refuge=%d col=%d rot=%d msg=%s\n", ...
        caseId, S.loadedCount, targetId, success, chunk * 20, S.tempUnloadCount, ...
        S.collisionFlag, S.rotationFlag, string(S.message));
end
end

function targetId = chooseOldestTargetOnFloor(S, floor)
ids = S.ids(S.ids > 0 & S.floors == floor);
if isempty(ids)
    targetId = 0;
else
    targetId = min(ids);
end
end

function dims = seqDims(caseId, floor, n)
longSet = [115 135 155 175 205 230 245];
shortSet = [85 95 110 125 145 165 185];
k = 1 + mod(n * 3 + floor * 2 + caseId * 5, numel(longSet));
m = 1 + mod(n * 5 + floor * 3 + caseId * 2, numel(shortSet));
longSide = longSet(k);
shortSide = min(shortSet(m), longSide - 10);
height = 80 + 10 * mod(n + floor + caseId, 5);
dims = [longSide shortSide height];
end
