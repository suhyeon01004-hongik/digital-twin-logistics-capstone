function results = test_refuge_circulation()
%TEST_REFUGE_CIRCULATION Stress the circulation/refuge controller without auto loading.
rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

caseCount = 10;
results = struct('caseId', {}, 'loaded', {}, 'target', {}, 'steps', {}, ...
    'success', {}, 'collision', {}, 'rotation', {}, 'refuge', {}, 'message', {});

for caseId = 1:caseCount
    parcel_manual_core_step("reset", 0);
    [targetId, loaded] = seedCase(caseId);
    S = parcel_manual_core_step("unload", targetId);
    maxChunks = 9000;
    for chunk = 1:maxChunks
        S = parcel_manual_core_step("stepn", 20);
        if S.statusCode == 0 || S.collisionFlag > 0.5 || S.rotationFlag > 0.5
            break;
        end
    end
    success = S.statusCode == 0 && S.collisionFlag <= 0.5 && S.rotationFlag <= 0.5 && ...
        isfield(S, 'circCompleteTargetId') && S.circCompleteTargetId == targetId;
    results(caseId).caseId = caseId;
    results(caseId).loaded = loaded;
    results(caseId).target = targetId;
    results(caseId).steps = chunk * 20;
    results(caseId).success = success;
    results(caseId).collision = S.collisionFlag;
    results(caseId).rotation = S.rotationFlag;
    results(caseId).refuge = S.tempUnloadCount;
    results(caseId).message = string(S.message);
    fprintf("case %02d loaded=%d target=%d success=%d steps=%d refuge=%d col=%d rot=%d msg=%s\n", ...
        caseId, loaded, targetId, success, chunk * 20, S.tempUnloadCount, ...
        S.collisionFlag, S.rotationFlag, string(S.message));
end
end

function [targetId, loaded] = seedCase(caseId)
id = 1;
targetId = 0;
for floor = 1:3
    order = [3 2 1 4];
    if mod(caseId, 2) == 0
        order = [3 4 2 1];
    end
    for oi = 1:numel(order)
        belt = order(oi);
        used = beltUsed(floor, belt);
        limit = beltLengthLocal(belt) * fillFraction(caseId, floor, belt);
        while true
            [longSide, shortSide, height] = dimsFor(caseId, id);
            len = axisLengthLocal(belt, longSide, shortSide);
            if used + len > min(limit, beltLengthLocal(belt) - 0.005)
                break;
            end
            parcel_manual_core_step("seed_package", ...
                [id floor belt longSide shortSide height]);
            if belt == 3 && floor == targetFloorForCase(caseId)
                targetId = id;
            end
            used = used + len;
            id = id + 1;
        end
    end
end
loaded = id - 1;
if targetId <= 0
    targetId = max(1, loaded - 1);
end
end

function f = fillFraction(caseId, floor, belt)
base = [0.78 0.83 0.80 0.88];
offset = 0.015 * mod(caseId + floor + belt, 4);
f = min(0.94, base(belt) + offset);
if belt == 4
    f = min(0.96, f + 0.04);
end
if belt == 3 && floor == targetFloorForCase(caseId)
    f = min(0.94, f + 0.08);
end
end

function floor = targetFloorForCase(caseId)
floor = 1 + mod(caseId - 1, 3);
end

function used = beltUsed(floor, belt)
S = parcel_manual_core_step("snapshot", 0);
used = 0;
for i = 1:numel(S.ids)
    if S.ids(i) > 0 && S.floors(i) == floor && S.belts(i) == belt
        used = used + axisLengthLocal(belt, S.boxLong(i), S.boxShort(i));
    end
end
end

function [longSide, shortSide, height] = dimsFor(caseId, id)
longSet = [0.115 0.135 0.155 0.175 0.205 0.230 0.245];
shortSet = [0.085 0.095 0.110 0.125 0.145 0.165 0.185];
k = 1 + mod(id * 3 + caseId * 5, numel(longSet));
m = 1 + mod(id * 5 + caseId * 2, numel(shortSet));
longSide = longSet(k);
shortSide = min(shortSet(m), longSide - 0.010);
height = 0.080 + 0.010 * mod(id + caseId, 5);
end

function len = axisLengthLocal(belt, longSide, shortSide)
if belt == 1 || belt == 3
    len = longSide;
else
    len = shortSide;
end
end

function L = beltLengthLocal(belt)
if belt == 1 || belt == 3
    L = 0.500;
else
    L = 1.105;
end
end
