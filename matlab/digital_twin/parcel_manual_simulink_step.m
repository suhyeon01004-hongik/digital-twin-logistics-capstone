function y = parcel_manual_simulink_step(u)
%PARCEL_MANUAL_SIMULINK_STEP Interpreted MATLAB Fcn wrapper for Simulink.
%
% u = [load_cmd unload_cmd target_id reset_cmd]
% y = [status loaded unloaded active_floor active_belt target_id collision rotation idle]

persistent lastLoad lastUnload lastReset
if isempty(lastLoad)
    lastLoad = 0;
    lastUnload = 0;
    lastReset = 0;
    S = parcel_manual_core_step("reset", 0);
else
    S = parcel_manual_core_step("snapshot", 0);
end

loadCmd = u(1) > 0.5;
unloadCmd = u(2) > 0.5;
targetId = round(u(3));
resetCmd = u(4) > 0.5;

if resetCmd && ~lastReset
    S = parcel_manual_core_step("reset", 0);
elseif loadCmd && ~lastLoad
    S = parcel_manual_core_step("load", 0);
elseif unloadCmd && ~lastUnload
    S = parcel_manual_core_step("unload", targetId);
else
    S = parcel_manual_core_step("step", 0);
end

lastLoad = loadCmd;
lastUnload = unloadCmd;
lastReset = resetCmd;

parcel_manual_animation_update(S);

y = zeros(9,1);
y(1) = S.statusCode;
y(2) = S.loadedCount;
y(3) = S.unloadedCount;
y(4) = S.activeFloor;
y(5) = S.activeBelt;
y(6) = S.targetId;
y(7) = S.collisionFlag;
y(8) = S.rotationFlag;
y(9) = double(S.isIdle);
end
