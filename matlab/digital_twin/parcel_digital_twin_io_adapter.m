function out = parcel_digital_twin_io_adapter(action, varargin)
%PARCEL_DIGITAL_TWIN_IO_ADAPTER ROS2/YOLO I/O adapter for the parcel twin.
% This file keeps the simulator core independent from ROS2 message types.
% ROS2 callbacks should normalize raw sensor messages here, and YOLO results
% from the main PC can be converted to measured load commands here.

if nargin < 1 || isempty(action)
    action = "schema";
end

action = lower(string(action));
switch action
    case "schema"
        out = twinSchema();
    case "ros_floor_template"
        out = rosFloorTemplate();
    case "yolo_template"
        out = yoloTemplate();
    case "normalize_ros_floor"
        requireArgs(action, varargin, 1);
        out = normalizeRosFloor(varargin{1});
    case "normalize_yolo"
        requireArgs(action, varargin, 1);
        out = normalizeYolo(varargin{1});
    case "measured_load_spec"
        requireArgs(action, varargin, 1);
        yolo = normalizeYolo(varargin{1});
        out = yolo.measuredLoadSpec;
    case "apply_yolo_load"
        requireArgs(action, varargin, 1);
        mode = "auto";
        if numel(varargin) >= 2
            mode = lower(string(varargin{2}));
        end
        out = applyYoloLoad(varargin{1}, mode);
    case "snapshot_to_command"
        requireArgs(action, varargin, 1);
        out = snapshotToCommandFrame(varargin{1});
    case "validate_ros_floor"
        requireArgs(action, varargin, 1);
        out = validateRosFloor(varargin{1});
    case "validate_yolo"
        requireArgs(action, varargin, 1);
        out = validateYolo(varargin{1});
    otherwise
        error("Unknown parcel_digital_twin_io_adapter action: %s", action);
end
end

function requireArgs(action, args, n)
if numel(args) < n
    error("%s requires at least %d input argument(s)", action, n);
end
end

function s = twinSchema()
cfg = parcel_manual_config();
s = struct();
s.version = "digital_twin_io_schema_20260624_physical_v2";
s.floorCount = cfg.floorCount;
s.beltNames = ["B1" "B2" "B3" "B4"];
s.tofOrder = tofOrder();
s.tofRawChannelCount = 8;
s.tofActiveChannels0 = [0 2 4 6];
s.tofActiveIndex = [1 3 5 7];
s.tofGapIndex = s.tofActiveIndex;
s.tofTransferIndex = s.tofActiveIndex;
s.rosTopics = struct( ...
    "floorState", "/parcel/floor_%d/state", ...
    "floorCommand", "/parcel/floor_%d/cmd", ...
    "twinState", "/parcel/digital_twin/state", ...
    "twinCommand", "/parcel/digital_twin/command_frame", ...
    "faultEvent", "/parcel/floor_%d/event");
s.units = struct( ...
    "tof", "mm from Arduino/ROS2, m inside simulator", ...
    "encoder", "count and mm from Arduino/ROS2, m inside simulator", ...
    "servo", "degree", ...
    "platform", "sensorless commanded z/tilt estimate unless homing sensors are added", ...
    "pusher", "sensorless screw-linear commanded mm estimate unless position sensors are added", ...
    "unloadArea", "estimated queue from unload commands and package dimensions", ...
    "uncertainty", "mm or degree confidence radius for sensorless estimates", ...
    "yoloSize", "mm", ...
    "yoloYaw", "degree");
s.yoloRequiredFields = ["longMm" "shortMm" "heightMm" "yawDeg"];
s.yoloOptionalFields = ["parcelId" "qrText" "destinationCode" "isFirstDelivery" "confidence"];
s.simActions = ["load_measured" "wait_load_measured" "step" "unload" "snapshot"];
s.notes = [
    "ROS2 carries encoder, TOF, motor, platform, pusher, and servo states."
    "YOLO runs directly on the main PC and enters the twin through apply_yolo_load."
    "Only CH0/2/4/6 are active; each active ToF is used for both gap and transfer decisions."
    "B4 TOF barrier must be DOWN before using TOF gap values and UP only during platform/B4 interactions."
    "The wait area is a fixed plate and must not be used as a refuge buffer."
    "Platform, pusher, and unload-area states are command-integrated estimates by default."
    "Every sensorless physical state should expose confidence/source/uncertainty fields."
];
end

function t = tofOrder()
t = ["B1_GAP_TRANSFER" "UNUSED_CH1" "B2_GAP_TRANSFER" "UNUSED_CH3" ...
     "B3_GAP_TRANSFER" "UNUSED_CH5" "B4_GAP_TRANSFER" "UNUSED_CH7"];
end

function t = rosFloorTemplate()
t = struct();
t.stampSec = 0;
t.floorId = 1;
t.tofMm = nan(1,8);
t.tofOk = false(1,8);
t.encoderCount = zeros(1,4);
t.encoderMm = nan(1,4);
t.mmPerEncoderCount = ones(1,4);
t.motorPwm = zeros(1,4);
t.motorDir = zeros(1,4);
t.motorEnabled = false(1,4);
t.b4BarrierServoDeg = 0;
t.b4BarrierState = "DOWN";
t.platformZMm = nan;
t.platformFloor = nan;
t.platformTargetFloor = nan;
t.platformTiltDeg = 0;
t.platformTargetTiltDeg = 0;
t.platformZUncertaintyMm = nan;
t.platformTiltUncertaintyDeg = nan;
t.platformConfidence = "";
t.platformBusy = false;
t.pusherMm = nan;
t.pusherTargetMm = nan;
t.sidePusherMm = nan;
t.sidePusherTargetMm = nan;
t.pusherMainUncertaintyMm = nan;
t.pusherSideUncertaintyMm = nan;
t.pusherConfidence = "";
t.pusherBusy = false;
t.unloadPackages = struct([]);
t.unloadLayoutUncertaintyMm = nan;
t.unloadConfidence = "";
t.estop = false;
t.faultCode = 0;
t.faultText = "";
end

function t = yoloTemplate()
t = struct();
t.stampSec = 0;
t.parcelId = 0;
t.longMm = nan;
t.shortMm = nan;
t.heightMm = nan;
t.yawDeg = 0;
t.confidence = nan;
t.qrText = "";
t.destinationCode = "";
t.isFirstDelivery = false;
t.source = "main_pc_yolo";
end

function s = normalizeRosFloor(raw)
tmpl = rosFloorTemplate();
if ~isstruct(raw)
    error("normalize_ros_floor expects a struct-like ROS2 floor state");
end

s = tmpl;
s.stampSec = scalarField(raw, ["stampSec" "stamp" "t"], tmpl.stampSec);
s.floorId = round(scalarField(raw, ["floorId" "floor" "floor_id"], tmpl.floorId));
s.tofMm = asRow(readAny(raw, ["tofMm" "tof_mm" "tof" "tofDistanceMm"], tmpl.tofMm), 8, nan);
s.tofOk = asLogicalRow(readAny(raw, ["tofOk" "tof_ok" "tofValid" "tof_valid"], isfinite(s.tofMm)), 8, false);
s.encoderCount = asRow(readAny(raw, ["encoderCount" "encoder_count" "encCount" "enc"], tmpl.encoderCount), 4, 0);
s.mmPerEncoderCount = asRow(readAny(raw, ["mmPerEncoderCount" "mm_per_encoder_count" "encoderScaleMm"], tmpl.mmPerEncoderCount), 4, 1);
s.encoderMm = asRow(readAny(raw, ["encoderMm" "encoder_mm" "encoderDistanceMm"], tmpl.encoderMm), 4, nan);
missingEncoderMm = ~isfinite(s.encoderMm);
s.encoderMm(missingEncoderMm) = s.encoderCount(missingEncoderMm) .* s.mmPerEncoderCount(missingEncoderMm);
s.encoderM = s.encoderMm / 1000;
s.tofM = s.tofMm / 1000;
s.tofGapMm = s.tofMm([1 3 5 7]);
s.tofTransferMm = s.tofMm([2 4 6 8]);
s.tofGapM = s.tofGapMm / 1000;
s.tofTransferM = s.tofTransferMm / 1000;
s.motorPwm = asRow(readAny(raw, ["motorPwm" "motor_pwm" "pwm"], tmpl.motorPwm), 4, 0);
s.motorDir = asRow(readAny(raw, ["motorDir" "motor_dir" "dir"], tmpl.motorDir), 4, 0);
s.motorEnabled = asLogicalRow(readAny(raw, ["motorEnabled" "motor_enabled" "enabled"], abs(s.motorDir) > 0), 4, false);
s.b4BarrierServoDeg = scalarField(raw, ["b4BarrierServoDeg" "barrierServoDeg" "b4_tof_barrier_servo_deg"], tmpl.b4BarrierServoDeg);
s.b4BarrierState = upper(string(readAny(raw, ["b4BarrierState" "barrierState" "barrier_state"], tmpl.b4BarrierState)));
s.b4BarrierDown = s.b4BarrierState == "DOWN";
s.b4BarrierUp = s.b4BarrierState == "UP";
s.platformZMm = scalarField(raw, ["platformZMm" "platform_z_mm" "platformZ"], tmpl.platformZMm);
s.platformFloor = scalarField(raw, ["platformFloor" "platform_floor"], tmpl.platformFloor);
s.platformTargetFloor = scalarField(raw, ["platformTargetFloor" "platform_target_floor"], tmpl.platformTargetFloor);
s.platformTiltDeg = scalarField(raw, ["platformTiltDeg" "platform_tilt_deg" "tiltDeg" "tilt_deg"], tmpl.platformTiltDeg);
s.platformTargetTiltDeg = scalarField(raw, ["platformTargetTiltDeg" "platform_target_tilt_deg" "targetTiltDeg" "target_tilt_deg"], tmpl.platformTargetTiltDeg);
s.platformZUncertaintyMm = scalarField(raw, ["platformZUncertaintyMm" "platform_z_uncertainty_mm" "z_uncertainty_mm"], tmpl.platformZUncertaintyMm);
s.platformTiltUncertaintyDeg = scalarField(raw, ["platformTiltUncertaintyDeg" "platform_tilt_uncertainty_deg" "tilt_uncertainty_deg"], tmpl.platformTiltUncertaintyDeg);
s.platformConfidence = string(readAny(raw, ["platformConfidence" "platform_confidence" "confidence"], tmpl.platformConfidence));
s.platformBusy = logicalScalar(readAny(raw, ["platformBusy" "platform_busy"], tmpl.platformBusy), tmpl.platformBusy);
s.pusherMm = scalarField(raw, ["pusherMm" "pusher_mm"], tmpl.pusherMm);
s.pusherTargetMm = scalarField(raw, ["pusherTargetMm" "pusher_target_mm"], tmpl.pusherTargetMm);
s.sidePusherMm = scalarField(raw, ["sidePusherMm" "side_pusher_mm" "waitSidePusherMm"], tmpl.sidePusherMm);
s.sidePusherTargetMm = scalarField(raw, ["sidePusherTargetMm" "side_pusher_target_mm" "waitSidePusherTargetMm"], tmpl.sidePusherTargetMm);
s.pusherMainUncertaintyMm = scalarField(raw, ["pusherMainUncertaintyMm" "main_uncertainty_mm" "pusher_main_uncertainty_mm"], tmpl.pusherMainUncertaintyMm);
s.pusherSideUncertaintyMm = scalarField(raw, ["pusherSideUncertaintyMm" "side_uncertainty_mm" "pusher_side_uncertainty_mm"], tmpl.pusherSideUncertaintyMm);
s.pusherConfidence = string(readAny(raw, ["pusherConfidence" "pusher_confidence" "confidence"], tmpl.pusherConfidence));
s.pusherBusy = logicalScalar(readAny(raw, ["pusherBusy" "pusher_busy"], tmpl.pusherBusy), tmpl.pusherBusy);
s.unloadPackages = readAny(raw, ["unloadPackages" "unload_packages" "unloadQueue" "unload_queue"], tmpl.unloadPackages);
s.unloadLayoutUncertaintyMm = scalarField(raw, ["unloadLayoutUncertaintyMm" "layout_uncertainty_mm" "unload_layout_uncertainty_mm"], tmpl.unloadLayoutUncertaintyMm);
s.unloadConfidence = string(readAny(raw, ["unloadConfidence" "unload_confidence"], tmpl.unloadConfidence));
s.estop = logicalScalar(readAny(raw, ["estop" "eStop" "emergencyStop"], tmpl.estop), tmpl.estop);
s.faultCode = round(scalarField(raw, ["faultCode" "fault_code"], tmpl.faultCode));
s.faultText = string(readAny(raw, ["faultText" "fault_text" "message"], tmpl.faultText));
s.valid = isempty(validateRosFloor(s).issues);
end

function y = normalizeYolo(raw)
tmpl = yoloTemplate();
y = tmpl;

if isnumeric(raw)
    vals = double(raw(:)');
    if numel(vals) == 4
        y.longMm = coerceMm(vals(1));
        y.shortMm = coerceMm(vals(2));
        y.heightMm = coerceMm(vals(3));
        y.yawDeg = vals(4);
    elseif numel(vals) >= 5
        y.parcelId = round(vals(1));
        y.longMm = coerceMm(vals(2));
        y.shortMm = coerceMm(vals(3));
        y.heightMm = coerceMm(vals(4));
        y.yawDeg = vals(5);
    else
        y.valid = false;
        y.measuredLoadSpec = [];
        y.issue = "YOLO numeric input must be [long short height yawDeg] or [id long short height yawDeg]";
        return;
    end
elseif isstruct(raw)
    y.stampSec = scalarField(raw, ["stampSec" "stamp" "t"], tmpl.stampSec);
    y.parcelId = round(scalarField(raw, ["parcelId" "id" "boxId" "trackingIndex"], tmpl.parcelId));
    y.longMm = coerceMm(scalarField(raw, ["longMm" "longSideMm" "lengthMm" "maxSideMm" "long" "length"], tmpl.longMm));
    y.shortMm = coerceMm(scalarField(raw, ["shortMm" "shortSideMm" "widthMm" "minSideMm" "short" "width"], tmpl.shortMm));
    y.heightMm = coerceMm(scalarField(raw, ["heightMm" "height" "hMm"], tmpl.heightMm));
    y.yawDeg = scalarField(raw, ["yawDeg" "angleDeg" "yaw"], tmpl.yawDeg);
    y.confidence = scalarField(raw, ["confidence" "score"], tmpl.confidence);
    y.qrText = string(readAny(raw, ["qrText" "qr" "code"], tmpl.qrText));
    y.destinationCode = string(readAny(raw, ["destinationCode" "destination" "deliveryCode"], tmpl.destinationCode));
    y.isFirstDelivery = logicalScalar(readAny(raw, ["isFirstDelivery" "firstDelivery" "waitAreaCandidate"], tmpl.isFirstDelivery), tmpl.isFirstDelivery);
    y.source = string(readAny(raw, "source", tmpl.source));
else
    error("normalize_yolo expects a numeric vector or a struct");
end

if y.shortMm > y.longMm
    tmp = y.longMm;
    y.longMm = y.shortMm;
    y.shortMm = tmp;
end

if y.parcelId > 0
    y.measuredLoadSpec = [y.parcelId y.longMm y.shortMm y.heightMm y.yawDeg];
else
    y.measuredLoadSpec = [y.longMm y.shortMm y.heightMm y.yawDeg];
end
validation = validateYolo(y);
y.valid = isempty(validation.issues);
y.issue = strjoin(validation.issues, "; ");
end

function S = applyYoloLoad(raw, mode)
y = normalizeYolo(raw);
if ~y.valid
    error("Cannot apply YOLO load: %s", y.issue);
end
if mode == "wait" || (mode == "auto" && y.isFirstDelivery)
    simAction = "wait_load_measured";
elseif mode == "load" || mode == "normal" || mode == "auto"
    simAction = "load_measured";
else
    error("Unknown YOLO load mode: %s", mode);
end
S = parcel_manual_core_step(simAction, y.measuredLoadSpec);
S.dtAppliedTwinAction = simAction;
S.dtAppliedYolo = y;
end

function frame = snapshotToCommandFrame(S)
if ~isstruct(S)
    error("snapshot_to_command expects a parcel_manual_core_step snapshot struct");
end

motor = readAny(S, ["dtMotorCmd" "motorCmd"], zeros(12,1));
floorCount = max(1, round(numel(motor) / 4));
frame = struct();
frame.version = "digital_twin_command_frame_20260615";
frame.floorCount = floorCount;
frame.beltNames = ["B1" "B2" "B3" "B4"];
frame.motorDir = reshapeChannels(motor, floorCount, 0);
frame.encoderTotalMm = 1000 * reshapeChannels(readAny(S, "dtEncoder", zeros(floorCount*4,1)), floorCount, 0);
frame.encoderDeltaMm = 1000 * reshapeChannels(readAny(S, "dtEncoderDelta", zeros(floorCount*4,1)), floorCount, 0);
frame.tofGapMm = 1000 * reshapeChannels(readAny(S, "dtTofGap", nan(floorCount*4,1)), floorCount, nan);
frame.tofEmpty = reshapeChannels(readAny(S, "dtTofEmpty", zeros(floorCount*4,1)), floorCount, 0) > 0.5;
frame.noHandoffMarginMm = 1000 * reshapeChannels(readAny(S, "dtNoHandoffMargin", nan(floorCount*4,1)), floorCount, nan);
frame.platformStepCmd = scalarField(S, ["dtPlatformStepCmd" "platformStepCmd"], 0);
frame.pusherStepCmd = scalarField(S, ["dtPusherStepCmd" "pusherStepCmd"], 0);
frame.waitSidePusherStepCmd = scalarField(S, ["dtWaitSidePusherStepCmd" "waitSidePusherStepCmd"], 0);
frame.b4BarrierServoDeg = asRow(readAny(S, "b4TofBarrierServoDeg", zeros(floorCount,1)), floorCount, 0);
frame.b4BarrierTarget = asRow(readAny(S, "b4TofBarrierTarget", zeros(floorCount,1)), floorCount, 0);
frame.b4BarrierPosition = asRow(readAny(S, "b4TofBarrierPos", zeros(floorCount,1)), floorCount, 0);
frame.b4BarrierMoving = asLogicalRow(readAny(S, "b4TofBarrierMoving", false(floorCount,1)), floorCount, false);
frame.activeFloor = scalarField(S, "activeFloor", 0);
frame.activeBelt = scalarField(S, "activeBelt", 0);
frame.platformFloor = scalarField(S, "platformFloor", 0);
frame.platformTargetFloor = scalarField(S, "platformTargetFloor", 0);
frame.mode = string(readAny(S, "phase", ""));
frame.statusCode = scalarField(S, "statusCode", 0);
frame.message = string(readAny(S, "message", ""));
frame.collisionFlag = logicalScalar(readAny(S, "collisionFlag", false), false);
frame.rotationFlag = logicalScalar(readAny(S, "rotationFlag", false), false);
frame.barrierFault = any(asLogicalRow(readAny(S, "b4TofBarrierFault", false(floorCount,1)), floorCount, false));
frame.readyToPublish = ~(frame.collisionFlag || frame.rotationFlag || frame.barrierFault);
end

function report = validateRosFloor(raw)
if isstruct(raw) && isfield(raw, 'valid')
    s = raw;
elseif isstruct(raw)
    s = raw;
else
    s = struct();
end
issues = strings(0,1);
tof = asRow(readAny(s, ["tofMm" "tof_mm" "tof"], nan(1,8)), 8, nan);
enc = asRow(readAny(s, ["encoderCount" "encoder_count" "encCount"], zeros(1,4)), 4, 0);
floorId = round(scalarField(s, ["floorId" "floor" "floor_id"], 1));
if floorId < 1
    issues(end+1,1) = "floorId must be positive";
end
if numel(tof) ~= 8 || any(~isfinite(tof))
    issues(end+1,1) = "tofMm must contain 8 finite values before closed-loop use";
end
if numel(enc) ~= 4 || any(~isfinite(enc))
    issues(end+1,1) = "encoderCount must contain 4 finite values";
end
report = struct("ok", isempty(issues), "issues", issues);
end

function report = validateYolo(raw)
if isstruct(raw) && isfield(raw, 'measuredLoadSpec')
    y = raw;
else
    y = normalizeYolo(raw);
end
issues = strings(0,1);
if ~(isfinite(y.longMm) && isfinite(y.shortMm) && isfinite(y.heightMm))
    issues(end+1,1) = "YOLO size fields must be finite";
end
if y.longMm <= 0 || y.shortMm <= 0 || y.heightMm <= 0
    issues(end+1,1) = "YOLO size fields must be positive";
end
if y.shortMm > 250
    issues(end+1,1) = "shortMm exceeds the 250 mm belt/platform width";
end
if y.longMm > 1200
    issues(end+1,1) = "longMm is larger than the long belt length scale";
end
if ~isfinite(y.yawDeg)
    issues(end+1,1) = "yawDeg must be finite";
end
report = struct("ok", isempty(issues), "issues", issues);
end

function value = readAny(raw, names, defaultValue)
value = defaultValue;
if ~isstruct(raw)
    return;
end
names = string(names);
for k = 1:numel(names)
    field = char(names(k));
    if isfield(raw, field)
        value = raw.(field);
        return;
    end
end
end

function value = scalarField(raw, names, defaultValue)
value = readAny(raw, names, defaultValue);
try
    if isempty(value)
        value = defaultValue;
    else
        value = double(value(1));
    end
catch
    value = defaultValue;
end
end

function value = logicalScalar(rawValue, defaultValue)
try
    if isstring(rawValue) || ischar(rawValue)
        s = lower(string(rawValue));
        value = s == "true" || s == "1" || s == "yes" || s == "on" || s == "down" || s == "up";
    else
        value = logical(rawValue(1));
    end
catch
    value = logical(defaultValue);
end
end

function arr = asRow(value, n, fillValue)
try
    arr = double(value(:))';
catch
    arr = [];
end
if numel(arr) < n
    arr(end+1:n) = fillValue;
elseif numel(arr) > n
    arr = arr(1:n);
end
end

function arr = asLogicalRow(value, n, fillValue)
try
    arr = logical(value(:))';
catch
    arr = [];
end
if numel(arr) < n
    arr(end+1:n) = fillValue;
elseif numel(arr) > n
    arr = arr(1:n);
end
end

function M = reshapeChannels(values, floorCount, fillValue)
vals = asRow(values, floorCount * 4, fillValue);
M = reshape(vals, 4, floorCount).';
end

function mm = coerceMm(value)
mm = double(value);
if isfinite(mm) && abs(mm) > 0 && abs(mm) < 5
    mm = mm * 1000;
end
end
