function cfg = parcel_manual_config(action, value)
%PARCEL_MANUAL_CONFIG Shared parameters for the parcel conveyor simulator.
%   cfg = parcel_manual_config() returns the active configuration.
%   parcel_manual_config("reset") restores the validated baseline.
%   parcel_manual_config("set", overrides) merges fields from a struct.
%   parcel_manual_config("preset", "small_dominant") applies a box mix preset.

persistent activeCfg
if isempty(activeCfg)
    activeCfg = defaultConfig();
end

if nargin < 1 || strlength(string(action)) == 0
    cfg = activeCfg;
    return;
end

cmd = lower(string(action));
if cmd == "reset"
    activeCfg = defaultConfig();
elseif cmd == "set"
    if nargin < 2 || ~isstruct(value)
        error("parcel_manual_config:set requires a struct");
    end
    activeCfg = mergeStruct(activeCfg, value);
    activeCfg = normalizeConfig(activeCfg);
elseif cmd == "preset"
    if nargin < 2
        error("parcel_manual_config:preset requires a preset name");
    end
    activeCfg = applyPreset(activeCfg, string(value));
elseif cmd == "default"
    cfg = defaultConfig();
    return;
else
    error("Unknown parcel_manual_config action: %s", cmd);
end

cfg = activeCfg;
end

function cfg = defaultConfig()
cfg.versionName = "baseline_sparse_circulation_20260615";

cfg.maxPkg = 96;
cfg.floorCount = 3;
cfg.sampleTimeSec = 0.01;

cfg.beltLengthM = [0.498 1.080 0.498 1.080];
cfg.cornerGapM = 0.250;
cfg.beltWidthM = 0.250;
cfg.loopPeriodSec = 30.0;
cfg.beltSpeedMps = 0.0;
cfg.reverseSpeedMps = 0.0;

cfg.platformSpeedMps = 0.18;
cfg.pusherSpeedMps = 0.22;
cfg.alignYawSpeedRadps = 90 * pi / 180;
cfg.pusherTravelM = 0.40;
cfg.platformStepsPerM = 12000;
cfg.pusherStepsPerM = 10000;
cfg.b4TofBarrierTravelSec = 0.25;
cfg.b4TofBarrierServoDownDeg = 0;
cfg.b4TofBarrierServoUpDeg = 90;
cfg.unloadTravelM = 0.34;
cfg.waitSidePusherTravelM = 0.34;
cfg.loadLockFraction = 0.83;
cfg.toleranceM = 1.0e-6;

cfg.floorHeightsM = [0.00 0.27 0.54];
cfg.waitAreaLengthM = 0.500;

% Korea Post box sizes used by the simulator before packageScale is applied.
% Rows are [long short height] in meters for box types 1..5.
cfg.packageScale = 0.5;
cfg.packageSizeM = [
    0.220 0.190 0.090
    0.270 0.180 0.150
    0.340 0.250 0.210
    0.410 0.310 0.280
    0.480 0.380 0.340
];

% Baseline keeps the old deterministic near-uniform type mix.
cfg.boxTypeWeights = [1 1 1 1 1];
cfg.boxTypeSeedA = 1103515245;
cfg.boxTypeSeedB = 12345;

cfg = normalizeConfig(cfg);
end

function cfg = applyPreset(cfg, presetName)
presetName = lower(presetName);
if presetName == "baseline" || presetName == "default"
    cfg = defaultConfig();
elseif presetName == "small_dominant" || presetName == "korea_post_small_dominant"
    cfg.boxTypeWeights = [0.38 0.30 0.20 0.08 0.04];
    cfg.versionName = "small_dominant_sparse_circulation";
    cfg = normalizeConfig(cfg);
elseif presetName == "large_stress"
    cfg.boxTypeWeights = [0.08 0.12 0.20 0.30 0.30];
    cfg.versionName = "large_stress_sparse_circulation";
    cfg = normalizeConfig(cfg);
else
    error("Unknown parcel manual config preset: %s", presetName);
end
end

function out = mergeStruct(base, overrides)
out = base;
names = fieldnames(overrides);
for i = 1:numel(names)
    out.(names{i}) = overrides.(names{i});
end
end

function cfg = normalizeConfig(cfg)
cfg.beltLengthM = reshape(double(cfg.beltLengthM), 1, []);
if numel(cfg.beltLengthM) ~= 4 || any(cfg.beltLengthM <= 0)
    error("cfg.beltLengthM must contain four positive lengths");
end
if cfg.floorCount < 1 || cfg.maxPkg < 1
    error("cfg.floorCount and cfg.maxPkg must be positive");
end
cfg.floorHeightsM = reshape(double(cfg.floorHeightsM), 1, []);
if numel(cfg.floorHeightsM) < cfg.floorCount
    last = cfg.floorHeightsM(end);
    step = max(0.1, median(diff([0 cfg.floorHeightsM])));
    while numel(cfg.floorHeightsM) < cfg.floorCount
        last = last + step;
        cfg.floorHeightsM(end+1) = last; %#ok<AGROW>
    end
end
cfg.packageSizeM = double(cfg.packageSizeM);
if size(cfg.packageSizeM, 2) ~= 3
    error("cfg.packageSizeM must be N x 3");
end
cfg.boxTypeWeights = reshape(double(cfg.boxTypeWeights), 1, []);
if numel(cfg.boxTypeWeights) ~= size(cfg.packageSizeM, 1) || any(cfg.boxTypeWeights < 0) || sum(cfg.boxTypeWeights) <= 0
    error("cfg.boxTypeWeights must match packageSizeM rows and have positive sum");
end
if cfg.beltSpeedMps <= 0
    cfg.beltSpeedMps = sum(cfg.beltLengthM) / cfg.loopPeriodSec;
end
if cfg.reverseSpeedMps <= 0
    cfg.reverseSpeedMps = cfg.beltSpeedMps;
end
if ~isfield(cfg, 'b4TofBarrierTravelSec') || cfg.b4TofBarrierTravelSec <= 0
    cfg.b4TofBarrierTravelSec = 0.25;
end
if ~isfield(cfg, 'b4TofBarrierServoDownDeg')
    cfg.b4TofBarrierServoDownDeg = 0;
end
if ~isfield(cfg, 'b4TofBarrierServoUpDeg')
    cfg.b4TofBarrierServoUpDeg = 90;
end
end
