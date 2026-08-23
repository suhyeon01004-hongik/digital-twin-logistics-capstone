function S = parcel_manual_core_step(action, targetId)
%PARCEL_MANUAL_CORE_STEP Stateful command-driven parcel conveyor simulation.
%
% Actions:
%   reset    Clear all packages and return to idle.
%   load     Load one random parcel using the staged sequence-buffer rule.
%   load_measured Load one measured parcel: [long short height yawDeg] mm/deg
%   manual_b4_load_measured Manual load already placed at B4 top:
%                [id floor long short height yawDeg] mm/deg
%   wait_load_measured Load one measured first-delivery parcel to wait area.
%   unload   Queue simple circulation/unload for targetId.
%   step     Advance one fixed simulation step.
%   snapshot Return current state without advancing.

persistent st

if nargin < 1 || isempty(action)
    action = "snapshot";
end
if nargin < 2
    targetId = 0;
end
action = string(action);

if isempty(st) || action == "reset"
    st = initialState();
    st = ensureRuntimeFields(st);
end

if action == "load"
    if platformIsFree(st) && st.total_loaded < st.MAX_PKG
        st.circ_complete_target_id = 0;
        st.circ_complete_floor = 0;
        st = startLoad(st);
    else
        st.last_message = "LOAD BLOCKED: platform is busy";
    end
elseif action == "wait_load"
    if platformIsFree(st) && st.total_loaded < st.MAX_PKG
        st.circ_complete_target_id = 0;
        st.circ_complete_floor = 0;
        st = startWaitAreaLoad(st);
    else
        st.last_message = "WAIT LOAD BLOCKED: platform is busy";
    end
elseif action == "load_measured"
    if platformIsFree(st) && st.total_loaded < st.MAX_PKG
        st.circ_complete_target_id = 0;
        st.circ_complete_floor = 0;
        st = startMeasuredLoad(st, targetId);
    else
        st.last_message = "LOAD MEASURED BLOCKED: platform is busy";
    end
elseif action == "manual_b4_load_measured"
    if st.total_loaded < st.MAX_PKG
        st.circ_complete_target_id = 0;
        st.circ_complete_floor = 0;
        st = startManualB4MeasuredLoad(st, targetId);
    else
        st.last_message = "MANUAL B4 LOAD BLOCKED: DB full";
    end
elseif action == "resume_manual_b4_load"
    st.circ_complete_target_id = 0;
    st.circ_complete_floor = 0;
    st = resumeManualB4Load(st, targetId);
elseif action == "wait_load_measured"
    if platformIsFree(st) && st.total_loaded < st.MAX_PKG
        st.circ_complete_target_id = 0;
        st.circ_complete_floor = 0;
        st = startMeasuredWaitAreaLoad(st, targetId);
    else
        st.last_message = "WAIT LOAD MEASURED BLOCKED: platform is busy";
    end
elseif action == "clear_wait"
    st = clearOneWaitAreaPackage(st);
elseif action == "seed_package"
    st.circ_complete_target_id = 0;
    st.circ_complete_floor = 0;
    st = seedManualPackage(st, targetId);
elseif action == "seq_package"
    st.circ_complete_target_id = 0;
    st.circ_complete_floor = 0;
    st = addSequencePackage(st, targetId);
elseif action == "unload"
    if targetId > 0
        st = requestTargetUnload(st, targetId);
    end
elseif action == "circulate"
    if st.mode == 0 && ~hasPendingLoadRoute(st) && targetId > 0
        idx = findPackageIndex(targetId, st);
        if idx > 0
            st.circ_complete_target_id = 0;
            st.circ_complete_floor = 0;
            st.current_target_id = targetId;
            st.circ_start_belt = st.pkg_belt(idx);
            st.circ_start_pos = st.pkg_pos(idx);
            st.circ_start_coord = packageLoopCoord(idx, st);
            st.circ_prev_coord = st.circ_start_coord;
            st.circ_accum = 0;
            st.circ_progress_target_id = targetId;
            st.circ_progress_pos = 0;
            st.circ_best_progress = 0;
            st.mode = 13;
            st.last_message = sprintf("LAP CIRCULATION P%d", targetId);
        else
            st.last_message = sprintf("P%d NOT FOUND", targetId);
        end
    end
elseif action == "step"
    st = advanceState(st);
elseif action == "stepn"
    n = max(1, round(double(targetId)));
    for k = 1:n
        st = advanceState(st);
    end
elseif action == "set_next_id"
    if st.mode == 0 && st.total_loaded == 0
        st.next_id = max(1, round(double(targetId)));
        st.last_message = sprintf("TEST NEXT ID %d", st.next_id);
    end
elseif action == "save_state"
    save(fullfile(tempdir, 'parcel_manual_core_state.mat'), 'st');
elseif action == "load_state"
    data = load(fullfile(tempdir, 'parcel_manual_core_state.mat'), 'st');
    st = data.st;
    st = ensureRuntimeFields(st);
elseif action == "raw_state"
    S = st;
    return;
elseif action == "restore_raw_state"
    st = ensureRuntimeFields(targetId);
end

S = snapshotState(st);
end

function st = initialState()
cfg = parcel_manual_config();
st.config_version = cfg.versionName;
st.MAX_PKG = cfg.maxPkg;
st.FLOOR_COUNT = cfg.floorCount;
st.Ts = cfg.sampleTimeSec;
st.BELT_SPEED = cfg.beltSpeedMps;
st.REVERSE_SPEED = cfg.reverseSpeedMps;
st.PLATFORM_SPEED = cfg.platformSpeedMps;
st.PUSHER_SPEED = cfg.pusherSpeedMps;
st.ALIGN_YAW_SPEED = cfg.alignYawSpeedRadps;
st.PUSHER_TRAVEL = cfg.pusherTravelM;
st.PLATFORM_STEPS_PER_M = cfg.platformStepsPerM;
st.PUSHER_STEPS_PER_M = cfg.pusherStepsPerM;
st.B4_TOF_BARRIER_TRAVEL_SEC = cfg.b4TofBarrierTravelSec;
st.B4_TOF_BARRIER_SERVO_DOWN_DEG = cfg.b4TofBarrierServoDownDeg;
st.B4_TOF_BARRIER_SERVO_UP_DEG = cfg.b4TofBarrierServoUpDeg;
st.B4_TOF_BARRIER_TOL = 0.01;
st.UNLOAD_TRAVEL = cfg.unloadTravelM;
st.LOAD_LOCK_FRACTION = cfg.loadLockFraction;
st.TOL = cfg.toleranceM;

st.pkg_id = zeros(st.MAX_PKG,1);
st.pkg_floor = zeros(st.MAX_PKG,1);
st.pkg_belt = zeros(st.MAX_PKG,1);
st.pkg_pos = zeros(st.MAX_PKG,1);
st.pkg_long = zeros(st.MAX_PKG,1);
st.pkg_short = zeros(st.MAX_PKG,1);
st.pkg_height = zeros(st.MAX_PKG,1);
st.pkg_active = zeros(st.MAX_PKG,1);
st.pkg_aligned = zeros(st.MAX_PKG,1);
st.pkg_target_belt = zeros(st.MAX_PKG,1);
st.pkg_route_phase = zeros(st.MAX_PKG,1);
st.pkg_shift_remaining = -ones(st.MAX_PKG,1);
st.pkg_close_after_shift = zeros(st.MAX_PKG,1);
st.pkg_seq_order = zeros(st.MAX_PKG,1);
st.wait_id = zeros(st.MAX_PKG,1);
st.wait_floor = zeros(st.MAX_PKG,1);
st.wait_pos = zeros(st.MAX_PKG,1);
st.wait_long = zeros(st.MAX_PKG,1);
st.wait_short = zeros(st.MAX_PKG,1);
st.wait_height = zeros(st.MAX_PKG,1);
st.wait_active = zeros(st.MAX_PKG,1);
st.wait_total = 0;
st.wait_side_pusher_pos = 0;
st.wait_side_pusher_step_cmd = 0;
st.wait_side_pusher_travel = cfg.waitSidePusherTravelM;

st.mode = 0;
st.total_loaded = 0;
st.total_unloaded = 0;
st.next_id = 1;
st.load_id = 0;
st.load_floor = 0;
st.load_belt = 0;
st.load_long = 0;
st.load_short = 0;
st.load_height = 0;
st.load_yaw = 0;
st.load_yaw0 = 0;
st.load_lane = 0.5;
st.load_stage_idx = ones(st.FLOOR_COUNT,1);
st.seq_stage_idx = ones(st.FLOOR_COUNT,1);
st.next_seq_order = 1;
st.load_shift_remaining = -1;
st.load_close_after_shift = 0;
st.load_prepare_required = 0;
st.load_prepare_steps = 0;
st.load_from_platform_contact = 0;
st.finalize_floor = 1;
st.current_target_id = 0;
st.pending_unload_id = 0;
st.circ_start_coord = 0;
st.circ_prev_coord = 0;
st.circ_accum = 0;
st.circ_start_belt = 0;
st.circ_start_pos = 0;
st.circ_compact_phase = 0;
st.circ_compact_floor = 0;
st.circ_compact_belt = 0;
st.circ_reverse_phase = 0;
st.circ_reverse_floor = 0;
st.circ_reverse_source_belt = 0;
st.circ_reverse_receiver_belt = 0;
st.circ_b4_blocker_force_floor = 0;
st.circ_last_reverse_floor = 0;
st.circ_last_reverse_source_belt = 0;
st.circ_last_reverse_repeat = 0;
st.circ_last_target_belt = 0;
st.circ_progress_target_id = 0;
st.circ_progress_pos = 0;
st.circ_best_progress = 0;
st.circ_complete_target_id = 0;
st.circ_complete_floor = 0;
st.stage_wait_id = 0;
st.stage_wait_floor = 0;
st.stage_wait_travel = 0;
st.stage_wait_remaining = 0;
st.stage_wait_restore_remaining = 0;
st.stage_wait_area_floor = 0;
st.target_yolo_mode_count = 0;
st.target_yolo_f1_seen = 0;
st.target_yolo_aligned_seen = 0;
st.target_yolo_last_id = 0;
st.temp_buffer_active = 0;
st.temp_buffer_id = 0;
st.temp_buffer_floor = 0;
st.temp_buffer_long = 0;
st.temp_buffer_short = 0;
st.temp_buffer_height = 0;
st.temp_buffer_original_floor = 0;
st.temp_buffer_original_belt = 0;
st.temp_buffer_original_pos = 0;
st.temp_reinsert_floor = 0;
st.temp_reinsert_target_belt = 0;
st.temp_unload_travel = 0;
st.temp_unload_remaining = 0;
st.temp_unload_restore_remaining = 0;
st.temp_unload_count = 0;
st.temp_reinsert_count = 0;
st.db_order_revision = 0;
st.reverse_remaining = 0;
st.restore_remaining = 0;
st.platform_z = floorHeightForTwin(1);
st.platform_floor = 1;
st.platform_target_floor = 1;
st.platform_step_cmd = 0;
st.pusher_pos = 0;
st.pusher_step_cmd = 0;
st.b4_tof_barrier_pos = zeros(st.FLOOR_COUNT,1);
st.b4_tof_barrier_target = zeros(st.FLOOR_COUNT,1);
st.b4_tof_barrier_servo_cmd_deg = st.B4_TOF_BARRIER_SERVO_DOWN_DEG * ones(st.FLOOR_COUNT,1);
st.b4_tof_barrier_moving = zeros(st.FLOOR_COUNT,1);
st.b4_tof_barrier_hold_floor = 0;
st.b4_tof_barrier_wait_count = 0;
st.b4_tof_barrier_fault = 0;
st.b4_tof_barrier_fault_count = 0;
st.b4_tof_barrier_last_fault = "";
channelCount = st.FLOOR_COUNT * 4;
st.belt_encoder = zeros(channelCount,1);
st.motor_cmd = zeros(channelCount,1);
st.encoder_delta_vec = zeros(channelCount,1);
st.active_floor = 0;
st.active_belt = 0;
st.motor_dir = 0;
st.encoder_delta = 0;
st.last_message = "IDLE";
end

function st = ensureRuntimeFields(st)
cfg = parcel_manual_config();
if ~isfield(st, 'B4_TOF_BARRIER_TRAVEL_SEC')
    st.B4_TOF_BARRIER_TRAVEL_SEC = cfg.b4TofBarrierTravelSec;
end
if ~isfield(st, 'B4_TOF_BARRIER_SERVO_DOWN_DEG')
    st.B4_TOF_BARRIER_SERVO_DOWN_DEG = cfg.b4TofBarrierServoDownDeg;
end
if ~isfield(st, 'B4_TOF_BARRIER_SERVO_UP_DEG')
    st.B4_TOF_BARRIER_SERVO_UP_DEG = cfg.b4TofBarrierServoUpDeg;
end
if ~isfield(st, 'B4_TOF_BARRIER_TOL')
    st.B4_TOF_BARRIER_TOL = 0.01;
end
if ~isfield(st, 'b4_tof_barrier_pos')
    st.b4_tof_barrier_pos = zeros(st.FLOOR_COUNT,1);
end
if ~isfield(st, 'b4_tof_barrier_target')
    st.b4_tof_barrier_target = zeros(st.FLOOR_COUNT,1);
end
if ~isfield(st, 'b4_tof_barrier_servo_cmd_deg')
    st.b4_tof_barrier_servo_cmd_deg = st.B4_TOF_BARRIER_SERVO_DOWN_DEG * ones(st.FLOOR_COUNT,1);
end
if ~isfield(st, 'b4_tof_barrier_moving')
    st.b4_tof_barrier_moving = zeros(st.FLOOR_COUNT,1);
end
if ~isfield(st, 'b4_tof_barrier_hold_floor')
    st.b4_tof_barrier_hold_floor = 0;
end
if ~isfield(st, 'b4_tof_barrier_wait_count')
    st.b4_tof_barrier_wait_count = 0;
end
if ~isfield(st, 'b4_tof_barrier_fault')
    st.b4_tof_barrier_fault = 0;
end
if ~isfield(st, 'b4_tof_barrier_fault_count')
    st.b4_tof_barrier_fault_count = 0;
end
if ~isfield(st, 'b4_tof_barrier_last_fault')
    st.b4_tof_barrier_last_fault = "";
end
st.b4_tof_barrier_pos = normalizeFloorVector(st.b4_tof_barrier_pos, st.FLOOR_COUNT, 0);
st.b4_tof_barrier_target = normalizeFloorVector(st.b4_tof_barrier_target, st.FLOOR_COUNT, 0);
st.b4_tof_barrier_servo_cmd_deg = normalizeFloorVector(st.b4_tof_barrier_servo_cmd_deg, st.FLOOR_COUNT, st.B4_TOF_BARRIER_SERVO_DOWN_DEG);
st.b4_tof_barrier_moving = normalizeFloorVector(st.b4_tof_barrier_moving, st.FLOOR_COUNT, 0);
if ~isfield(st, 'circ_compact_phase')
    st.circ_compact_phase = 0;
end
if ~isfield(st, 'wait_id')
    st.wait_id = zeros(st.MAX_PKG,1);
end
if ~isfield(st, 'pkg_seq_order')
    st.pkg_seq_order = zeros(st.MAX_PKG,1);
end
if ~isfield(st, 'seq_stage_idx')
    st.seq_stage_idx = ones(st.FLOOR_COUNT,1);
end
if ~isfield(st, 'next_seq_order')
    st.next_seq_order = max([1; st.pkg_seq_order(:)]) + 1;
end
if ~isfield(st, 'wait_floor')
    st.wait_floor = zeros(st.MAX_PKG,1);
end
if ~isfield(st, 'wait_pos')
    st.wait_pos = zeros(st.MAX_PKG,1);
end
if ~isfield(st, 'wait_long')
    st.wait_long = zeros(st.MAX_PKG,1);
end
if ~isfield(st, 'wait_short')
    st.wait_short = zeros(st.MAX_PKG,1);
end
if ~isfield(st, 'wait_height')
    st.wait_height = zeros(st.MAX_PKG,1);
end
if ~isfield(st, 'wait_active')
    st.wait_active = zeros(st.MAX_PKG,1);
end
if ~isfield(st, 'wait_total')
    st.wait_total = sum(st.wait_active > 0.5);
end
if ~isfield(st, 'wait_side_pusher_pos')
    st.wait_side_pusher_pos = 0;
end
if ~isfield(st, 'wait_side_pusher_step_cmd')
    st.wait_side_pusher_step_cmd = 0;
end
if ~isfield(st, 'wait_side_pusher_travel')
    cfg = parcel_manual_config();
    st.wait_side_pusher_travel = cfg.waitSidePusherTravelM;
end
if ~isfield(st, 'circ_compact_floor')
    st.circ_compact_floor = 0;
end
if ~isfield(st, 'circ_compact_belt')
    st.circ_compact_belt = 0;
end
if ~isfield(st, 'circ_reverse_phase')
    st.circ_reverse_phase = 0;
end
if ~isfield(st, 'circ_reverse_floor')
    st.circ_reverse_floor = 0;
end
if ~isfield(st, 'circ_reverse_source_belt')
    st.circ_reverse_source_belt = 0;
end
if ~isfield(st, 'circ_reverse_receiver_belt')
    st.circ_reverse_receiver_belt = 0;
end
if ~isfield(st, 'circ_b4_blocker_force_floor')
    st.circ_b4_blocker_force_floor = 0;
end
if ~isfield(st, 'circ_last_reverse_floor')
    st.circ_last_reverse_floor = 0;
end
if ~isfield(st, 'circ_last_reverse_source_belt')
    st.circ_last_reverse_source_belt = 0;
end
if ~isfield(st, 'circ_last_reverse_repeat')
    st.circ_last_reverse_repeat = 0;
end
if ~isfield(st, 'circ_progress_target_id')
    st.circ_progress_target_id = 0;
end
if ~isfield(st, 'circ_last_target_belt')
    st.circ_last_target_belt = 0;
end
if ~isfield(st, 'circ_complete_target_id')
    st.circ_complete_target_id = 0;
end
if ~isfield(st, 'circ_complete_floor')
    st.circ_complete_floor = 0;
end
if ~isfield(st, 'stage_wait_id')
    st.stage_wait_id = 0;
end
if ~isfield(st, 'stage_wait_floor')
    st.stage_wait_floor = 0;
end
if ~isfield(st, 'stage_wait_travel')
    st.stage_wait_travel = 0;
end
if ~isfield(st, 'stage_wait_remaining')
    st.stage_wait_remaining = 0;
end
if ~isfield(st, 'stage_wait_restore_remaining')
    st.stage_wait_restore_remaining = 0;
end
if ~isfield(st, 'stage_wait_area_floor')
    st.stage_wait_area_floor = 0;
end
if ~isfield(st, 'target_yolo_mode_count')
    st.target_yolo_mode_count = 0;
end
if ~isfield(st, 'target_yolo_f1_seen')
    st.target_yolo_f1_seen = 0;
end
if ~isfield(st, 'target_yolo_aligned_seen')
    st.target_yolo_aligned_seen = 0;
end
if ~isfield(st, 'target_yolo_last_id')
    st.target_yolo_last_id = 0;
end
if ~isfield(st, 'circ_progress_pos')
    st.circ_progress_pos = 0;
end
if ~isfield(st, 'circ_best_progress')
    st.circ_best_progress = 0;
end
if ~isfield(st, 'temp_buffer_active')
    st.temp_buffer_active = 0;
end
if ~isfield(st, 'temp_buffer_id')
    st.temp_buffer_id = 0;
end
if ~isfield(st, 'temp_buffer_floor')
    st.temp_buffer_floor = 0;
end
if ~isfield(st, 'temp_buffer_long')
    st.temp_buffer_long = 0;
end
if ~isfield(st, 'temp_buffer_short')
    st.temp_buffer_short = 0;
end
if ~isfield(st, 'temp_buffer_height')
    st.temp_buffer_height = 0;
end
if ~isfield(st, 'temp_buffer_original_floor')
    st.temp_buffer_original_floor = 0;
end
if ~isfield(st, 'temp_buffer_original_belt')
    st.temp_buffer_original_belt = 0;
end
if ~isfield(st, 'temp_buffer_original_pos')
    st.temp_buffer_original_pos = 0;
end
if ~isfield(st, 'temp_reinsert_floor')
    st.temp_reinsert_floor = 0;
end
if ~isfield(st, 'temp_reinsert_target_belt')
    st.temp_reinsert_target_belt = 0;
end
if ~isfield(st, 'temp_unload_travel')
    st.temp_unload_travel = 0;
end
if ~isfield(st, 'temp_unload_remaining')
    st.temp_unload_remaining = 0;
end
if ~isfield(st, 'temp_unload_restore_remaining')
    st.temp_unload_restore_remaining = 0;
end
if ~isfield(st, 'temp_unload_count')
    st.temp_unload_count = 0;
end
if ~isfield(st, 'temp_reinsert_count')
    st.temp_reinsert_count = 0;
end
if ~isfield(st, 'db_order_revision')
    st.db_order_revision = 0;
end
if ~isfield(st, 'load_prepare_required')
    st.load_prepare_required = 0;
end
if ~isfield(st, 'load_prepare_steps')
    st.load_prepare_steps = 0;
end
if ~isfield(st, 'load_from_platform_contact')
    st.load_from_platform_contact = 0;
end
if ~isfield(st, 'pending_unload_id')
    st.pending_unload_id = 0;
end
end

function values = normalizeFloorVector(values, floorCount, fillValue)
values = reshape(double(values), [], 1);
if numel(values) < floorCount
    values(end+1:floorCount,1) = fillValue;
elseif numel(values) > floorCount
    values = values(1:floorCount);
end
end

function st = requestTargetUnload(st, targetId)
targetId = round(double(targetId));
if targetId <= 0
    return;
end
idx = findPackageIndex(targetId, st);
if idx <= 0
    st.last_message = sprintf("P%d NOT FOUND", targetId);
    return;
end
st.circ_complete_target_id = 0;
st.circ_complete_floor = 0;
if st.mode ~= 0
    st.pending_unload_id = targetId;
    st.last_message = sprintf("UNLOAD QUEUED P%d", targetId);
    return;
end
if hasPendingLoadRoute(st)
    st.pending_unload_id = targetId;
    st.last_message = sprintf("UNLOAD QUEUED P%d: finishing load route", targetId);
    return;
end
st = startTargetUnloadCirculation(st, targetId);
end

function st = maybeStartPendingUnload(st)
if st.pending_unload_id <= 0 || st.mode ~= 0 || hasPendingLoadRoute(st)
    return;
end
targetId = st.pending_unload_id;
idx = findPackageIndex(targetId, st);
if idx <= 0
    st.pending_unload_id = 0;
    st.last_message = sprintf("P%d NOT FOUND", targetId);
    return;
end
st = startTargetUnloadCirculation(st, targetId);
end

function st = startTargetUnloadCirculation(st, targetId)
idx = findPackageIndex(targetId, st);
if idx <= 0
    st.last_message = sprintf("P%d NOT FOUND", targetId);
    return;
end
st.pending_unload_id = 0;
st.current_target_id = targetId;
st.circ_progress_target_id = targetId;
st.circ_prev_coord = packageLoopCoord(idx, st);
st.circ_progress_pos = 0;
st.circ_best_progress = 0;
st.circ_start_belt = st.pkg_belt(idx);
st.circ_start_pos = st.pkg_pos(idx);
st.circ_last_target_belt = st.pkg_belt(idx);
st.platform_target_floor = st.pkg_floor(idx);
st.mode = 6;
st.last_message = sprintf("CIRCULATING P%d", targetId);
end

function st = startLoad(st)
st.load_id = st.next_id;
st.load_floor = 1 + mod(st.load_id - 1, st.FLOOR_COUNT);
[st.load_long, st.load_short, st.load_height] = packageDims(st.load_id);
st.load_yaw0 = randomLoadYaw(st.load_id);
st.load_yaw = st.load_yaw0;
st.load_lane = platformLaneFactor(st.load_id);
st.pusher_pos = 0;
st.load_shift_remaining = -1;
st.load_close_after_shift = 0;
st = clearLoadPrepareState(st);

preferredFloor = 1 + mod(st.load_id - 1, st.FLOOR_COUNT);
[st, floor, belt] = chooseLoadTargetForPackage(preferredFloor, st.load_long, st.load_short, st);
st.load_floor = floor;
st.load_belt = belt;
if belt <= 0
    st.finalize_floor = 1;
    st.platform_target_floor = 1;
    st.mode = 12;
    st.last_message = sprintf("FINALIZING LOAD: no floor can accept P%d", st.load_id);
    clearPendingLoadFields();
    return;
end

st.platform_target_floor = st.load_floor;
st.mode = 1;
st.last_message = sprintf("MOVING PLATFORM FOR P%d", st.load_id);

    function clearPendingLoadFields()
        st.load_id = 0;
        st.load_floor = 0;
        st.load_belt = 0;
        st.load_long = 0;
        st.load_short = 0;
        st.load_height = 0;
        st.load_yaw = 0;
        st.load_yaw0 = 0;
        st = clearLoadPrepareState(st);
    end
end

function st = startMeasuredLoad(st, spec)
[id, longSide, shortSide, height, yawRad, ok, msg] = parseMeasuredLoadSpec(st, spec);
if ~ok
    st.last_message = msg;
    return;
end
if packageIdExists(st, id)
    st.last_message = sprintf("LOAD MEASURED BLOCKED: P%d already exists", id);
    return;
end

st.load_id = id;
st.load_floor = 1 + mod(st.load_id - 1, st.FLOOR_COUNT);
st.load_long = longSide;
st.load_short = shortSide;
st.load_height = height;
st.load_yaw0 = yawRad;
st.load_yaw = st.load_yaw0;
st.load_lane = platformLaneFactor(st.load_id);
st.pusher_pos = 0;
st.load_shift_remaining = -1;
st.load_close_after_shift = 0;
st = clearLoadPrepareState(st);

preferredFloor = 1 + mod(st.load_id - 1, st.FLOOR_COUNT);
[st, floor, belt] = chooseLoadTargetForPackage(preferredFloor, st.load_long, st.load_short, st);
st.load_floor = floor;
st.load_belt = belt;
if belt <= 0
    st.finalize_floor = 1;
    st.platform_target_floor = 1;
    st.mode = 12;
    st.last_message = sprintf("FINALIZING LOAD: no floor can accept measured P%d", st.load_id);
    clearPendingMeasuredLoadFields();
    return;
end

st.platform_target_floor = st.load_floor;
st.mode = 1;
st.last_message = sprintf("MOVING PLATFORM FOR MEASURED P%d", st.load_id);

    function clearPendingMeasuredLoadFields()
        st.load_id = 0;
        st.load_floor = 0;
        st.load_belt = 0;
        st.load_long = 0;
        st.load_short = 0;
        st.load_height = 0;
        st.load_yaw = 0;
        st.load_yaw0 = 0;
        st = clearLoadPrepareState(st);
    end
end

function st = startManualB4MeasuredLoad(st, spec)
vals = double(spec(:)');
if numel(vals) < 6
    st.last_message = "MANUAL B4 LOAD BLOCKED: use [id floor long short height yawDeg]";
    return;
end

floor = round(vals(2));
measuredSpec = [vals(1), vals(3), vals(4), vals(5), vals(6)];
[id, longSide, shortSide, height, yawRad, ok, msg] = parseMeasuredLoadSpec(st, measuredSpec);
if ~ok
    st.last_message = msg;
    return;
end
if floor < 1 || floor > st.FLOOR_COUNT
    st.last_message = "MANUAL B4 LOAD BLOCKED: invalid floor";
    return;
end
if packageIdExists(st, id)
    st.last_message = sprintf("MANUAL B4 LOAD BLOCKED: P%d already exists", id);
    return;
end

beforeChoose = st;
st.load_id = id;
st.load_floor = floor;
st.load_long = longSide;
st.load_short = shortSide;
st.load_height = height;
st.load_yaw0 = yawRad;
st.load_yaw = st.load_yaw0;
st.load_lane = platformLaneFactor(st.load_id);
st.pusher_pos = 0;
st.load_shift_remaining = -1;
st.load_close_after_shift = 0;
st = clearLoadPrepareState(st);

[st, belt] = chooseLoadBelt(floor, st.load_long, st.load_short, st);
if belt <= 0
    st = beforeChoose;
    st.last_message = sprintf("MANUAL B4 LOAD BLOCKED: F%d cannot accept P%d", floor, id);
    return;
end

st.load_floor = floor;
st.load_belt = belt;
st.platform_target_floor = floor;
st.load_from_platform_contact = 1;
st = commitPushedPackage(st);
if st.load_belt == 4
    st.load_shift_remaining = -1;
    st.mode = 11;
    st.last_message = sprintf("MANUAL B4 LOAD P%d SETTLE F%d B4", st.load_id, floor);
else
    st.mode = 10;
    st.last_message = sprintf("MANUAL B4 LOAD P%d ROUTE F%d B4->B%d", ...
        st.load_id, floor, st.load_belt);
end
end

function st = resumeManualB4Load(st, spec)
vals = double(spec(:)');
if numel(vals) < 6
    st.last_message = "RESUME MANUAL LOAD BLOCKED: use [id floor targetBelt long short height]";
    return;
end
id = round(vals(1));
floor = round(vals(2));
targetBelt = round(vals(3));
longSide = manualUnitToMeters(vals(4));
shortSide = manualUnitToMeters(vals(5));
height = manualUnitToMeters(vals(6));
if shortSide > longSide
    tmp = longSide;
    longSide = shortSide;
    shortSide = tmp;
end
idx = findPackageIndex(id, st);
if idx <= 0
    st.last_message = sprintf("RESUME MANUAL LOAD BLOCKED: P%d not found", id);
    return;
end
if floor < 1 || floor > st.FLOOR_COUNT || targetBelt < 1 || targetBelt > 4
    st.last_message = "RESUME MANUAL LOAD BLOCKED: invalid floor/belt";
    return;
end

st.load_id = id;
st.load_floor = floor;
st.load_belt = targetBelt;
st.load_long = longSide;
st.load_short = shortSide;
st.load_height = height;
st.load_yaw0 = 0;
st.load_yaw = 0;
st.load_lane = platformLaneFactor(st.load_id);
st.pusher_pos = 0;
st.platform_target_floor = floor;
st.load_shift_remaining = -1;
st.load_close_after_shift = 0;
st = clearLoadPrepareState(st);

st.pkg_target_belt(idx) = targetBelt;
if st.pkg_belt(idx) == targetBelt
    st.mode = 11;
    st.last_message = sprintf("RESUME MANUAL LOAD P%d SETTLE F%d B%d", id, floor, targetBelt);
else
    st.mode = 10;
    st.last_message = sprintf("RESUME MANUAL LOAD P%d ROUTE F%d B%d->B%d", ...
        id, floor, st.pkg_belt(idx), targetBelt);
end
end

function st = startWaitAreaLoad(st)
st.load_id = st.next_id;
[st.load_long, st.load_short, st.load_height] = packageDims(st.load_id);
st.load_yaw0 = randomLoadYaw(st.load_id);
st.load_yaw = st.load_yaw0;
st.load_lane = platformLaneFactor(st.load_id);
st.load_belt = 0;
st.load_shift_remaining = -1;
st.load_close_after_shift = 0;
st.wait_side_pusher_pos = 0;
floor = chooseWaitAreaFloor(st.load_short, st);
if floor <= 0
    loadedId = st.load_id;
    st.load_id = 0;
    st.load_long = 0;
    st.load_short = 0;
    st.load_height = 0;
    st.load_yaw = 0;
    st.load_yaw0 = 0;
    st.last_message = sprintf("WAIT AREA FULL FOR P%d", loadedId);
    return;
end
st.load_floor = floor;
st.platform_target_floor = floor;
st.mode = 19;
st.last_message = sprintf("WAIT AREA QR P%d -> F%d", st.load_id, floor);
end

function st = startMeasuredWaitAreaLoad(st, spec)
[id, longSide, shortSide, height, yawRad, ok, msg] = parseMeasuredLoadSpec(st, spec);
if ~ok
    st.last_message = msg;
    return;
end
if packageIdExists(st, id)
    st.last_message = sprintf("WAIT LOAD MEASURED BLOCKED: P%d already exists", id);
    return;
end
st.load_id = id;
st.load_long = longSide;
st.load_short = shortSide;
st.load_height = height;
st.load_yaw0 = yawRad;
st.load_yaw = st.load_yaw0;
st.load_lane = platformLaneFactor(st.load_id);
st.load_belt = 0;
st.load_shift_remaining = -1;
st.load_close_after_shift = 0;
st.wait_side_pusher_pos = 0;
floor = chooseWaitAreaFloor(st.load_short, st);
if floor <= 0
    loadedId = st.load_id;
    clearWaitMeasuredFields();
    st.last_message = sprintf("WAIT AREA FULL FOR MEASURED P%d", loadedId);
    return;
end
st.load_floor = floor;
st.platform_target_floor = floor;
st.mode = 19;
st.last_message = sprintf("WAIT AREA QR MEASURED P%d -> F%d", st.load_id, floor);

    function clearWaitMeasuredFields()
        st.load_id = 0;
        st.load_long = 0;
        st.load_short = 0;
        st.load_height = 0;
        st.load_yaw = 0;
        st.load_yaw0 = 0;
    end
end

function [id, longSide, shortSide, height, yawRad, ok, msg] = parseMeasuredLoadSpec(st, spec)
vals = double(spec(:)');
id = st.next_id;
longSide = 0;
shortSide = 0;
height = 0;
yawRad = 0;
ok = false;
msg = "LOAD MEASURED BLOCKED: use [long short height yawDeg] or [id long short height yawDeg]";
if numel(vals) == 3
    longSide = manualUnitToMeters(vals(1));
    shortSide = manualUnitToMeters(vals(2));
    height = manualUnitToMeters(vals(3));
elseif numel(vals) == 4
    longSide = manualUnitToMeters(vals(1));
    shortSide = manualUnitToMeters(vals(2));
    height = manualUnitToMeters(vals(3));
    yawRad = vals(4) * pi / 180;
elseif numel(vals) >= 5
    id = round(vals(1));
    longSide = manualUnitToMeters(vals(2));
    shortSide = manualUnitToMeters(vals(3));
    height = manualUnitToMeters(vals(4));
    yawRad = vals(5) * pi / 180;
else
    return;
end
if id <= 0 || longSide <= 0 || shortSide <= 0 || height <= 0
    msg = "LOAD MEASURED BLOCKED: invalid measured package";
    return;
end
if shortSide > longSide
    tmp = longSide;
    longSide = shortSide;
    shortSide = tmp;
end
ok = true;
msg = "OK";
end

function flag = packageIdExists(st, id)
flag = findPackageIndex(id, st) > 0 || any(st.wait_active > 0.5 & st.wait_id == id);
end

function st = advanceWaitAreaAlign(st)
if st.load_id <= 0
    st.mode = 0;
    return;
end
floor = chooseWaitAreaFloor(st.load_short, st);
if floor <= 0
    st.mode = 0;
    st.last_message = sprintf("WAIT AREA FULL FOR P%d", st.load_id);
    clearWaitLoadFields();
    return;
end
st.load_floor = floor;
st.platform_target_floor = 1;
atCameraFloor = platformAtFloor(st, 1);
if atCameraFloor
    st = advanceLoadYaw(st);
end
yawDone = abs(st.load_yaw) <= st.TOL;
if atCameraFloor && yawDone
    st.mode = 20;
    st.platform_target_floor = floor;
    if floor == 1
        st.last_message = sprintf("SIDE PUSH TO WAIT F%d P%d", floor, st.load_id);
    else
        st.last_message = sprintf("WAIT YOLO ALIGNED F1 P%d -> F%d", st.load_id, floor);
    end
elseif ~atCameraFloor
    st.last_message = sprintf("PLATFORM TO F1 CAMERA FOR WAIT P%d", st.load_id);
else
    st.last_message = sprintf("WAIT YOLO ALIGN F1 P%d yaw %.1f deg", ...
        st.load_id, st.load_yaw * 180 / pi);
end

    function clearWaitLoadFields()
        st.load_id = 0;
        st.load_floor = 0;
        st.load_belt = 0;
        st.load_long = 0;
        st.load_short = 0;
        st.load_height = 0;
        st.load_yaw = 0;
        st.load_yaw0 = 0;
    end
end

function st = advanceWaitAreaSidePush(st)
if st.load_id <= 0
    st.mode = 0;
    return;
end
floor = chooseWaitAreaFloor(st.load_short, st);
if floor <= 0
    st.mode = 19;
    st.last_message = sprintf("WAIT AREA FULL FOR P%d", st.load_id);
    return;
end
st.load_floor = floor;
st.platform_target_floor = floor;
if ~platformAtFloor(st, floor)
    st.last_message = sprintf("WAIT AREA PLATFORM TO F%d P%d", floor, st.load_id);
    return;
end
d = min(st.PUSHER_SPEED * st.Ts, st.wait_side_pusher_travel - st.wait_side_pusher_pos);
if d > 0
    st.wait_side_pusher_pos = st.wait_side_pusher_pos + d;
    st.wait_side_pusher_step_cmd = round(d * st.PUSHER_STEPS_PER_M);
end
if st.wait_side_pusher_pos >= st.wait_side_pusher_travel - st.TOL
    st = commitWaitAreaPackage(st, floor);
    st.mode = 21;
    st.platform_target_floor = 1;
    st.last_message = sprintf("WAIT AREA STORED P%d F%d", st.wait_id(st.next_id - 1), floor);
else
    st.last_message = sprintf("SIDE PUSHING WAIT P%d F%d", st.load_id, floor);
end
end

function st = advanceWaitAreaFinish(st)
st.platform_target_floor = 1;
d = min(st.PUSHER_SPEED * st.Ts, st.wait_side_pusher_pos);
if d > 0
    st.wait_side_pusher_pos = st.wait_side_pusher_pos - d;
    st.wait_side_pusher_step_cmd = -round(d * st.PUSHER_STEPS_PER_M);
end
pusherDone = st.wait_side_pusher_pos <= st.TOL;
platformDone = platformAtFloor(st, 1);
if pusherDone
    st.wait_side_pusher_pos = 0;
end
if pusherDone && platformDone
    st.load_id = 0;
    st.load_floor = 0;
    st.load_belt = 0;
    st.load_long = 0;
    st.load_short = 0;
    st.load_height = 0;
    st.load_yaw = 0;
    st.load_yaw0 = 0;
    st = clearLoadPrepareState(st);
    st.mode = 0;
    st.last_message = "READY AFTER WAIT AREA LOAD";
elseif pusherDone
    st.last_message = "WAIT AREA PLATFORM RETURNING";
else
    st.last_message = "WAIT AREA PUSHER RETURNING";
end
end

function st = startTargetWaitAreaStaging(st, idx)
if idx <= 0 || st.pkg_active(idx) <= 0.5 || st.pkg_belt(idx) ~= 4
    return;
end
floor = st.pkg_floor(idx);
areaFloor = chooseTargetWaitAreaFloor(floor, st.pkg_long(idx), st.pkg_short(idx), st);
if areaFloor <= 0
    st.last_message = sprintf("WAIT AREA FULL FOR TARGET P%d", st.pkg_id(idx));
    return;
end
len = axisLengthForBelt(4, st.pkg_long(idx), st.pkg_short(idx));
travel = st.pkg_pos(idx) + len / 2;
if ~tempUnloadKeepsOneBoxOnPlatform(idx, travel, st)
    st.last_message = sprintf("TARGET UNLOAD BLOCKED P%d", st.pkg_id(idx));
    return;
end
st.stage_wait_id = st.pkg_id(idx);
st.stage_wait_floor = floor;
st.stage_wait_area_floor = areaFloor;
st.stage_wait_travel = travel;
st.stage_wait_remaining = travel;
st.stage_wait_restore_remaining = travel;
st.target_yolo_mode_count = 0;
st.target_yolo_f1_seen = 0;
st.target_yolo_aligned_seen = 0;
st.target_yolo_last_id = st.pkg_id(idx);
st.load_id = st.pkg_id(idx);
st.load_floor = floor;
st.load_belt = 0;
st.load_long = st.pkg_long(idx);
st.load_short = st.pkg_short(idx);
st.load_height = st.pkg_height(idx);
st.load_yaw0 = randomLoadYaw(st.pkg_id(idx) + 9000);
st.load_yaw = st.load_yaw0;
st.load_lane = platformLaneFactor(st.load_id);
st.wait_side_pusher_pos = 0;
st.platform_target_floor = floor;
st.mode = 22;
st.last_message = sprintf("TARGET UNLOAD TO PLATFORM P%d", st.stage_wait_id);
end

function st = advanceTargetUnloadToPlatform(st, reverseStep)
idx = findPackageIndex(st.stage_wait_id, st);
floor = st.stage_wait_floor;
if idx <= 0 || floor <= 0
    st = clearTargetWaitStage(st);
    st.mode = 0;
    return;
end
st.platform_target_floor = floor;
if ~platformAtFloor(st, floor)
    st.last_message = sprintf("TARGET WAIT PLATFORM F%d P%d", floor, st.stage_wait_id);
    return;
end
d = min(reverseStep, st.stage_wait_remaining);
if d > 0
    st.pkg_pos = moveBeltSigned(st, floor, 4, -d);
    st.stage_wait_remaining = st.stage_wait_remaining - d;
    st = registerMotorMove(st, floor, 4, d, -1);
end
if st.stage_wait_remaining <= st.TOL
    st.pkg_active(idx) = 0;
    st.pkg_belt(idx) = 0;
    st.pkg_pos(idx) = 0;
    st.pkg_aligned(idx) = 0;
    st.pkg_route_phase(idx) = 0;
    st.pkg_shift_remaining(idx) = -1;
    st.pkg_close_after_shift(idx) = 0;
    st.db_order_revision = st.db_order_revision + 1;
    st.mode = 23;
    st.last_message = sprintf("TARGET ON PLATFORM P%d", st.stage_wait_id);
else
    st.last_message = sprintf("TARGET UNLOADING P%d", st.stage_wait_id);
end
end

function st = advanceTargetUnloadRestore(st, reverseStep)
floor = st.stage_wait_floor;
if floor <= 0
    st = clearTargetWaitStage(st);
    st.mode = 0;
    return;
end
d = min(reverseStep, st.stage_wait_restore_remaining);
if d > 0
    st.pkg_pos = moveBeltSigned(st, floor, 4, d);
    st.stage_wait_restore_remaining = st.stage_wait_restore_remaining - d;
    st = registerMotorMove(st, floor, 4, d, 1);
end
if st.stage_wait_restore_remaining <= st.TOL
    st.stage_wait_restore_remaining = 0;
    st.mode = 27;
    st.platform_target_floor = 1;
    st.last_message = sprintf("TARGET B4 RESTORED P%d -> F1 YOLO", st.stage_wait_id);
else
    st.last_message = sprintf("TARGET RESTORE F%d P%d", floor, st.stage_wait_id);
end
end

function st = advanceTargetWaitAreaYoloAlign(st)
if st.stage_wait_id <= 0
    st = clearTargetWaitStage(st);
    st.mode = 0;
    return;
end
st.target_yolo_mode_count = st.target_yolo_mode_count + 1;
st.target_yolo_last_id = st.stage_wait_id;
st.platform_target_floor = 1;
if ~platformAtFloor(st, 1)
    st.last_message = sprintf("TARGET CAMERA MOVE F1 P%d", st.stage_wait_id);
    return;
end
st.target_yolo_f1_seen = 1;
st = advanceLoadYaw(st);
if abs(st.load_yaw) <= st.TOL
    st.load_yaw = 0;
    st.target_yolo_aligned_seen = 1;
    st.mode = 24;
    st.platform_target_floor = st.stage_wait_area_floor;
    st.last_message = sprintf("TARGET YOLO ALIGNED P%d -> WAIT F%d", ...
        st.stage_wait_id, st.stage_wait_area_floor);
else
    st.last_message = sprintf("TARGET YOLO ALIGN F1 P%d yaw %.1f deg", ...
        st.stage_wait_id, st.load_yaw * 180 / pi);
end
end

function st = advanceTargetWaitAreaSidePush(st)
floor = st.stage_wait_area_floor;
if floor <= 0
    st = clearTargetWaitStage(st);
    st.mode = 0;
    return;
end
if waitAreaUsed(floor, st) + waitAreaAxisLength(st.load_long, st.load_short) > waitAreaLength() + st.TOL
    st.last_message = sprintf("WAIT AREA FULL F%d P%d", floor, st.stage_wait_id);
    return;
end
st.platform_target_floor = floor;
if ~platformAtFloor(st, floor)
    st.last_message = sprintf("TARGET WAIT AREA PLATFORM F%d P%d", floor, st.stage_wait_id);
    return;
end
d = min(st.PUSHER_SPEED * st.Ts, st.wait_side_pusher_travel - st.wait_side_pusher_pos);
if d > 0
    st.wait_side_pusher_pos = st.wait_side_pusher_pos + d;
    st.wait_side_pusher_step_cmd = round(d * st.PUSHER_STEPS_PER_M);
end
if st.wait_side_pusher_pos >= st.wait_side_pusher_travel - st.TOL
    st = commitExistingPackageToWaitArea(st, floor);
    st.circ_complete_target_id = st.stage_wait_id;
    st.circ_complete_floor = floor;
    st.current_target_id = 0;
    st.mode = 25;
    st.platform_target_floor = 1;
    st.last_message = sprintf("TARGET P%d STORED WAIT F%d", st.stage_wait_id, floor);
else
    st.last_message = sprintf("TARGET SIDE PUSH P%d F%d", st.stage_wait_id, floor);
end
end

function st = advanceTargetWaitAreaFinish(st)
st.platform_target_floor = 1;
d = min(st.PUSHER_SPEED * st.Ts, st.wait_side_pusher_pos);
if d > 0
    st.wait_side_pusher_pos = st.wait_side_pusher_pos - d;
    st.wait_side_pusher_step_cmd = -round(d * st.PUSHER_STEPS_PER_M);
end
pusherDone = st.wait_side_pusher_pos <= st.TOL;
platformDone = platformAtFloor(st, 1);
if pusherDone
    st.wait_side_pusher_pos = 0;
end
if pusherDone && platformDone
    doneId = st.stage_wait_id;
    doneFloor = st.circ_complete_floor;
    st = clearTargetWaitStage(st);
    st.circ_complete_target_id = doneId;
    st.circ_complete_floor = doneFloor;
    st.mode = 0;
    st.last_message = sprintf("TARGET P%d AT WAIT AREA F%d", doneId, doneFloor);
elseif pusherDone
    st.last_message = "TARGET WAIT PLATFORM RETURNING";
else
    st.last_message = sprintf("TARGET WAIT PUSHER RETURNING P%d", st.stage_wait_id);
end
end

function st = commitExistingPackageToWaitArea(st, floor)
idx = find(st.wait_active <= 0.5 & st.wait_id == 0, 1);
if isempty(idx)
    idx = find(st.wait_active <= 0.5, 1);
end
if isempty(idx)
    return;
end
axisLen = waitAreaAxisLength(st.load_long, st.load_short);
if waitAreaUsed(floor, st) + axisLen > waitAreaLength() + st.TOL
    return;
end
st = shiftWaitAreaFloorAwayFromPlatform(floor, axisLen, st);
st.wait_id(idx) = st.stage_wait_id;
st.wait_floor(idx) = floor;
st.wait_pos(idx) = waitAreaEntryPosition(axisLen);
st.wait_long(idx) = st.load_long;
st.wait_short(idx) = st.load_short;
st.wait_height(idx) = st.load_height;
st.wait_active(idx) = 1;
st.wait_total = st.wait_total + 1;
end

function st = clearTargetWaitStage(st)
st.stage_wait_id = 0;
st.stage_wait_floor = 0;
st.stage_wait_travel = 0;
st.stage_wait_remaining = 0;
st.stage_wait_restore_remaining = 0;
st.stage_wait_area_floor = 0;
st.load_id = 0;
st.load_floor = 0;
st.load_belt = 0;
st.load_long = 0;
st.load_short = 0;
st.load_height = 0;
st.load_yaw = 0;
st.load_yaw0 = 0;
end

function floor = chooseTargetWaitAreaFloor(preferredFloor, longSide, shortSide, st)
floor = 0;
order = [preferredFloor, setdiff(1:st.FLOOR_COUNT, preferredFloor, 'stable')];
axisLen = waitAreaAxisLength(longSide, shortSide);
for k = 1:numel(order)
    f = order(k);
    if waitAreaUsed(f, st) + axisLen <= waitAreaLength() + st.TOL
        floor = f;
        return;
    end
end
end

function st = commitWaitAreaPackage(st, floor)
idx = st.next_id;
axisLen = waitAreaAxisLength(st.load_long, st.load_short);
if waitAreaUsed(floor, st) + axisLen > waitAreaLength() + st.TOL
    return;
end
st = shiftWaitAreaFloorAwayFromPlatform(floor, axisLen, st);
st.wait_id(idx) = st.load_id;
st.wait_floor(idx) = floor;
st.wait_pos(idx) = waitAreaEntryPosition(axisLen);
st.wait_long(idx) = st.load_long;
st.wait_short(idx) = st.load_short;
st.wait_height(idx) = st.load_height;
st.wait_active(idx) = 1;
st.wait_total = st.wait_total + 1;
st.total_loaded = st.total_loaded + 1;
st.next_id = st.next_id + 1;
end

function st = clearOneWaitAreaPackage(st)
bestIdx = 0;
bestFloor = inf;
bestPos = inf;
for i = 1:st.MAX_PKG
    if st.wait_active(i) > 0.5
        if st.wait_floor(i) < bestFloor || ...
                (st.wait_floor(i) == bestFloor && st.wait_pos(i) < bestPos)
            bestFloor = st.wait_floor(i);
            bestPos = st.wait_pos(i);
            bestIdx = i;
        end
    end
end
if bestIdx <= 0
    st.last_message = "WAIT AREA EMPTY";
    return;
end
removedId = st.wait_id(bestIdx);
removedFloor = st.wait_floor(bestIdx);
st.wait_id(bestIdx) = 0;
st.wait_floor(bestIdx) = 0;
st.wait_pos(bestIdx) = 0;
st.wait_long(bestIdx) = 0;
st.wait_short(bestIdx) = 0;
st.wait_height(bestIdx) = 0;
st.wait_active(bestIdx) = 0;
st.wait_total = max(0, st.wait_total - 1);
st.total_unloaded = st.total_unloaded + 1;
st = compactWaitAreaFloor(removedFloor, st);
st.last_message = sprintf("WAIT AREA RELEASED P%d F%d", removedId, removedFloor);
end

function st = compactWaitAreaFloor(floor, st)
indices = find(st.wait_active > 0.5 & st.wait_floor == floor);
if isempty(indices)
    return;
end
[~, order] = sort(st.wait_pos(indices), 'ascend');
indices = indices(order);
cursor = 0;
for k = 1:numel(indices)
    i = indices(k);
    len = waitAreaAxisLength(st.wait_long(i), st.wait_short(i));
    st.wait_pos(i) = cursor + len / 2;
    cursor = cursor + len;
end
end

function floor = chooseWaitAreaFloor(axisLen, st)
floor = 0;
for f = 1:st.FLOOR_COUNT
    if waitAreaUsed(f, st) + axisLen <= waitAreaLength() + st.TOL
        floor = f;
        return;
    end
end
end

function used = waitAreaUsed(floor, st)
used = 0;
for i = 1:st.MAX_PKG
    if st.wait_active(i) > 0.5 && st.wait_floor(i) == floor
        used = used + waitAreaAxisLength(st.wait_long(i), st.wait_short(i));
    end
end
end

function len = waitAreaAxisLength(longSide, shortSide)
unused = longSide; %#ok<NASGU>
len = shortSide;
end

function pos = waitAreaEntryPosition(axisLen)
pos = axisLen / 2;
end

function st = shiftWaitAreaFloorAwayFromPlatform(floor, axisLen, st)
for i = 1:st.MAX_PKG
    if st.wait_active(i) > 0.5 && st.wait_floor(i) == floor
        st.wait_pos(i) = st.wait_pos(i) + axisLen;
    end
end
end

function len = waitAreaLength()
cfg = parcel_manual_config();
len = cfg.waitAreaLengthM;
end

function st = seedManualPackage(st, spec)
% Test-only DB input for hand-loaded circulation experiments.
% Forms:
%   [id floor belt long_mm short_mm height_mm] appends compactly on belt.
%   [id floor belt pos_mm long_mm short_mm height_mm] uses explicit center pos.
vals = double(spec(:)');
if numel(vals) < 6
    st.last_message = "SEED BLOCKED: use [id floor belt long short height] or [id floor belt pos long short height]";
    return;
end
id = round(vals(1));
floor = round(vals(2));
belt = round(vals(3));
if numel(vals) >= 7
    pos = manualUnitToMeters(vals(4));
    longSide = manualUnitToMeters(vals(5));
    shortSide = manualUnitToMeters(vals(6));
    height = manualUnitToMeters(vals(7));
else
    longSide = manualUnitToMeters(vals(4));
    shortSide = manualUnitToMeters(vals(5));
    height = manualUnitToMeters(vals(6));
    pos = appendManualSeedPosition(floor, belt, longSide, shortSide, st);
end
if id <= 0 || floor < 1 || floor > st.FLOOR_COUNT || belt < 1 || belt > 4 || ...
        longSide <= 0 || shortSide <= 0 || height <= 0
    st.last_message = "SEED BLOCKED: invalid package spec";
    return;
end
if shortSide > longSide
    tmp = longSide;
    longSide = shortSide;
    shortSide = tmp;
end
axisLen = axisLengthForBelt(belt, longSide, shortSide);
tail = pos - axisLen/2;
front = pos + axisLen/2;
% Actual DB can contain a parcel partially hanging over the outbound corner
% while ToF handoff confirmation is in progress. Allow that physical state
% during seeding, but still reject impossible positions far outside the belt.
if tail < -st.TOL || tail > beltLength(belt) + st.TOL || front > beltLength(belt) + axisLen + st.TOL
    st.last_message = sprintf("SEED BLOCKED: P%d outside F%d B%d", id, floor, belt);
    return;
end
if findPackageIndex(id, st) > 0
    st.last_message = sprintf("SEED BLOCKED: P%d already exists", id);
    return;
end
idx = find(st.pkg_active <= 0.5 & st.pkg_id == 0, 1);
if isempty(idx)
    idx = find(st.pkg_active <= 0.5, 1);
end
if isempty(idx)
    st.last_message = "SEED BLOCKED: DB full";
    return;
end
candidate = st;
candidate.pkg_id(idx) = id;
candidate.pkg_floor(idx) = floor;
candidate.pkg_belt(idx) = belt;
candidate.pkg_pos(idx) = pos;
candidate.pkg_long(idx) = longSide;
candidate.pkg_short(idx) = shortSide;
candidate.pkg_height(idx) = height;
candidate.pkg_active(idx) = 1;
candidate.pkg_aligned(idx) = 1;
candidate.pkg_target_belt(idx) = belt;
candidate.pkg_route_phase(idx) = 0;
candidate.pkg_shift_remaining(idx) = -1;
candidate.pkg_close_after_shift(idx) = 0;
candidate.pkg_seq_order(idx) = st.next_seq_order;
if detectOverlap(candidate) > 0.5
    st.last_message = sprintf("SEED BLOCKED: P%d overlaps existing boxes", id);
    return;
end
st = candidate;
st.total_loaded = sum(st.pkg_active > 0.5) + sum(st.wait_active > 0.5);
st.next_id = max(st.next_id, id + 1);
st.next_seq_order = st.next_seq_order + 1;
st.last_message = sprintf("SEEDED P%d F%d B%d", id, floor, belt);
end

function st = addSequencePackage(st, spec)
% Test-only sequence input: [floor long_mm short_mm height_mm] or
% [floor id long_mm short_mm height_mm]. Arduino test code mirrors this rule.
vals = double(spec(:)');
if numel(vals) < 4
    st.last_message = "SEQ BLOCKED: use [floor long short height] or [floor id long short height]";
    return;
end
floor = round(vals(1));
if numel(vals) >= 5
    id = round(vals(2));
    longSide = manualUnitToMeters(vals(3));
    shortSide = manualUnitToMeters(vals(4));
    height = manualUnitToMeters(vals(5));
else
    id = st.next_id;
    longSide = manualUnitToMeters(vals(2));
    shortSide = manualUnitToMeters(vals(3));
    height = manualUnitToMeters(vals(4));
end
if id <= 0 || floor < 1 || floor > st.FLOOR_COUNT || longSide <= 0 || shortSide <= 0 || height <= 0
    st.last_message = "SEQ BLOCKED: invalid package spec";
    return;
end
if shortSide > longSide
    tmp = longSide;
    longSide = shortSide;
    shortSide = tmp;
end
if findPackageIndex(id, st) > 0
    st.last_message = sprintf("SEQ BLOCKED: P%d already exists", id);
    return;
end
[belt, st] = chooseSequenceBelt(floor, longSide, shortSide, st);
if belt <= 0
    st.last_message = sprintf("SEQ BLOCKED: F%d layout full", floor);
    return;
end
idx = find(st.pkg_active <= 0.5 & st.pkg_id == 0, 1);
if isempty(idx)
    idx = find(st.pkg_active <= 0.5, 1);
end
if isempty(idx)
    st.last_message = "SEQ BLOCKED: DB full";
    return;
end
candidate = st;
candidate.pkg_id(idx) = id;
candidate.pkg_floor(idx) = floor;
candidate.pkg_belt(idx) = belt;
candidate.pkg_pos(idx) = axisLengthForBelt(belt, longSide, shortSide) / 2;
candidate.pkg_long(idx) = longSide;
candidate.pkg_short(idx) = shortSide;
candidate.pkg_height(idx) = height;
candidate.pkg_active(idx) = 1;
candidate.pkg_aligned(idx) = 1;
candidate.pkg_target_belt(idx) = belt;
candidate.pkg_route_phase(idx) = 0;
candidate.pkg_shift_remaining(idx) = -1;
candidate.pkg_close_after_shift(idx) = 0;
candidate.pkg_seq_order(idx) = st.next_seq_order;
candidate.next_seq_order = st.next_seq_order + 1;
candidate.next_id = max(st.next_id, id + 1);
candidate = rebuildSequenceLayoutForFloor(candidate, floor);
if detectOverlap(candidate) > 0.5
    st.last_message = sprintf("SEQ BLOCKED: P%d overlaps after layout", id);
    return;
end
st = candidate;
st.total_loaded = sum(st.pkg_active > 0.5) + sum(st.wait_active > 0.5);
st.last_message = sprintf("SEQ P%d -> F%d B%d", id, floor, belt);
end

function [belt, st] = chooseSequenceBelt(floor, longSide, shortSide, st)
belt = 0;
order = loadOrder();
idx = st.seq_stage_idx(floor);
while idx <= numel(order)
    candidateBelt = order(idx);
    if sequenceBeltCanAccept(floor, candidateBelt, longSide, shortSide, st)
        belt = candidateBelt;
        st.seq_stage_idx(floor) = idx;
        return;
    end
    idx = idx + 1;
    st.seq_stage_idx(floor) = idx;
end
end

function flag = sequenceBeltCanAccept(floor, belt, longSide, shortSide, st)
newLen = axisLengthForBelt(belt, longSide, shortSide);
used = sequenceBeltLoadUsed(floor, belt, st);
flag = loadStageCanAcceptLength(belt, used, newLen, st);
end

function used = sequenceBeltLoadUsed(floor, belt, st)
used = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        used = used + axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
    end
end
end

function st = rebuildSequenceLayoutForFloor(st, floor)
order = loadOrder();
for k = 1:numel(order)
    st = rebuildSequenceBeltLayout(st, floor, order(k));
end
end

function st = rebuildSequenceBeltLayout(st, floor, belt)
[idxs, n] = beltPackageIndices(floor, belt, st);
if n <= 0
    return;
end
seq = zeros(n,1);
for k = 1:n
    seq(k) = st.pkg_seq_order(idxs(k));
end
[~, order] = sort(seq, 'descend');
idxs = idxs(order);
totalLen = 0;
for k = 1:n
    i = idxs(k);
    totalLen = totalLen + axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
end
cursor = sequenceLoadTopGapForBelt(belt, totalLen);
for k = 1:n
    i = idxs(k);
    len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
    st.pkg_pos(i) = cursor + len / 2;
    cursor = cursor + len;
end
end

function flag = loadStageCanAcceptLength(belt, used, newLen, st)
if belt == 4
    flag = used + newLen <= b4UsableLoadLength() + st.TOL;
    return;
end
canMakeReceiverGapBeforeLoad = used <= beltLength(belt) - physicalCornerWidth() + st.TOL;
fitsOnBeltAfterLoad = used + newLen <= beltLength(belt) + st.TOL;
flag = canMakeReceiverGapBeforeLoad && fitsOnBeltAfterLoad;
end

function spare = loadStageResidualLength(belt, used, newLen)
if belt == 4
    cap = b4UsableLoadLength();
else
    cap = beltLength(belt);
end
spare = cap - used - newLen;
end

function topGap = sequenceLoadTopGapForBelt(belt, totalLen)
if belt == 4
    topGap = physicalCornerWidth();
else
    topGap = min(physicalCornerWidth(), max(0, beltLength(belt) - totalLen));
end
end

function v = manualUnitToMeters(v)
if abs(v) > 5
    v = v / 1000;
end
end

function pos = appendManualSeedPosition(floor, belt, longSide, shortSide, st)
cursor = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        cursor = max(cursor, st.pkg_pos(i) + len/2);
    end
end
pos = cursor + axisLengthForBelt(belt, longSide, shortSide) / 2;
end

function st = advanceState(st)
st.active_floor = 0;
st.active_belt = 0;
st.motor_dir = 0;
st.encoder_delta = 0;
st.motor_cmd = zeros(st.FLOOR_COUNT * 4,1);
st.encoder_delta_vec = zeros(st.FLOOR_COUNT * 4,1);
st.platform_step_cmd = 0;
st.pusher_step_cmd = 0;
st.wait_side_pusher_step_cmd = 0;

step = st.BELT_SPEED * st.Ts;
reverseStep = st.REVERSE_SPEED * st.Ts;
st = commandB4TofBarrierForMode(st);
st = advanceB4TofBarrier(st);
st = advancePlatform(st);

if st.mode == 0
    if st.circ_complete_target_id > 0
        st.last_message = sprintf("TARGET P%d AT WAIT AREA F%d", ...
            st.circ_complete_target_id, st.circ_complete_floor);
    else
        st.last_message = "IDLE";
    end
elseif st.mode == 1
    st = advanceLoadYaw(st);
    atFloor = platformAtFloor(st, st.load_floor);
    yawDone = abs(st.load_yaw) <= st.TOL;
    if atFloor && yawDone
        if st.load_prepare_required > 0.5
            st.mode = 26;
            st.last_message = sprintf("PRE-BACKFILL GAP PREP P%d", st.load_id);
        else
            st.mode = 3;
            st.last_message = sprintf("CLEARING B4 TOP FOR P%d", st.load_id);
        end
    elseif ~atFloor && ~yawDone
        st.last_message = sprintf("PLATFORM TO F%d + YOLO ALIGN P%d", ...
            st.load_floor, st.load_id);
    elseif ~atFloor
        st.last_message = sprintf("PLATFORM MOVING TO F%d", st.load_floor);
    else
        st.last_message = sprintf("YOLO ALIGNING P%d  yaw %.1f deg", ...
            st.load_id, st.load_yaw * 180 / pi);
    end
elseif st.mode == 3
    if ~b4TofBarrierReadyDown(st, st.load_floor)
        st = waitForB4TofBarrierDown(st, st.load_floor, ...
            sprintf("LOAD TOF P%d", st.load_id));
    else
        st = advanceB4TopClearance(st, step);
    end
elseif st.mode == 4
    if ~b4TofBarrierReadyUp(st, st.load_floor)
        st = waitForB4TofBarrierUp(st, st.load_floor, ...
            sprintf("LOAD PUSH P%d", st.load_id));
    else
        d = min(st.PUSHER_SPEED * st.Ts, st.PUSHER_TRAVEL - st.pusher_pos);
        if d > 0
            st.pusher_pos = st.pusher_pos + d;
            st.pusher_step_cmd = round(d * st.PUSHER_STEPS_PER_M);
        end
        if st.pusher_pos >= st.PUSHER_TRAVEL - st.TOL
            st = commitPushedPackage(st);
            st.mode = 5;
            st.platform_target_floor = 1;
            st.last_message = sprintf("PUSHER COMPLETE P%d", st.load_id);
        end
    end
elseif st.mode == 10
    st = advanceLoadRoute(st, step);
elseif st.mode == 11
    st = advanceLoadGapShift(st, step);
elseif st.mode == 5
    if ~b4TofBarrierReadyUp(st, st.load_floor)
        st = waitForB4TofBarrierUp(st, st.load_floor, ...
            sprintf("LOAD FINISH P%d", st.load_id));
    else
        st = advanceLoadFinish(st);
    end
elseif st.mode == 6
    if ~allB4TofBarriersReadyDown(st)
        st = waitForAllB4TofBarriersDown(st, "CIRCULATION TOF");
    else
        st = advanceSimpleCirculation(st, step);
    end
elseif st.mode == 12
    if ~allB4TofBarriersReadyDown(st)
        st = waitForAllB4TofBarriersDown(st, "LOAD FINALIZE TOF");
    else
        st = advanceLoadFinalizeGap(st, step);
    end
elseif st.mode == 13
    if ~allB4TofBarriersReadyDown(st)
        st = waitForAllB4TofBarriersDown(st, "LAP TOF");
    else
        st = advanceLapCirculation(st, step);
    end
elseif st.mode == 19
    st = advanceWaitAreaAlign(st);
elseif st.mode == 20
    st = advanceWaitAreaSidePush(st);
elseif st.mode == 21
    st = advanceWaitAreaFinish(st);
elseif st.mode == 22
    if ~b4TofBarrierReadyUp(st, st.stage_wait_floor)
        st = waitForB4TofBarrierUp(st, st.stage_wait_floor, ...
            sprintf("TARGET UNLOAD P%d", st.stage_wait_id));
    else
        st = advanceTargetUnloadToPlatform(st, reverseStep);
    end
elseif st.mode == 23
    if ~b4TofBarrierReadyUp(st, st.stage_wait_floor)
        st = waitForB4TofBarrierUp(st, st.stage_wait_floor, ...
            sprintf("TARGET RESTORE P%d", st.stage_wait_id));
    else
        st = advanceTargetUnloadRestore(st, reverseStep);
    end
elseif st.mode == 27
    st = advanceTargetWaitAreaYoloAlign(st);
elseif st.mode == 24
    st = advanceTargetWaitAreaSidePush(st);
elseif st.mode == 25
    st = advanceTargetWaitAreaFinish(st);
elseif st.mode == 26
    if ~allB4TofBarriersReadyDown(st)
        st = waitForAllB4TofBarriersDown(st, "PRE-BACKFILL TOF");
    else
        st = advanceLoadPreBackfillGapPrepare(st, step);
    end
elseif st.mode == 14
    if ~b4TofBarrierReadyUp(st, st.temp_buffer_original_floor)
        st = waitForB4TofBarrierUp(st, st.temp_buffer_original_floor, ...
            sprintf("REFUGE UNLOAD P%d", st.temp_buffer_id));
    else
        st = advanceTempUnloadToPlatform(st, reverseStep);
    end
elseif st.mode == 15
    if ~b4TofBarrierReadyUp(st, st.temp_buffer_original_floor)
        st = waitForB4TofBarrierUp(st, st.temp_buffer_original_floor, ...
            sprintf("REFUGE RESTORE P%d", st.temp_buffer_id));
    else
        st = advanceTempUnloadRestore(st, reverseStep);
    end
elseif st.mode == 16
    if ~allB4TofBarriersReadyDown(st)
        st = waitForAllB4TofBarriersDown(st, "TEMP REINSERT TOF");
    else
        st = advanceTempReinsertPrep(st, step);
    end
elseif st.mode == 17
    if ~b4TofBarrierReadyUp(st, st.temp_reinsert_floor)
        st = waitForB4TofBarrierUp(st, st.temp_reinsert_floor, ...
            sprintf("TEMP PUSH P%d", st.temp_buffer_id));
    else
        st = advanceTempReinsertPush(st);
    end
elseif st.mode == 18
    barrierFloor = b4TofBarrierUpFloorForMode(st);
    if ~b4TofBarrierReadyUp(st, barrierFloor)
        st = waitForB4TofBarrierUp(st, barrierFloor, ...
            sprintf("TEMP FINISH P%d", st.temp_buffer_id));
    else
        st = advanceTempReinsertFinish(st);
    end
elseif st.mode == 8
    idx = findPackageIndex(st.current_target_id, st);
    if idx <= 0
        st.mode = 9;
    else
        tfloor = st.pkg_floor(idx);
        if ~b4TofBarrierReadyUp(st, tfloor)
            st = waitForB4TofBarrierUp(st, tfloor, ...
                sprintf("LEGACY UNLOAD P%d", st.current_target_id));
        else
            st.active_floor = tfloor;
            st.active_belt = 4;
            d = min(reverseStep, st.reverse_remaining);
            st.pkg_pos = moveBeltSigned(st, tfloor, 4, -d);
            st.reverse_remaining = st.reverse_remaining - d;
            st = registerMotorMove(st, tfloor, 4, d, -1);
            if st.reverse_remaining <= st.TOL
                st.pkg_active(idx) = 0;
                st.pkg_belt(idx) = 0;
                st.pkg_pos(idx) = 0;
                st.pkg_aligned(idx) = 0;
                st.total_unloaded = st.total_unloaded + 1;
                st.restore_remaining = st.UNLOAD_TRAVEL;
                st.mode = 9;
            end
        end
    end
elseif st.mode == 9
    tfloor = targetFloorFromId(st.current_target_id, st.FLOOR_COUNT);
    if ~b4TofBarrierReadyUp(st, tfloor)
        st = waitForB4TofBarrierUp(st, tfloor, ...
            sprintf("LEGACY RESTORE P%d", st.current_target_id));
    else
        st.active_floor = tfloor;
        st.active_belt = 4;
        d = min(reverseStep, st.restore_remaining);
        st.pkg_pos = moveBeltSigned(st, tfloor, 4, d);
        st.restore_remaining = st.restore_remaining - d;
        st = registerMotorMove(st, tfloor, 4, d, 1);
        if st.restore_remaining <= st.TOL
            st.last_message = sprintf("UNLOADED P%d", st.current_target_id);
            st.current_target_id = 0;
            st.mode = 0;
        end
    end
end

if canAdvanceBackgroundLoadRoutes(st)
    st = advanceBackgroundLoadRoutes(st, step);
end
st = maybeStartPendingUnload(st);

st = recordB4TofBarrierDiagnostics(st);

for idx = 1:numel(st.encoder_delta_vec)
    if st.encoder_delta_vec(idx) > 0
        st.belt_encoder(idx) = st.belt_encoder(idx) + st.motor_cmd(idx) * st.encoder_delta_vec(idx);
    end
end
end

function st = commandB4TofBarrierForMode(st)
target = zeros(st.FLOOR_COUNT,1);
floor = b4TofBarrierUpFloorForMode(st);
if floor >= 1 && floor <= st.FLOOR_COUNT
    target(floor) = 1;
end
st.b4_tof_barrier_target = target;
st = refreshB4TofBarrierServoCommand(st);
end

function floor = b4TofBarrierUpFloorForMode(st)
floor = 0;
if st.mode == 4 || st.mode == 5
    floor = st.load_floor;
elseif st.mode == 14 || st.mode == 15
    floor = st.temp_buffer_original_floor;
elseif st.mode == 17 || st.mode == 18
    floor = st.temp_reinsert_floor;
    if floor <= 0
        floor = st.b4_tof_barrier_hold_floor;
    end
elseif st.mode == 22 || st.mode == 23
    floor = st.stage_wait_floor;
elseif st.mode == 8 || st.mode == 9
    floor = targetFloorFromId(st.current_target_id, st.FLOOR_COUNT);
end
end

function st = advanceB4TofBarrier(st)
if st.B4_TOF_BARRIER_TRAVEL_SEC <= 0
    st.b4_tof_barrier_pos = st.b4_tof_barrier_target;
else
    step = st.Ts / st.B4_TOF_BARRIER_TRAVEL_SEC;
    err = st.b4_tof_barrier_target - st.b4_tof_barrier_pos;
    move = min(abs(err), step) .* sign(err);
    st.b4_tof_barrier_pos = st.b4_tof_barrier_pos + move;
end
st.b4_tof_barrier_pos = min(max(st.b4_tof_barrier_pos, 0), 1);
st.b4_tof_barrier_moving = double(abs(st.b4_tof_barrier_target - st.b4_tof_barrier_pos) > st.B4_TOF_BARRIER_TOL);
st = refreshB4TofBarrierServoCommand(st);
end

function st = refreshB4TofBarrierServoCommand(st)
span = st.B4_TOF_BARRIER_SERVO_UP_DEG - st.B4_TOF_BARRIER_SERVO_DOWN_DEG;
st.b4_tof_barrier_servo_cmd_deg = st.B4_TOF_BARRIER_SERVO_DOWN_DEG + span .* st.b4_tof_barrier_target;
end

function flag = b4TofBarrierReadyDown(st, floor)
if floor < 1 || floor > st.FLOOR_COUNT
    flag = true;
    return;
end
flag = st.b4_tof_barrier_pos(floor) <= st.B4_TOF_BARRIER_TOL && ...
    st.b4_tof_barrier_target(floor) <= st.B4_TOF_BARRIER_TOL;
end

function flag = b4TofBarrierReadyUp(st, floor)
if floor < 1 || floor > st.FLOOR_COUNT
    flag = true;
    return;
end
flag = st.b4_tof_barrier_pos(floor) >= 1 - st.B4_TOF_BARRIER_TOL && ...
    st.b4_tof_barrier_target(floor) >= 1 - st.B4_TOF_BARRIER_TOL;
end

function flag = allB4TofBarriersReadyDown(st)
flag = all(st.b4_tof_barrier_pos <= st.B4_TOF_BARRIER_TOL) && ...
    all(st.b4_tof_barrier_target <= st.B4_TOF_BARRIER_TOL);
end

function st = waitForB4TofBarrierDown(st, floor, reason)
if floor >= 1 && floor <= st.FLOOR_COUNT
    st.b4_tof_barrier_target(floor) = 0;
end
st = refreshB4TofBarrierServoCommand(st);
st.b4_tof_barrier_wait_count = st.b4_tof_barrier_wait_count + 1;
st.last_message = sprintf("B4 TOF BAR DOWN F%d: %s", floor, reason);
end

function st = waitForB4TofBarrierUp(st, floor, reason)
if floor >= 1 && floor <= st.FLOOR_COUNT
    st.b4_tof_barrier_target(floor) = 1;
end
st = refreshB4TofBarrierServoCommand(st);
st.b4_tof_barrier_wait_count = st.b4_tof_barrier_wait_count + 1;
st.last_message = sprintf("B4 TOF BAR UP F%d: %s", floor, reason);
end

function st = waitForAllB4TofBarriersDown(st, reason)
st.b4_tof_barrier_target = zeros(st.FLOOR_COUNT,1);
st = refreshB4TofBarrierServoCommand(st);
st.b4_tof_barrier_wait_count = st.b4_tof_barrier_wait_count + 1;
st.last_message = sprintf("B4 TOF BAR DOWN: %s", reason);
end

function st = recordB4TofBarrierDiagnostics(st)
tofMode = st.mode == 3 || st.mode == 6 || st.mode == 12 || st.mode == 13 || ...
    st.mode == 16 || st.mode == 26 || (st.mode == 0 && hasPendingLoadRoute(st));
if tofMode && all(st.b4_tof_barrier_target <= st.B4_TOF_BARRIER_TOL) && ...
        ~allB4TofBarriersReadyDown(st) && any(abs(st.motor_cmd) > 0.5)
    st = markB4TofBarrierFault(st, "TOF motor command while barrier is not down");
end
floor = b4TofBarrierUpFloorForMode(st);
if floor >= 1 && floor <= st.FLOOR_COUNT && ~b4TofBarrierReadyUp(st, floor)
    movingThroughInterface = abs(st.pusher_step_cmd) > 0 || ...
        abs(st.motor_cmd(sensorIndex(floor, 4))) > 0.5;
    if movingThroughInterface
        st = markB4TofBarrierFault(st, "Platform/B4 transfer while barrier is not up");
    end
end
end

function st = markB4TofBarrierFault(st, message)
st.b4_tof_barrier_fault = 1;
st.b4_tof_barrier_fault_count = st.b4_tof_barrier_fault_count + 1;
st.b4_tof_barrier_last_fault = string(message);
end

function st = advanceB4TopClearance(st, step)
floor = st.load_floor;
newLen = axisLengthForBelt(4, st.load_long, st.load_short);
if movingBeltWouldRotateInbound(floor, 4, st)
    source = prevBelt(4);
    if ~movingBeltWouldRotateInbound(floor, source, st) && ...
            topGap(floor, 4, st) >= physicalCornerWidth() - st.TOL
        d = safeForwardMoveDistance(floor, source, step, st, 1);
        if d > 5.0e-5
            st = moveBeltForward(st, floor, source, d);
            st = registerMotorMove(st, floor, source, d, 1);
            st.last_message = sprintf("CLEARING B3->B4 INBOUND F%d FOR P%d", floor, st.load_id);
            return;
        end
    end
    st.last_message = sprintf("WAITING B3->B4 INBOUND F%d FOR P%d", floor, st.load_id);
    return;
end
if st.load_belt == 4
    requiredGap = newLen;
else
    requiredGap = physicalCornerWidth();
end
need = max(0, requiredGap - topGap(floor, 4, st));
if need <= st.TOL
    st.mode = 4;
    st.last_message = sprintf("PUSHER LOADING P%d TO B4", st.load_id);
    return;
end

margin = noHandoffForwardMargin(floor, 4, st);
allowHandoff = double(st.load_belt ~= 4);
if allowHandoff > 0.5
    d = safeForwardMoveDistance(floor, 4, min(step, need), st, 1);
else
    d = min([step, need, margin]);
    d = safeForwardMoveDistance(floor, 4, d, st, 0);
end
if d <= 5.0e-5
    d = 0;
end
if d > 0
    st = moveBeltForward(st, floor, 4, d);
    st = registerMotorMove(st, floor, 4, d, 1);
    st.last_message = sprintf("CLEARING B4 TOP F%d FOR P%d", floor, st.load_id);
else
    st.last_message = sprintf("WAITING B4 TOP F%d FOR P%d", floor, st.load_id);
end
end

function st = advanceLoadPreBackfillGapPrepare(st, step)
if st.load_prepare_required <= 0.5 || st.load_belt == 4 || st.load_id <= 0
    st = clearLoadPrepareState(st);
    st.mode = 3;
    st.last_message = sprintf("CLEARING B4 TOP FOR P%d", st.load_id);
    return;
end

floor = st.load_floor;
targetBelt = st.load_belt;
if preBackfillRouteGapsReady(floor, targetBelt, st)
    st = clearLoadPrepareState(st);
    st.mode = 3;
    st.last_message = sprintf("PRE-BACKFILL READY P%d", st.load_id);
    return;
end

[candidate, moved, label] = advancePreBackfillGapPrepareAction(st, floor, targetBelt, step);
if moved
    st = candidate;
    st.load_prepare_steps = st.load_prepare_steps + 1;
    st.last_message = sprintf("PRE-BACKFILL %s F%d P%d", label, floor, st.load_id);
    return;
end

st.finalize_floor = 1;
st.platform_target_floor = 1;
blockedId = st.load_id;
st = clearCurrentLoadForFinalize(st);
st.mode = 12;
st.last_message = sprintf("PRE-BACKFILL BLOCKED P%d", blockedId);
end

function [stOut, moved, label] = advancePreBackfillGapPrepareAction(st, floor, targetBelt, step)
stOut = st;
moved = false;
label = "WAIT";
if movingBeltWouldRotateInbound(floor, 4, st)
    source = prevBelt(4);
    if ~movingBeltWouldRotateInbound(floor, source, st) && ...
            topGap(floor, 4, st) >= physicalCornerWidth() - st.TOL
        d = safeForwardMoveDistance(floor, source, step, st, 1);
        if d > 5.0e-5
            [stOut, moved] = applyPreBackfillForwardMove(st, floor, source, d);
            if moved
                label = sprintf("INBOUND B%d", source);
            end
        end
    end
    return;
end
neededBelt = preBackfillNeededGapBelt(floor, targetBelt, st);
if neededBelt <= 0
    return;
end

[stOut, moved, label] = tryPreBackfillNoHandoffGapMove(st, floor, neededBelt, step);
if moved
    return;
end

[stOut, moved, label] = tryPreBackfillCompactToBottom(st, floor, targetBelt, neededBelt);
if moved
    return;
end

[belt, d] = chooseFullGapChaseMove(floor, neededBelt, step, st);
if d > 5.0e-5
    [stOut, moved] = applyPreBackfillForwardMove(st, floor, belt, d);
    if moved
        label = sprintf("CHASE B%d", belt);
        return;
    end
end

[belt, d] = chooseGapConsolidationMove(floor, step, st);
if d > 5.0e-5
    [stOut, moved] = applyPreBackfillForwardMove(st, floor, belt, d);
    if moved
        label = sprintf("CONSOLIDATE B%d", belt);
        return;
    end
end

[stOut, moved, label] = tryPreBackfillBestScoredMove(st, floor, targetBelt, step);
end

function [stOut, moved, label] = tryPreBackfillNoHandoffGapMove(st, floor, belt, step)
stOut = st;
moved = false;
label = "WAIT";
if belt <= 0 || movingBeltWouldRotateInbound(floor, belt, st)
    return;
end
need = physicalCornerWidth() - topGap(floor, belt, st);
if need <= st.TOL
    return;
end
d = min([step, need, noHandoffForwardMargin(floor, belt, st)]);
d = safeForwardMoveDistance(floor, belt, d, st, 0);
if d > 5.0e-5
    [stOut, moved] = applyPreBackfillForwardMove(st, floor, belt, d);
    if moved
        label = sprintf("DIRECT B%d", belt);
    end
end
end

function [stOut, moved, label] = tryPreBackfillCompactToBottom(st, floor, targetBelt, preferredBelt)
stOut = st;
moved = false;
label = "WAIT";
order = preBackfillCompactOrder(targetBelt, preferredBelt);
bestScore = -inf;
bestCandidate = st;
bestTravel = 0;
bestBelt = 0;
for k = 1:numel(order)
    b = order(k);
    if topGap(floor, b, st) >= physicalCornerWidth() - st.TOL
        continue;
    end
    if ~beltCanCompactToFullTopGap(floor, b, st)
        continue;
    end
    [candidate, travel, ok] = compactBeltToBottom(st, floor, b);
    if ~ok || detectOverlap(candidate) > 0.5
        continue;
    end
    score = preBackfillRouteGapScore(floor, targetBelt, candidate) - ...
        preBackfillRouteGapScore(floor, targetBelt, st);
    if b == preferredBelt
        score = score + 1;
    elseif ismember(b, preBackfillGapBelts(targetBelt))
        score = score + 0.5;
    end
    if preBackfillRouteGapsReady(floor, targetBelt, candidate)
        score = score + 100;
    end
    if score > bestScore
        bestScore = score;
        bestCandidate = candidate;
        bestTravel = travel;
        bestBelt = b;
    end
end
if bestBelt > 0 && bestScore > -inf
    stOut = bestCandidate;
    if bestTravel > st.TOL
        stOut = registerMotorMove(stOut, floor, bestBelt, bestTravel, 1);
    end
    moved = true;
    label = sprintf("COMPACT B%d", bestBelt);
end
end

function order = preBackfillCompactOrder(targetBelt, preferredBelt)
required = preBackfillGapBelts(targetBelt);
order = [preferredBelt; required(:); (1:4)'];
order = order(order >= 1 & order <= 4);
order = unique(order, 'stable');
end

function [stOut, moved, label] = tryPreBackfillBestScoredMove(st, floor, targetBelt, step)
stOut = st;
moved = false;
label = "WAIT";
baseScore = preBackfillRouteGapScore(floor, targetBelt, st);
bestScore = baseScore;
bestCandidate = st;
bestBelt = 0;
bestD = 0;
for b = 1:4
    if movingBeltWouldRotateInbound(floor, b, st)
        continue;
    end
    d = safeForwardMoveDistance(floor, b, step, st, 1);
    if d <= 5.0e-5
        continue;
    end
    candidate = moveBeltForward(st, floor, b, d);
    if detectOverlap(candidate) > 0.5
        continue;
    end
    score = preBackfillRouteGapScore(floor, targetBelt, candidate);
    if preBackfillRouteGapsReady(floor, targetBelt, candidate)
        score = score + 100;
    end
    if score > bestScore + 1.0e-5
        bestScore = score;
        bestCandidate = candidate;
        bestBelt = b;
        bestD = d;
    end
end
if bestBelt > 0
    stOut = bestCandidate;
    stOut = registerMotorMove(stOut, floor, bestBelt, bestD, 1);
    moved = true;
    label = sprintf("BEST B%d", bestBelt);
end
end

function [stOut, moved] = applyPreBackfillForwardMove(st, floor, belt, d)
stOut = st;
moved = false;
if d <= 5.0e-5 || belt <= 0
    return;
end
candidate = moveBeltForward(st, floor, belt, d);
if detectOverlap(candidate) > 0.5
    return;
end
stOut = candidate;
stOut = registerMotorMove(stOut, floor, belt, d, 1);
moved = true;
end

function st = advanceLoadFinalizeGap(st, step)
if hasPendingLoadRoute(st)
    st = advanceBackgroundLoadRoutes(st, step);
    st.last_message = "FINALIZING LOAD: waiting routed boxes";
    return;
end

if st.finalize_floor < 1
    st.finalize_floor = 1;
end
while st.finalize_floor <= st.FLOOR_COUNT && topGap(st.finalize_floor, 4, st) >= physicalCornerWidth() - st.TOL
    st.finalize_floor = st.finalize_floor + 1;
end
if st.finalize_floor > st.FLOOR_COUNT
    st.mode = 0;
    st.last_message = "LOAD FINALIZED: B4 GAP READY";
    return;
end

floor = st.finalize_floor;
need = physicalCornerWidth() - topGap(floor, 4, st);
margin = noHandoffForwardMargin(floor, 4, st);
d = min([step, need, margin]);
d = safeForwardMoveDistance(floor, 4, d, st, 0);
if d > 0
    st = moveBeltForward(st, floor, 4, d);
    st = registerMotorMove(st, floor, 4, d, 1);
    st.last_message = sprintf("FINAL B4 GAP F%d", floor);
else
    st.last_message = sprintf("LOAD FINALIZE BLOCKED F%d B4", floor);
    st.mode = 0;
end
end

function flag = canAdvanceBackgroundLoadRoutes(st)
flag = hasPendingLoadRoute(st) && ...
    allB4TofBarriersReadyDown(st) && ...
    (st.mode == 0 || st.mode == 1 || st.mode == 3 || st.mode == 4 || st.mode == 5 || st.mode == 6);
end

function flag = hasPendingLoadRoute(st)
flag = any(st.pkg_active > 0.5 & st.pkg_route_phase > 0.5);
end

function st = advanceBackgroundLoadRoutes(st, step)
st = clampTinyBlockedOverhangs(st);
madeProgress = false;
for idx = 1:st.MAX_PKG
    if st.pkg_active(idx) <= 0.5 || st.pkg_route_phase(idx) <= 0.5
        continue;
    end
    floor = st.pkg_floor(idx);
    if floorHasMotorCommand(floor, st)
        continue;
    end
    if st.pkg_route_phase(idx) == 1
        belt = st.pkg_belt(idx);
    else
        belt = st.pkg_target_belt(idx);
    end
    if movingBeltWouldRotateInbound(floor, belt, st)
        continue;
    end
    candidate = st;
    if candidate.pkg_route_phase(idx) == 1
        candidate = advanceBackgroundRouteToTarget(candidate, idx, step);
    elseif candidate.pkg_route_phase(idx) == 2
        candidate = advanceBackgroundRouteGapShift(candidate, idx, step);
    end
    if backgroundRouteMadeProgress(st, candidate, idx)
        st = candidate;
        madeProgress = true;
    end
end

if st.mode == 0 && hasPendingLoadRoute(st) && ~madeProgress
    st.last_message = "LOAD ROUTE WAIT";
end
end

function flag = backgroundRouteMadeProgress(before, after, idx)
flag = any(abs(after.motor_cmd - before.motor_cmd) > 0.5) || ...
    any(abs(after.pkg_pos - before.pkg_pos) > 1.0e-9) || ...
    after.pkg_route_phase(idx) ~= before.pkg_route_phase(idx) || ...
    abs(after.pkg_shift_remaining(idx) - before.pkg_shift_remaining(idx)) > 1.0e-9 || ...
    after.pkg_close_after_shift(idx) ~= before.pkg_close_after_shift(idx);
end

function st = clampTinyBlockedOverhangs(st)
st = clampBlockedOverhangs(st, 0.010);
end

function st = clampCirculationBlockedOverhangs(st)
st = clampBlockedOverhangs(st, 0.025);
end

function st = clampBlockedOverhangs(st, overhangTol)
backoff = 1.0e-5;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) <= 0.5
        continue;
    end
    floor = st.pkg_floor(i);
    belt = st.pkg_belt(i);
    nb = nextBelt(belt);
    if topGap(floor, nb, st) >= physicalCornerWidth() - st.TOL
        continue;
    end
    len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
    tail = st.pkg_pos(i) - len/2;
    front = st.pkg_pos(i) + len/2;
    overhang = front - beltLength(belt);
    if overhang > 0 && overhang <= overhangTol && tail < beltLength(belt) - st.TOL
        candidate = st;
        candidate.pkg_pos(i) = beltLength(belt) - len/2 - backoff;
        if detectOverlap(candidate) <= 0.5
            st = candidate;
        end
    end
end
end

function idx = chooseBackgroundRoutePackage(st)
idx = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) <= 0.5 || st.pkg_route_phase(i) <= 0.5
        continue;
    end
    floor = st.pkg_floor(i);
    if floorHasMotorCommand(floor, st)
        continue;
    end
    if st.pkg_route_phase(i) == 1
        belt = st.pkg_belt(i);
    else
        belt = st.pkg_target_belt(i);
    end
    if movingBeltWouldRotateInbound(floor, belt, st)
        continue;
    end
    idx = i;
    return;
end
end

function flag = floorHasMotorCommand(floor, st)
flag = false;
for b = 1:4
    if abs(st.motor_cmd(sensorIndex(floor, b))) > 0.5
        flag = true;
        return;
    end
end
end

function st = advanceBackgroundRouteToTarget(st, idx, step)
floor = st.pkg_floor(idx);
targetBelt = st.pkg_target_belt(idx);
if st.pkg_belt(idx) == targetBelt
    st.pkg_route_phase(idx) = 2;
    st.pkg_shift_remaining(idx) = -1;
    st = advanceBackgroundRouteGapShift(st, idx, step);
    return;
end

belt = st.pkg_belt(idx);
d = safeForwardMoveDistance(floor, belt, step, st, 1);
if d <= 5.0e-5
    d = 0;
end
if d > 0
    st = moveBeltForward(st, floor, belt, d);
    st = registerMotorMove(st, floor, belt, d, 1);
    if st.mode == 0
        st.last_message = sprintf("BACKGROUND ROUTE P%d TO B%d", ...
            st.pkg_id(idx), targetBelt);
    end
elseif topGap(floor, nextBelt(belt), st) < physicalCornerWidth() - st.TOL
    st = advanceBackgroundReceiverGap(st, floor, nextBelt(belt), step, idx);
elseif st.mode == 0
    st.last_message = sprintf("BACKGROUND ROUTE WAIT P%d", st.pkg_id(idx));
end
end

function st = advanceBackgroundReceiverGap(st, floor, belt, step, routeIdx)
st = advanceBackgroundReceiverGapDepth(st, floor, belt, step, routeIdx, 0);
end

function st = advanceBackgroundReceiverGapDepth(st, floor, belt, step, routeIdx, depth)
if depth >= 4
    if st.mode == 0
        st.last_message = sprintf("BACKGROUND RECEIVER WAIT P%d", st.pkg_id(routeIdx));
    end
    return;
end
if floorHasMotorCommand(floor, st)
    if st.mode == 0
        st.last_message = sprintf("BACKGROUND RECEIVER WAIT P%d", st.pkg_id(routeIdx));
    end
    return;
end
if movingBeltWouldRotateInbound(floor, belt, st)
    st = advanceBackgroundReceiverGapDepth(st, floor, nextBelt(belt), step, routeIdx, depth + 1);
    return;
end
d = safeForwardMoveDistance(floor, belt, step, st, 1);
if d > 0
    st = moveBeltForward(st, floor, belt, d);
    st = registerMotorMove(st, floor, belt, d, 1);
    if st.mode == 0
        st.last_message = sprintf("BACKGROUND RECEIVER GAP F%d B%d", floor, belt);
    end
elseif topGap(floor, nextBelt(belt), st) < physicalCornerWidth() - st.TOL
    st = advanceBackgroundReceiverGapDepth(st, floor, nextBelt(belt), step, routeIdx, depth + 1);
elseif st.mode == 0
    st.last_message = sprintf("BACKGROUND RECEIVER WAIT P%d", st.pkg_id(routeIdx));
end
end

function st = advanceBackgroundRouteGapShift(st, idx, step)
if st.pkg_active(idx) <= 0.5
    return;
end

floor = st.pkg_floor(idx);
belt = st.pkg_target_belt(idx);
if st.pkg_belt(idx) ~= belt
    st.pkg_route_phase(idx) = 1;
    return;
end

requiredShift = axisLengthForBelt(belt, st.pkg_long(idx), st.pkg_short(idx));
if st.pkg_shift_remaining(idx) < -0.5
    margin = noHandoffForwardMargin(floor, belt, st);
    if margin + st.TOL >= requiredShift
        st.pkg_shift_remaining(idx) = requiredShift;
        st.pkg_close_after_shift(idx) = 0;
    elseif margin > st.TOL
        st.pkg_shift_remaining(idx) = margin;
        st.pkg_close_after_shift(idx) = 1;
    else
        st = settleBackgroundRoute(st, idx);
        st = closeLoadStage(st, floor, belt);
        return;
    end
end

if st.pkg_shift_remaining(idx) > st.TOL
    margin = noHandoffForwardMargin(floor, belt, st);
    d = min([step, st.pkg_shift_remaining(idx), max(0, margin)]);
    d = safeForwardMoveDistance(floor, belt, d, st, 0);
    if d > 0
        st = moveBeltForward(st, floor, belt, d);
        st = registerMotorMove(st, floor, belt, d, 1);
        st.pkg_shift_remaining(idx) = st.pkg_shift_remaining(idx) - d;
        if st.mode == 0
            st.last_message = sprintf("BACKGROUND GAP F%d B%d", floor, belt);
        end
    else
        st = settleBackgroundRoute(st, idx);
        st = closeLoadStage(st, floor, belt);
        return;
    end
end

if st.pkg_shift_remaining(idx) <= st.TOL
    if st.pkg_close_after_shift(idx) > 0.5
        st = closeLoadStage(st, floor, belt);
    end
    st = settleBackgroundRoute(st, idx);
    if st.mode == 0
        st.last_message = sprintf("BACKGROUND SETTLED P%d", st.pkg_id(idx));
    end
end
end

function st = settleBackgroundRoute(st, idx)
st.pkg_route_phase(idx) = 0;
st.pkg_shift_remaining(idx) = -1;
st.pkg_close_after_shift(idx) = 0;
end

function st = advanceLapCirculation(st, step)
st = clampCirculationBlockedOverhangs(st);
idx = findPackageIndex(st.current_target_id, st);
if idx <= 0
    st.current_target_id = 0;
    st.mode = 0;
    return;
end

floor = st.pkg_floor(idx);
[belt, d] = chooseLapCirculationMove(floor, step, st);
if d > 0
    [st, d] = applyForwardMoveNoOverlap(st, floor, belt, d);
    if d <= st.TOL
        st.last_message = sprintf("LAP CIRC WAIT P%d", st.current_target_id);
        return;
    end
    st = registerMotorMove(st, floor, belt, d, 1);
    idx = findPackageIndex(st.current_target_id, st);
    coord = packageLoopCoord(idx, st);
    delta = coord - st.circ_prev_coord;
    if delta < -fullLoopLength() / 2
        delta = delta + fullLoopLength();
    elseif delta > fullLoopLength() / 2
        delta = delta - fullLoopLength();
    end
    if delta > 0
        st.circ_accum = st.circ_accum + delta;
    end
    st.circ_prev_coord = coord;
    if st.circ_accum >= fullLoopLength() - 0.010 && ...
            st.pkg_belt(idx) == st.circ_start_belt && ...
            abs(st.pkg_pos(idx) - st.circ_start_pos) <= 0.020
        st.last_message = sprintf("LAP COMPLETE P%d", st.current_target_id);
        st.current_target_id = 0;
        st.mode = 0;
    else
        st.last_message = sprintf("LAP CIRC %.1f%% P%d", ...
            100 * st.circ_accum / fullLoopLength(), st.current_target_id);
    end
else
    st.last_message = sprintf("LAP CIRC WAIT P%d", st.current_target_id);
end
end

function [stOut, dOut] = applyForwardMoveNoOverlap(st, floor, belt, d)
stOut = st;
dOut = d;
for iter = 1:10
    candidate = moveBeltForward(st, floor, belt, dOut);
    if detectOverlap(candidate) <= 0.5
        stOut = candidate;
        return;
    end
    dOut = dOut / 2;
    if dOut <= st.TOL
        dOut = 0;
        return;
    end
end
dOut = 0;
end

function st = advanceCirculationReverseRedistribution(st, step)
% Reverse transfer across belt corners is not physically available.
% Reverse motion may only be used for same-belt compaction or B4 unload.
st = clearReverseRedistribution(st);
return;
floor = st.circ_reverse_floor;
source = st.circ_reverse_source_belt;
receiver = st.circ_reverse_receiver_belt;
if floor <= 0 || source <= 0 || receiver <= 0
    st = clearReverseRedistribution(st);
    return;
end

if st.circ_reverse_phase == 1
    [candidate, travel, ok] = compactBeltToTop(st, floor, receiver);
    if ok && detectOverlap(candidate) <= 0.5
        st = candidate;
        if travel > st.TOL
            st = registerMotorMove(st, floor, receiver, travel, -1);
        end
        st.circ_reverse_phase = 2;
        st.last_message = sprintf("CIRC REV PREP F%d B%d", floor, receiver);
    else
        st = clearReverseRedistribution(st);
        st.last_message = sprintf("CIRC REV PREP BLOCKED F%d B%d", floor, receiver);
    end
    return;
end

if st.circ_reverse_phase == 2
    idx = topPackageOnBelt(floor, source, st);
    if idx <= 0
        st = clearReverseRedistribution(st);
        return;
    end
    len = axisLengthForBelt(source, st.pkg_long(idx), st.pkg_short(idx));
    front = st.pkg_pos(idx) + len/2;
    need = front + st.TOL;
    d = min(step, need);
    [stMoved, d] = applyReverseMoveNoOverlap(st, floor, source, d);
    if d <= st.TOL
        st = clearReverseRedistribution(st);
        st.last_message = sprintf("CIRC REV BLOCKED F%d B%d", floor, source);
        return;
    end
    st = stMoved;
    st = registerMotorMove(st, floor, source, d, -1);
    idxAfter = topPackageOnBelt(floor, source, st);
    if idxAfter <= 0 || st.pkg_belt(idx) ~= source
        st = clearReverseRedistribution(st);
        if beltCanCompactToFullTopGap(floor, source, st)
            st.circ_compact_phase = 1;
            st.circ_compact_floor = floor;
            st.circ_compact_belt = source;
        end
    end
    st.last_message = sprintf("CIRC REV MOVE F%d B%d", floor, source);
end
end

function st = clearReverseRedistribution(st)
st.circ_reverse_phase = 0;
st.circ_reverse_floor = 0;
st.circ_reverse_source_belt = 0;
st.circ_reverse_receiver_belt = 0;
end

function flag = targetReadyForB4Unload(idx, step, st)
flag = 0;
if idx <= 0 || st.pkg_belt(idx) ~= 4
    return;
end
targetLen = axisLengthForBelt(4, st.pkg_long(idx), st.pkg_short(idx));
targetTail = st.pkg_pos(idx) - targetLen/2;
targetFront = st.pkg_pos(idx) + targetLen/2;
if targetFront > physicalCornerWidth() + max(step, 0.005)
    return;
end
for i = 1:st.MAX_PKG
    if i ~= idx && st.pkg_active(i) > 0.5 && ...
            st.pkg_floor(i) == st.pkg_floor(idx) && st.pkg_belt(i) == 4
        len = axisLengthForBelt(4, st.pkg_long(i), st.pkg_short(i));
        tail = st.pkg_pos(i) - len/2;
        if tail < targetTail - st.TOL
            return;
        end
    end
end
flag = 1;
end

function [stOut, dOut] = applyReverseMoveNoOverlap(st, floor, belt, d)
stOut = st;
dOut = min(d, reverseNoHandoffMargin(floor, belt, st));
if dOut <= st.TOL
    dOut = 0;
    return;
end
for iter = 1:12
    candidate = moveBeltReverse(st, floor, belt, dOut);
    if detectOverlap(candidate) <= 0.5
        stOut = candidate;
        return;
    end
    dOut = dOut / 2;
    if dOut <= st.TOL
        dOut = 0;
        return;
    end
end
dOut = 0;
end

function [belt, d] = chooseLapCirculationMove(floor, step, st)
[belt, d] = activeOutboundCompletionMove(floor, step, st);
if belt > 0
    return;
end

idx = findPackageIndex(st.current_target_id, st);
if idx <= 0 || st.pkg_floor(idx) ~= floor
    belt = 0;
    d = 0;
    return;
end

neededGapBelt = nextBelt(st.pkg_belt(idx));
[belt, d] = chooseTargetForwardLapMove(floor, idx, step, st);
if d > 0
    return;
end

[belt, d] = chooseDownstreamGapCreationMove(floor, neededGapBelt, st.pkg_belt(idx), step, st);
if d > 0
    return;
end

[belt, d] = chooseFullGapChaseMove(floor, neededGapBelt, step, st);
if d > 0
    return;
end

[belt, d] = chooseGapConsolidationMove(floor, step, st);
if d > 0
    return;
end

belt = 0;
d = 0;
end

function [belt, d] = chooseTargetForwardLapMove(floor, idx, step, st)
belt = st.pkg_belt(idx);
d = 0;
if movingBeltWouldRotateInbound(floor, belt, st)
    belt = 0;
    return;
end
candidateD = safeForwardMoveDistance(floor, belt, step, st, 1);
if candidateD <= st.TOL
    belt = 0;
    return;
end
candidate = moveBeltForward(st, floor, belt, candidateD);
idxAfter = findPackageIndex(st.current_target_id, candidate);
if idxAfter <= 0
    belt = 0;
    return;
end
delta = forwardLoopDelta(packageLoopCoord(idx, st), packageLoopCoord(idxAfter, candidate));
if delta > st.TOL
    d = candidateD;
else
    belt = 0;
end
end

function [belt, d] = chooseDownstreamGapCreationMove(floor, neededGapBelt, targetBelt, step, st)
belt = 0;
d = 0;
b = neededGapBelt;
for k = 1:3
    if b == targetBelt
        b = nextBelt(b);
        continue;
    end
    if ~movingBeltWouldRotateInbound(floor, b, st)
        candidateD = safeForwardMoveDistance(floor, b, step, st, 1);
        if candidateD > st.TOL
            belt = b;
            d = candidateD;
            return;
        end
    end
    b = nextBelt(b);
end
end

function [belt, d] = chooseFullGapChaseMove(floor, neededGapBelt, step, st)
belt = 0;
d = 0;
bestDistance = inf;
for gapBelt = 1:4
    if topGap(floor, gapBelt, st) < physicalCornerWidth() - st.TOL
        continue;
    end
    source = prevBelt(gapBelt);
    if movingBeltWouldRotateInbound(floor, source, st)
        continue;
    end
    candidateD = safeForwardMoveDistance(floor, source, step, st, 1);
    if candidateD <= st.TOL
        continue;
    end
    dist = backwardGapDistance(gapBelt, neededGapBelt);
    if dist < bestDistance
        bestDistance = dist;
        belt = source;
        d = candidateD;
    end
end
end

function dist = backwardGapDistance(fromBelt, toBelt)
dist = 0;
b = fromBelt;
while b ~= toBelt && dist < 4
    b = prevBelt(b);
    dist = dist + 1;
end
end

function [belt, d] = chooseAnySafeLapMove(floor, idx, step, st)
order = zeros(4,1);
order(1) = st.pkg_belt(idx);
for k = 2:4
    order(k) = nextBelt(order(k-1));
end
belt = 0;
d = 0;
for k = 1:4
    b = order(k);
    if movingBeltWouldRotateInbound(floor, b, st)
        continue;
    end
    candidateD = safeForwardMoveDistance(floor, b, step, st, 1);
    if candidateD > st.TOL
        belt = b;
        d = candidateD;
        return;
    end
end
end

function delta = forwardLoopDelta(beforeCoord, afterCoord)
delta = afterCoord - beforeCoord;
if delta < -fullLoopLength() / 2
    delta = delta + fullLoopLength();
elseif delta > fullLoopLength() / 2
    delta = delta - fullLoopLength();
end
end

function [belt, d] = chooseGreedySafeLapMove(floor, step, st)
belt = 0;
d = 0;
idx = findPackageIndex(st.current_target_id, st);
if idx <= 0
    return;
end
bestScore = -inf;
for b = 1:4
    if movingBeltWouldRotateInbound(floor, b, st)
        continue;
    end
    candidateD = safeForwardMoveDistance(floor, b, step, st, 1);
    if candidateD <= st.TOL
        continue;
    end
    stCandidate = moveBeltForward(st, floor, b, candidateD);
    if detectOverlap(stCandidate) > 0.5
        continue;
    end
    idxAfter = findPackageIndex(st.current_target_id, stCandidate);
    if idxAfter <= 0
        continue;
    end
    coordBefore = packageLoopCoord(idx, st);
    coordAfter = packageLoopCoord(idxAfter, stCandidate);
    delta = coordAfter - coordBefore;
    if delta < -fullLoopLength() / 2
        delta = delta + fullLoopLength();
    elseif delta > fullLoopLength() / 2
        delta = delta - fullLoopLength();
    end
    targetGain = max(delta, 0);
    gapScore = 0;
    for q = 1:4
        gapScore = gapScore + min(topGap(floor, q, stCandidate), physicalCornerWidth());
    end
    score = 1000 * targetGain + gapScore;
    if b == st.pkg_belt(idx)
        score = score + 0.05;
    end
    if score > bestScore
        bestScore = score;
        belt = b;
        d = candidateD;
    end
end
end

function flag = movingBeltWouldRotateInbound(floor, belt, st)
flag = 0;
source = prevBelt(belt);
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == source
        len = axisLengthForBelt(source, st.pkg_long(i), st.pkg_short(i));
        tail = st.pkg_pos(i) - len/2;
        front = st.pkg_pos(i) + len/2;
        if front > beltLength(source) + st.TOL && tail < beltLength(source) - st.TOL
            flag = 1;
            return;
        end
    end
end
end

function margin = maxCandidateForwardDistance(floor, belt, st)
margin = beltLength(belt);
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        tail = st.pkg_pos(i) - len/2;
        front = st.pkg_pos(i) + len/2;
        if front <= beltLength(belt) + st.TOL
            margin = min(margin, max(0, beltLength(belt) - front + physicalCornerWidth()));
        else
            margin = min(margin, max(0, beltLength(belt) - tail + st.TOL));
        end
    end
end
margin = max(margin, 0);
end

function [belt, d] = chooseSplitGapSourceMove(floor, step, st)
belt = 0;
d = 0;
order = [1 2 3 4];
idx = findPackageIndex(st.current_target_id, st);
if idx > 0 && st.pkg_floor(idx) == floor
    targetBelt = st.pkg_belt(idx);
    order = [targetBelt, setdiff(order, targetBelt, 'stable')];
end
for oi = 1:numel(order)
    receiver = order(oi);
    receiverGap = topGap(floor, receiver, st);
    if receiverGap <= st.TOL || receiverGap >= physicalCornerWidth() - st.TOL
        continue;
    end
    source = prevBelt(receiver);
    sourceGap = topGap(floor, source, st);
    if sourceGap + receiverGap >= physicalCornerWidth() - st.TOL
        if movingBeltWouldRotateInbound(floor, source, st)
            continue;
        end
        candidateD = collisionLimitedForwardDistance(floor, source, step, st);
        if candidateD > st.TOL
            belt = source;
            d = candidateD;
            return;
        end
    end
end
end

function [belt, d] = activeOutboundCompletionMove(floor, step, st)
belt = 0;
d = 0;
bestNeed = inf;
for b = 1:4
    for i = 1:st.MAX_PKG
        if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == b
            len = axisLengthForBelt(b, st.pkg_long(i), st.pkg_short(i));
            tail = st.pkg_pos(i) - len/2;
            front = st.pkg_pos(i) + len/2;
            if front > beltLength(b) + st.TOL && tail < beltLength(b) - st.TOL
                need = beltLength(b) - tail + st.TOL;
                if need < bestNeed
                    bestNeed = need;
                    belt = b;
                    d = min(step, need);
                end
            end
        end
    end
end
end

function [belt, d] = chooseTargetBeltGapConsolidation(floor, step, st)
belt = 0;
d = 0;
idx = findPackageIndex(st.current_target_id, st);
if idx <= 0 || st.pkg_floor(idx) ~= floor
    return;
end
b = st.pkg_belt(idx);
gap = topGap(floor, b, st);
if gap > st.TOL && gap < physicalCornerWidth() - st.TOL
    need = physicalCornerWidth() - gap;
    if movingBeltWouldRotateInbound(floor, b, st)
        return;
    end
    candidateD = min([step, need, noHandoffForwardMargin(floor, b, st)]);
    if candidateD > st.TOL
        belt = b;
        d = candidateD;
    end
end
end

function [belt, d] = chooseGapConsolidationMove(floor, step, st)
belt = 0;
d = 0;
bestGap = 0;
for b = 1:4
    gap = topGap(floor, b, st);
    if gap > bestGap && gap > st.TOL && gap < physicalCornerWidth() - st.TOL
        need = physicalCornerWidth() - gap;
        if movingBeltWouldRotateInbound(floor, b, st)
            continue;
        end
        candidateD = min([step, need, noHandoffForwardMargin(floor, b, st)]);
        if candidateD > st.TOL
            bestGap = gap;
            belt = b;
            d = candidateD;
        end
    end
end
end

function belt = activeOutboundSource(floor, st)
belt = 0;
for b = 1:4
    for i = 1:st.MAX_PKG
        if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == b
            len = axisLengthForBelt(b, st.pkg_long(i), st.pkg_short(i));
            tail = st.pkg_pos(i) - len/2;
            front = st.pkg_pos(i) + len/2;
            if front > beltLength(b) + st.TOL && tail < beltLength(b) - st.TOL
                belt = b;
                return;
            end
        end
    end
end
end

function belt = circulationGapBelt(floor, st)
belt = 0;
bestGap = -inf;
for b = 1:4
    gap = topGap(floor, b, st);
    if gap >= physicalCornerWidth() - st.TOL && gap > bestGap
        bestGap = gap;
        belt = b;
    end
end
end

function st = advanceLoadRoute(st, step)
idx = findPackageIndex(st.load_id, st);
if idx <= 0
    st.mode = 5;
    return;
end
if st.pkg_belt(idx) == st.load_belt
    st.load_shift_remaining = -1;
    st.mode = 11;
    st.last_message = sprintf("P%d ARRIVED B%d", st.load_id, st.load_belt);
    return;
end

belt = st.pkg_belt(idx);
d = safeForwardMoveDistance(st.load_floor, belt, step, st, 1);
if d > 0
    st = moveBeltForward(st, st.load_floor, belt, d);
    st = registerMotorMove(st, st.load_floor, belt, d, 1);
    st.last_message = sprintf("ROUTING P%d F%d B%d->B%d", ...
        st.load_id, st.load_floor, belt, nextBelt(belt));
else
    st.last_message = sprintf("LOAD WAIT: F%d B%d receiver gap", st.load_floor, belt);
end
end

function st = advanceLoadGapShift(st, step)
idx = findPackageIndex(st.load_id, st);
if idx <= 0
    st.mode = 5;
    return;
end

floor = st.load_floor;
belt = st.load_belt;
requiredShift = axisLengthForBelt(belt, st.pkg_long(idx), st.pkg_short(idx));
if st.load_shift_remaining < -0.5
    margin = noHandoffForwardMargin(floor, belt, st);
    if margin + st.TOL >= requiredShift
        st.load_shift_remaining = requiredShift;
        st.load_close_after_shift = 0;
        st.last_message = sprintf("MAKING NEXT GAP F%d B%d", floor, belt);
    elseif margin > st.TOL
        st.load_shift_remaining = margin;
        st.load_close_after_shift = 1;
        st.last_message = sprintf("COMPACTING FINAL LOAD F%d B%d", floor, belt);
    else
        st = closeLoadStage(st, floor, belt);
        st.mode = 5;
        st.platform_target_floor = 1;
        st.last_message = sprintf("F%d B%d FULL AFTER P%d", floor, belt, st.load_id);
        return;
    end
end

if st.load_shift_remaining > st.TOL
    margin = noHandoffForwardMargin(floor, belt, st);
    d = min([step, st.load_shift_remaining, max(0, margin)]);
    d = safeForwardMoveDistance(floor, belt, d, st, 0);
    if d > 0
        st = moveBeltForward(st, floor, belt, d);
        st = registerMotorMove(st, floor, belt, d, 1);
        st.load_shift_remaining = st.load_shift_remaining - d;
        st.last_message = sprintf("MAKING NEXT GAP F%d B%d", floor, belt);
    else
        st = closeLoadStage(st, floor, belt);
        st.load_shift_remaining = 0;
        st.last_message = sprintf("F%d B%d FULL AFTER P%d", floor, belt, st.load_id);
    end
end

if st.load_shift_remaining <= st.TOL
    if st.load_close_after_shift > 0.5
        st = closeLoadStage(st, floor, belt);
        st.load_close_after_shift = 0;
        st.mode = 5;
        st.platform_target_floor = 1;
        st.last_message = sprintf("F%d B%d FULL AFTER P%d", floor, belt, st.load_id);
        return;
    end
    st.load_shift_remaining = -1;
    st.mode = 5;
    st.platform_target_floor = 1;
    st.last_message = sprintf("LOAD SETTLED P%d F%d B%d", st.load_id, floor, belt);
end
end

function st = advanceLoadFinish(st)
st.platform_target_floor = 1;
d = min(st.PUSHER_SPEED * st.Ts, st.pusher_pos);
if d > 0
    st.pusher_pos = st.pusher_pos - d;
    st.pusher_step_cmd = -round(d * st.PUSHER_STEPS_PER_M);
end
pusherDone = st.pusher_pos <= st.TOL;
platformDone = platformAtFloor(st, 1);
if pusherDone
    st.pusher_pos = 0;
end
if pusherDone && platformDone
    loadedId = st.load_id;
    st.load_id = 0;
    st.load_floor = 0;
    st.load_belt = 0;
    st.load_long = 0;
    st.load_short = 0;
    st.load_height = 0;
    st.load_yaw = 0;
    st.load_yaw0 = 0;
    st.mode = 0;
    st.last_message = sprintf("READY AFTER P%d", loadedId);
elseif pusherDone
    st.last_message = sprintf("PLATFORM RETURNING F1 AFTER P%d", st.load_id);
else
    st.last_message = sprintf("PUSHER RETURNING P%d", st.load_id);
end
end

function st = startTempBufferUnload(st, blockerIdx)
if blockerIdx <= 0 || st.temp_buffer_active > 0.5
    return;
end
if st.pkg_active(blockerIdx) <= 0.5 || st.pkg_belt(blockerIdx) ~= 4
    return;
end
if st.pkg_id(blockerIdx) == st.current_target_id
    return;
end
len = axisLengthForBelt(4, st.pkg_long(blockerIdx), st.pkg_short(blockerIdx));
front = st.pkg_pos(blockerIdx) + len/2;
if front > tempBufferTravelLimit(st) + st.TOL
    return;
end
travel = front;
if ~tempUnloadKeepsOneBoxOnPlatform(blockerIdx, travel, st)
    return;
end
[slotFloor, ~] = chooseTempReinsertSlotForPackage(st.pkg_floor(blockerIdx), ...
    st.pkg_long(blockerIdx), st.pkg_short(blockerIdx), st);
if slotFloor <= 0
    return;
end
st.temp_buffer_id = st.pkg_id(blockerIdx);
st.temp_buffer_original_floor = st.pkg_floor(blockerIdx);
st.temp_buffer_original_belt = st.pkg_belt(blockerIdx);
st.temp_buffer_original_pos = st.pkg_pos(blockerIdx);
st.temp_buffer_long = st.pkg_long(blockerIdx);
st.temp_buffer_short = st.pkg_short(blockerIdx);
st.temp_buffer_height = st.pkg_height(blockerIdx);
st.temp_buffer_floor = st.pkg_floor(blockerIdx);
st.temp_reinsert_floor = 0;
st.temp_reinsert_target_belt = 0;
st.temp_unload_travel = travel;
st.temp_unload_remaining = travel;
st.platform_target_floor = st.pkg_floor(blockerIdx);
st.mode = 14;
st.last_message = sprintf("REFUGE UNLOAD P%d", st.temp_buffer_id);
end

function st = advanceTempUnloadToPlatform(st, reverseStep)
idx = findPackageIndex(st.temp_buffer_id, st);
floor = st.temp_buffer_original_floor;
if idx <= 0 || floor <= 0
    st = clearTempBufferState(st);
    st.mode = 6;
    return;
end
st.platform_target_floor = floor;
if ~platformAtFloor(st, floor)
    st.last_message = sprintf("REFUGE WAIT PLATFORM F%d P%d", floor, st.temp_buffer_id);
    return;
end
d = min(reverseStep, st.temp_unload_remaining);
if d > 0
    st.pkg_pos = moveBeltSigned(st, floor, 4, -d);
    st.temp_unload_remaining = st.temp_unload_remaining - d;
    st = registerMotorMove(st, floor, 4, d, -1);
end
if st.temp_unload_remaining <= st.TOL
    st.pkg_active(idx) = 0;
    st.pkg_belt(idx) = 0;
    st.pkg_pos(idx) = 0;
    st.pkg_aligned(idx) = 0;
    st.pkg_route_phase(idx) = 0;
    st.pkg_shift_remaining(idx) = -1;
    st.pkg_close_after_shift(idx) = 0;
    st.temp_buffer_active = 1;
    st.temp_unload_count = st.temp_unload_count + 1;
    st.db_order_revision = st.db_order_revision + 1;
    st.temp_unload_restore_remaining = st.temp_unload_travel;
    st.mode = 15;
    st.last_message = sprintf("REFUGE ON PLATFORM P%d", st.temp_buffer_id);
else
    st.last_message = sprintf("REFUGE UNLOADING P%d", st.temp_buffer_id);
end
end

function st = advanceTempUnloadRestore(st, reverseStep)
floor = st.temp_buffer_original_floor;
if floor <= 0
    st.mode = 6;
    return;
end
d = min(reverseStep, st.temp_unload_restore_remaining);
if d > 0
    st.pkg_pos = moveBeltSigned(st, floor, 4, d);
    st.temp_unload_restore_remaining = st.temp_unload_restore_remaining - d;
    st = registerMotorMove(st, floor, 4, d, 1);
end
if st.temp_unload_restore_remaining <= st.TOL
    st.temp_unload_restore_remaining = 0;
    st.mode = 16;
    st.last_message = sprintf("REFUGE SOURCE RESTORED P%d", st.temp_buffer_id);
else
    st.last_message = sprintf("REFUGE SOURCE RESTORE F%d P%d", floor, st.temp_buffer_id);
end
end

function st = advanceTempReinsertPrep(st, step)
st = clampCirculationBlockedOverhangs(st);
if st.temp_buffer_active <= 0.5
    st.mode = 6;
    return;
end
floor = st.temp_reinsert_floor;
targetBelt = st.temp_reinsert_target_belt;
if floor <= 0 || targetBelt <= 0 || ~tempReinsertSlotStillValid(floor, targetBelt, st)
    [floor, targetBelt] = chooseTempReinsertSlot(st);
    st.temp_reinsert_floor = floor;
    st.temp_reinsert_target_belt = targetBelt;
end
if floor <= 0
    st.last_message = sprintf("TEMP REINSERT WAIT P%d", st.temp_buffer_id);
    return;
end
st.platform_target_floor = floor;
needGap = axisLengthForBelt(4, st.temp_buffer_long, st.temp_buffer_short);
[st, moved, ready] = prepareTempReinsertMeetGap(st, floor, needGap, step);
if moved > 0.5
    return;
end
if ready <= 0.5
    st.last_message = sprintf("TEMP REINSERT GAP BLOCKED F%d P%d", floor, st.temp_buffer_id);
    return;
end
if ~platformAtFloor(st, floor)
    st.last_message = sprintf("TEMP REINSERT PLATFORM F%d P%d", floor, st.temp_buffer_id);
    return;
end
st.mode = 17;
st.last_message = sprintf("TEMP REINSERT PUSH P%d F%d", st.temp_buffer_id, floor);
end

function st = advanceTempReinsertPush(st)
if st.temp_buffer_active <= 0.5
    st.mode = 6;
    return;
end
floor = st.temp_reinsert_floor;
targetBelt = st.temp_reinsert_target_belt;
if floor <= 0 || targetBelt <= 0 || ~platformAtFloor(st, floor) || ...
        ~tempReinsertSlotCanAccept(floor, targetBelt, st)
    st.mode = 16;
    return;
end
d = min(st.PUSHER_SPEED * st.Ts, st.PUSHER_TRAVEL - st.pusher_pos);
if d > 0
    st.pusher_pos = st.pusher_pos + d;
    st.pusher_step_cmd = round(d * st.PUSHER_STEPS_PER_M);
end
if st.pusher_pos >= st.PUSHER_TRAVEL - st.TOL
    barrierHoldFloor = floor;
    st = commitTempBufferedPackage(st);
    st.b4_tof_barrier_hold_floor = barrierHoldFloor;
    targetIdx = findPackageIndex(st.current_target_id, st);
    if targetIdx > 0
        st.platform_target_floor = st.pkg_floor(targetIdx);
    else
        st.platform_target_floor = 1;
    end
    st.mode = 18;
    st.last_message = "TEMP REINSERT COMPLETE";
else
    st.last_message = sprintf("TEMP PUSHING P%d", st.temp_buffer_id);
end
end

function st = advanceTempReinsertFinish(st)
d = min(st.PUSHER_SPEED * st.Ts, st.pusher_pos);
if d > 0
    st.pusher_pos = st.pusher_pos - d;
    st.pusher_step_cmd = -round(d * st.PUSHER_STEPS_PER_M);
end
pusherDone = st.pusher_pos <= st.TOL;
if pusherDone
    st.pusher_pos = 0;
end
targetIdx = findPackageIndex(st.current_target_id, st);
if targetIdx > 0
    st.platform_target_floor = st.pkg_floor(targetIdx);
    platformDone = platformAtFloor(st, st.pkg_floor(targetIdx));
else
    st.platform_target_floor = 1;
    platformDone = platformAtFloor(st, 1);
end
if pusherDone && platformDone
    st.b4_tof_barrier_hold_floor = 0;
    if targetIdx > 0
        st.mode = 6;
    else
        st.mode = 0;
    end
    st.last_message = "TEMP REINSERT READY";
elseif pusherDone
    st.last_message = "TEMP PLATFORM RETURNING";
else
    st.last_message = "TEMP PUSHER RETURNING";
end
end

function st = commitTempBufferedPackage(st)
id = st.temp_buffer_id;
floor = st.temp_reinsert_floor;
idx = id;
if idx <= 0 || idx > st.MAX_PKG
    idx = find(st.pkg_active <= 0.5 & st.pkg_id == 0, 1);
end
if isempty(idx) || idx <= 0
    return;
end
st.pkg_id(idx) = id;
st.pkg_floor(idx) = floor;
st.pkg_belt(idx) = 4;
st.pkg_pos(idx) = platformEntryPositionForB4(st.temp_buffer_long, st.temp_buffer_short);
st.pkg_long(idx) = st.temp_buffer_long;
st.pkg_short(idx) = st.temp_buffer_short;
st.pkg_height(idx) = st.temp_buffer_height;
st.pkg_active(idx) = 1;
st.pkg_aligned(idx) = 1;
targetBelt = 4;
st.pkg_target_belt(idx) = targetBelt;
st.pkg_route_phase(idx) = 0;
st.pkg_shift_remaining(idx) = -1;
st.pkg_close_after_shift(idx) = 0;
st.temp_reinsert_count = st.temp_reinsert_count + 1;
st.db_order_revision = st.db_order_revision + 1;
st = clearTempBufferState(st);
end

function st = clearTempBufferState(st)
st.temp_buffer_active = 0;
st.temp_buffer_id = 0;
st.temp_buffer_floor = 0;
st.temp_buffer_long = 0;
st.temp_buffer_short = 0;
st.temp_buffer_height = 0;
st.temp_buffer_original_floor = 0;
st.temp_buffer_original_belt = 0;
st.temp_buffer_original_pos = 0;
st.temp_reinsert_floor = 0;
st.temp_reinsert_target_belt = 0;
st.temp_unload_travel = 0;
st.temp_unload_remaining = 0;
st.temp_unload_restore_remaining = 0;
end

function idx = chooseTempBufferBlockerForTarget(targetIdx, st)
idx = 0;
if targetIdx <= 0 || st.temp_buffer_active > 0.5 || st.pusher_pos > st.TOL
    return;
end
floor = st.pkg_floor(targetIdx);
targetBelt = st.pkg_belt(targetIdx);
candidate = 0;
if targetBelt == 4 && targetHasB4BlockerAhead(targetIdx, st)
    candidate = topB4PackageAheadOfTarget(targetIdx, st);
elseif nextBelt(targetBelt) == 4 && topGap(floor, 4, st) < physicalCornerWidth() - st.TOL
    candidate = topPackageOnBelt(floor, 4, st);
elseif topGap(floor, 4, st) < physicalCornerWidth() - st.TOL
    candidate = topPackageOnBelt(floor, 4, st);
end
if candidate <= 0 || candidate == targetIdx
    return;
end
len = axisLengthForBelt(4, st.pkg_long(candidate), st.pkg_short(candidate));
front = st.pkg_pos(candidate) + len/2;
if front > tempBufferTravelLimit(st) + st.TOL
    return;
end
if ~tempUnloadKeepsOneBoxOnPlatform(candidate, front, st)
    return;
end
[slotFloor, ~] = chooseTempReinsertSlotForPackage(floor, ...
    st.pkg_long(candidate), st.pkg_short(candidate), st);
if slotFloor <= 0
    return;
end
idx = candidate;
end

function idx = topB4PackageAheadOfTarget(targetIdx, st)
idx = 0;
targetLen = axisLengthForBelt(4, st.pkg_long(targetIdx), st.pkg_short(targetIdx));
targetTail = st.pkg_pos(targetIdx) - targetLen/2;
bestTail = inf;
for i = 1:st.MAX_PKG
    if i == targetIdx || st.pkg_active(i) <= 0.5 || ...
            st.pkg_floor(i) ~= st.pkg_floor(targetIdx) || st.pkg_belt(i) ~= 4
        continue;
    end
    len = axisLengthForBelt(4, st.pkg_long(i), st.pkg_short(i));
    tail = st.pkg_pos(i) - len/2;
    if tail < targetTail - st.TOL && tail < bestTail
        bestTail = tail;
        idx = i;
    end
end
end

function flag = tempUnloadKeepsOneBoxOnPlatform(blockerIdx, travel, st)
flag = false;
if blockerIdx <= 0 || travel <= st.TOL
    return;
end
floor = st.pkg_floor(blockerIdx);
for i = 1:st.MAX_PKG
    if i == blockerIdx || st.pkg_active(i) <= 0.5 || ...
            st.pkg_floor(i) ~= floor || st.pkg_belt(i) ~= 4
        continue;
    end
    len = axisLengthForBelt(4, st.pkg_long(i), st.pkg_short(i));
    tail = st.pkg_pos(i) - len/2;
    if tail < travel - 1.0e-5
        return;
    end
end
flag = true;
end

function [floor, targetBelt] = chooseTempReinsertSlot(st)
[floor, targetBelt] = chooseTempReinsertSlotForPackage(st.temp_buffer_original_floor, ...
    st.temp_buffer_long, st.temp_buffer_short, st);
end

function [floor, targetBelt] = chooseTempReinsertSlotForPackage(originalFloor, longSide, shortSide, st)
floor = 0;
targetBelt = 0;
bestScore = -inf;
for f = 1:st.FLOOR_COUNT
    if f == originalFloor
        continue;
    end
    order = [2; 3; 1; 4];
    for q = 1:numel(order)
        b = order(q);
        if ~tempReinsertSlotCanAcceptPackage(f, b, originalFloor, longSide, shortSide, st)
            continue;
        end
        stageLen = axisLengthForBelt(4, longSide, shortSide);
        targetLen = axisLengthForBelt(b, longSide, shortSide);
        stageSpare = beltLength(4) - b4LoadUsed(f, st) - stageLen;
        targetSpare = loadStageResidualLength(b, reservedBeltLoadUsed(f, b, st), targetLen);
        possibleGap = topGap(f, 4, st) + noHandoffForwardMargin(f, 4, st);
        score = 1000 * double(b ~= 4) + 200 * min(possibleGap, physicalCornerWidth()) + ...
            10 * targetSpare + stageSpare - q;
        targetIdx = findPackageIndex(st.current_target_id, st);
        if targetIdx > 0 && st.pkg_floor(targetIdx) == f
            score = score - 10;
        end
        if score > bestScore
            bestScore = score;
            floor = f;
            targetBelt = b;
        end
    end
end
end

function flag = tempReinsertSlotCanAccept(floor, targetBelt, st)
flag = tempReinsertSlotCanAcceptPackage(floor, targetBelt, st.temp_buffer_original_floor, ...
    st.temp_buffer_long, st.temp_buffer_short, st);
end

function flag = tempReinsertSlotStillValid(floor, targetBelt, st)
flag = tempReinsertSlotBasicCanAcceptPackage(floor, targetBelt, st.temp_buffer_original_floor, ...
    st.temp_buffer_long, st.temp_buffer_short, st);
end

function flag = tempReinsertSlotCanAcceptPackage(floor, targetBelt, originalFloor, longSide, shortSide, st)
flag = false;
if ~tempReinsertSlotBasicCanAcceptPackage(floor, targetBelt, originalFloor, longSide, shortSide, st)
    return;
end
if ~canPrepareTempReinsertMeetGapPackage(floor, longSide, shortSide, st)
    return;
end
flag = true;
end

function flag = tempReinsertSlotBasicCanAcceptPackage(floor, targetBelt, originalFloor, longSide, shortSide, st)
flag = false;
if floor <= 0 || floor > st.FLOOR_COUNT || floor == originalFloor || targetBelt <= 0
    return;
end
stageLen = axisLengthForBelt(4, longSide, shortSide);
targetLen = axisLengthForBelt(targetBelt, longSide, shortSide);
if floorLoadUsed(floor, st) + stageLen > fullLoopLength() + st.TOL
    return;
end
if ~loadStageCanAcceptLength(targetBelt, reservedBeltLoadUsed(floor, targetBelt, st), targetLen, st)
    return;
end
if ~canAttemptTempReinsertMeetGapLength(floor, stageLen, st)
    return;
end
flag = true;
end

function [st, moved, ready] = prepareTempReinsertMeetGap(st, floor, needGap, step)
moved = 0;
ready = 0;
if floor <= 0 || needGap <= st.TOL
    return;
end
if ~canAttemptTempReinsertMeetGapLength(floor, needGap, st)
    return;
end
if beltPackageCount(floor, 4, st) <= 0
    ready = 1;
    return;
end

[outBelt, outD] = activeOutboundCompletionMove(floor, step, st);
if outD > 5.0e-5
    [st, outD] = applyForwardMoveNoOverlap(st, floor, outBelt, outD);
    if outD > 5.0e-5
        st = registerMotorMove(st, floor, outBelt, outD, 1);
        moved = 1;
        st.last_message = sprintf("TEMP REINSERT OUTBOUND F%d B%d P%d", ...
            floor, outBelt, st.temp_buffer_id);
    end
    return;
end

if beltHasTransferOverhang(floor, 4, st)
    return;
end

if b4LoadUsed(floor, st) + needGap > beltLength(4) + st.TOL
    [st, moved] = prepareTempReinsertB4StageCapacity(st, floor, step);
    return;
end

gapTol = max(2 * st.TOL, 0.00025);
gap = topGap(floor, 4, st);
if gap > needGap + gapTol
    [candidate, travel, ok] = compactBeltToTop(st, floor, 4);
    if ok && detectOverlap(candidate) <= 0.5
        st = candidate;
        if travel > st.TOL
            st = registerMotorMove(st, floor, 4, travel, -1);
        end
        moved = 1;
        st.last_message = sprintf("TEMP REINSERT MEET F%d P%d", floor, st.temp_buffer_id);
    end
    return;
end

if gap + gapTol < needGap
    d = min([step, needGap - gap, noHandoffForwardMargin(floor, 4, st)]);
    d = safeForwardMoveDistance(floor, 4, d, st, 0);
    if d > 5.0e-5
        st = moveBeltForward(st, floor, 4, d);
        st = registerMotorMove(st, floor, 4, d, 1);
        moved = 1;
        st.last_message = sprintf("TEMP REINSERT SLOT F%d P%d", floor, st.temp_buffer_id);
    end
    return;
end

ready = 1;
end

function [st, moved] = prepareTempReinsertB4StageCapacity(st, floor, step)
moved = 0;
if topGap(floor, 1, st) < physicalCornerWidth() - st.TOL
    [st, moved] = prepareTempB1ReceiverGap(st, floor, step);
    return;
end
if movingBeltWouldRotateInbound(floor, 4, st)
    return;
end
d = safeForwardMoveDistance(floor, 4, step, st, 1);
if d > 5.0e-5
    [st, d] = applyForwardMoveNoOverlap(st, floor, 4, d);
    if d > 5.0e-5
        st = registerMotorMove(st, floor, 4, d, 1);
        moved = 1;
        st.last_message = sprintf("TEMP REINSERT B4 FREE F%d P%d", floor, st.temp_buffer_id);
    end
end
end

function [st, moved] = prepareTempB1ReceiverGap(st, floor, step)
moved = 0;
if beltCanCompactToFullTopGap(floor, 1, st)
    [candidate, travel, ok] = compactBeltToBottom(st, floor, 1);
    if ok && detectOverlap(candidate) <= 0.5
        st = candidate;
        if travel > st.TOL
            st = registerMotorMove(st, floor, 1, travel, 1);
        end
        moved = 1;
        st.last_message = sprintf("TEMP REINSERT B1 GAP F%d P%d", floor, st.temp_buffer_id);
    end
    return;
end

[gapBelt, gapD] = chooseDownstreamGapCreationMove(floor, 1, 4, step, st);
if gapD <= st.TOL
    [gapBelt, gapD] = chooseGapConsolidationMove(floor, step, st);
end
if gapD > 5.0e-5
    [st, gapD] = applyForwardMoveNoOverlap(st, floor, gapBelt, gapD);
    if gapD > 5.0e-5
        st = registerMotorMove(st, floor, gapBelt, gapD, 1);
        moved = 1;
        st.last_message = sprintf("TEMP REINSERT B1 CHASE F%d B%d P%d", ...
            floor, gapBelt, st.temp_buffer_id);
    end
end
end

function flag = canPrepareTempReinsertMeetGapPackage(floor, longSide, shortSide, st)
needGap = axisLengthForBelt(4, longSide, shortSide);
flag = false;
if ~canAttemptTempReinsertMeetGapLength(floor, needGap, st)
    return;
end
if beltPackageCount(floor, 4, st) <= 0 || ...
        b4LoadUsed(floor, st) + needGap <= beltLength(4) + st.TOL
    flag = true;
    return;
end
outIdx = bottomPackageOnBelt(floor, 4, st);
if outIdx <= 0 || beltHasTransferOverhang(floor, 1, st)
    return;
end
outLenOnB1 = axisLengthForBelt(1, st.pkg_long(outIdx), st.pkg_short(outIdx));
b1Used = beltTotalAxisLength(floor, 1, st);
flag = b1Used <= beltLength(1) - physicalCornerWidth() + st.TOL && ...
    b1Used + outLenOnB1 <= beltLength(1) + st.TOL;
end

function flag = canAttemptTempReinsertMeetGapLength(floor, needGap, st)
flag = false;
if floor <= 0 || floor > st.FLOOR_COUNT || needGap <= st.TOL
    return;
end
if movingBeltWouldRotateInbound(floor, 4, st)
    return;
end
if beltPackageCount(floor, 4, st) <= 0
    flag = true;
    return;
end
flag = true;
end

function lim = tempBufferTravelLimit(st)
lim = st.PUSHER_TRAVEL;
end

function st = advanceSimpleCirculation(st, step)
st = clampCirculationBlockedOverhangs(st);
if st.circ_reverse_phase > 0.5
    st = clearReverseRedistribution(st);
end
if st.circ_compact_phase > 0.5
    st = advanceCirculationCompact(st);
    return;
end

idx = findPackageIndex(st.current_target_id, st);
if idx <= 0
    st.current_target_id = 0;
    st.mode = 0;
    return;
end

st = updateTargetProgressMemory(st, idx);

if st.temp_buffer_active > 0.5
    st.mode = 16;
    st = advanceTempReinsertPrep(st, step);
    return;
end

tfloor = st.pkg_floor(idx);
tbelt = st.pkg_belt(idx);
if targetReadyForB4Unload(idx, step, st)
    st = startTargetWaitAreaStaging(st, idx);
    return;
end

st.circ_b4_blocker_force_floor = 0;


[belt, d] = chooseUnloadTokenCirculationMove(tfloor, idx, step, st);
if d > 0
    [st, d] = applyForwardMoveNoOverlap(st, tfloor, belt, d);
    if d <= st.TOL
        blockerIdx = chooseTempBufferBlockerForTarget(idx, st);
        if blockerIdx > 0
            st = startTempBufferUnload(st, blockerIdx);
            return;
        end
        st.last_message = sprintf("CIRCULATION WAIT P%d", st.current_target_id);
        return;
    end
    st = registerMotorMove(st, tfloor, belt, d, 1);
    idxAfter = findPackageIndex(st.current_target_id, st);
    if idxAfter > 0
        st.circ_last_target_belt = st.pkg_belt(idxAfter);
    end
    st.last_message = sprintf("CIRCULATING P%d", st.current_target_id);
else
    [compactFloor, compactBelt] = chooseCirculationCompactBelt(tfloor, nextBelt(tbelt), st);
    if compactBelt > 0
        st.circ_compact_phase = 1;
        st.circ_compact_floor = compactFloor;
        st.circ_compact_belt = compactBelt;
        st = advanceCirculationCompact(st);
    else
        [compactFloor, compactBelt] = choosePreRefugeCompactBelt(tfloor, nextBelt(tbelt), st);
        if compactBelt > 0
            st.circ_compact_phase = 1;
            st.circ_compact_floor = compactFloor;
            st.circ_compact_belt = compactBelt;
            st = advanceCirculationCompact(st);
        else
            blockerIdx = chooseTempBufferBlockerForTarget(idx, st);
            if blockerIdx > 0
                st = startTempBufferUnload(st, blockerIdx);
            else
                st.last_message = sprintf("CIRCULATION WAIT P%d", st.current_target_id);
            end
        end
    end
end
end

function st = updateTargetProgressMemory(st, idx)
targetId = st.current_target_id;
coord = packageLoopCoord(idx, st);
if st.circ_progress_target_id ~= targetId
    st.circ_progress_target_id = targetId;
    st.circ_prev_coord = coord;
    st.circ_progress_pos = 0;
    st.circ_best_progress = 0;
    return;
end
delta = forwardLoopDelta(st.circ_prev_coord, coord);
st.circ_progress_pos = st.circ_progress_pos + delta;
st.circ_best_progress = max(st.circ_best_progress, st.circ_progress_pos);
st.circ_prev_coord = coord;
end

function st = noteReverseChoice(st, floor, sourceBelt)
if st.circ_last_reverse_floor == floor && st.circ_last_reverse_source_belt == sourceBelt
    st.circ_last_reverse_repeat = st.circ_last_reverse_repeat + 1;
else
    st.circ_last_reverse_floor = floor;
    st.circ_last_reverse_source_belt = sourceBelt;
    st.circ_last_reverse_repeat = 1;
end
end

function st = advanceCirculationCompact(st)
floor = st.circ_compact_floor;
belt = st.circ_compact_belt;
if floor <= 0 || belt <= 0
    st.circ_compact_phase = 0;
    return;
end

if st.circ_compact_phase == 1
    [candidate, travel, ok] = compactBeltToTop(st, floor, belt);
    if ok && detectOverlap(candidate) <= 0.5
        st = candidate;
        if travel > st.TOL
            st = registerMotorMove(st, floor, belt, travel, -1);
        end
        st.circ_compact_phase = 2;
        st.last_message = sprintf("CIRC COMPACT TOP F%d B%d", floor, belt);
    else
        st.circ_compact_phase = 0;
        st.last_message = sprintf("CIRC COMPACT BLOCKED F%d B%d", floor, belt);
    end
else
    [candidate, travel, ok] = compactBeltToBottom(st, floor, belt);
    if ok && detectOverlap(candidate) <= 0.5
        st = candidate;
        if travel > st.TOL
            st = registerMotorMove(st, floor, belt, travel, 1);
        end
        st.circ_compact_phase = 0;
        st.circ_compact_floor = 0;
        st.circ_compact_belt = 0;
        st.last_message = sprintf("CIRC COMPACT BOTTOM F%d B%d", floor, belt);
    else
        st.circ_compact_phase = 0;
        st.last_message = sprintf("CIRC COMPACT BLOCKED F%d B%d", floor, belt);
    end
end
end

function st = maybeStartSourceCompact(st, floor, belt)
% Circulation must not compact a belt merely because a local gap exists.
% Compact is only allowed when the current target's receiver gap is blocked
% and the selected receiver belt can actually recover the required corner
% gap. The receiver-scoped decision is made in chooseCirculationCompactBelt.
return;
end

function [floor, belt] = chooseCirculationCompactBelt(floorIn, neededGapBelt, st)
floor = floorIn;
belt = 0;
if circulationReceiverCompactNeeded(floorIn, neededGapBelt, st)
    belt = neededGapBelt;
end
end

function [floor, belt] = choosePreRefugeCompactBelt(floorIn, neededGapBelt, st)
floor = floorIn;
belt = 0;
if floor <= 0
    return;
end

if neededGapBelt >= 1 && neededGapBelt <= 4
    candidates = [4 neededGapBelt nextBelt(neededGapBelt) ...
        nextBelt(nextBelt(neededGapBelt)) nextBelt(nextBelt(nextBelt(neededGapBelt)))];
else
    candidates = [4 1 2 3];
end
seen = zeros(4, 1);
for k = 1:numel(candidates)
    b = round(candidates(k));
    if b < 1 || b > 4 || seen(b) > 0.5
        continue;
    end
    seen(b) = 1;
    if topGap(floor, b, st) >= physicalCornerWidth() - st.TOL
        continue;
    end
    if ~beltCanCompactToFullTopGap(floor, b, st)
        continue;
    end
    [candidate, ~, ok] = compactBeltToTop(st, floor, b);
    if ok
        [candidate, ~, ok] = compactBeltToBottom(candidate, floor, b);
    end
    if ok && detectOverlap(candidate) <= 0.5
        belt = b;
        return;
    end
end
end

function flag = circulationReceiverCompactNeeded(floor, belt, st)
flag = 0;
if floor <= 0 || belt <= 0
    return;
end
if topGap(floor, belt, st) >= physicalCornerWidth() - st.TOL
    return;
end
if beltHasTransferOverhang(floor, belt, st)
    return;
end
flag = beltCanCompactToFullTopGap(floor, belt, st);
end

function [floor, sourceBelt, receiverBelt] = chooseReverseRedistributionPlan(floorIn, neededGapBelt, targetBelt, st)
floor = floorIn;
sourceBelt = 0;
receiverBelt = 0;
% Disabled by the physical model: a conveyor can reverse only along itself,
% not transfer a parcel sideways through a corner to the previous belt.
return;
idx = findPackageIndex(st.current_target_id, st);
if idx <= 0
    return;
end
order = zeros(4,1);
if prevBelt(neededGapBelt) == targetBelt
    order(1) = nextBelt(neededGapBelt);
    order(2) = nextBelt(order(1));
    order(3) = nextBelt(order(2));
    order(4) = neededGapBelt;
else
    order(1) = neededGapBelt;
    for q = 2:4
        order(q) = nextBelt(order(q-1));
    end
end
bestScore = -inf;
for k = 1:4
    b = order(k);
    if b == targetBelt && targetBelt ~= 4
        continue;
    end
    if ~reverseRedistributionCanCreateGap(floorIn, b, st)
        continue;
    end
    candidate = simulateOneReverseRedistribution(st, floorIn, b);
    candidate = simulatePostReverseSourceCompact(candidate, floorIn, b);
    if detectOverlap(candidate) > 0.5
        continue;
    end
    score = scoreReverseRedistributionCandidate(st, candidate, floorIn, idx, neededGapBelt, targetBelt, b, k);
    if st.circ_last_reverse_floor == floorIn && st.circ_last_reverse_source_belt == b
        score = score - 5000 * min(st.circ_last_reverse_repeat, 5);
    end
    if score > bestScore
        bestScore = score;
        sourceBelt = b;
        receiverBelt = prevBelt(b);
    end
end
end

function candidate = simulatePostReverseSourceCompact(candidate, floor, sourceBelt)
if beltCanCompactToFullTopGap(floor, sourceBelt, candidate)
    [candidate, ~, ok] = compactBeltToTop(candidate, floor, sourceBelt);
    if ok
        [candidate, ~, ok] = compactBeltToBottom(candidate, floor, sourceBelt);
    end
    if ~ok
        return;
    end
end
end

function score = scoreReverseRedistributionCandidate(st, candidate, floor, idx, neededGapBelt, targetBelt, sourceBelt, orderRank)
score = -orderRank;
targetId = st.current_target_id;
idxCandidate = findPackageIndex(targetId, candidate);
if idxCandidate <= 0
    score = -inf;
    return;
end

baseCoord = packageLoopCoord(idx, st);
coord = packageLoopCoord(idxCandidate, candidate);
accum = max(0, forwardLoopDelta(baseCoord, coord));
bestReady = targetReadyForB4Unload(idxCandidate, st.BELT_SPEED * st.Ts, candidate);
if bestReady
    score = score + 100000;
end

local = candidate;
prevCoord = coord;
plannerStep = min(0.050, max(0.005, st.BELT_SPEED * st.Ts * 30));
for q = 1:30
    idxLocal = findPackageIndex(targetId, local);
    if idxLocal <= 0
        break;
    end
    if targetReadyForB4Unload(idxLocal, plannerStep, local)
        score = score + 100000 + 100 * q;
        break;
    end
    [belt, d] = chooseUnloadTokenCirculationMove(floor, idxLocal, plannerStep, local);
    if d <= local.TOL
        break;
    end
    [moved, d] = applyForwardMoveNoOverlap(local, floor, belt, d);
    if d <= local.TOL || detectOverlap(moved) > 0.5
        break;
    end
    local = moved;
    idxLocal = findPackageIndex(targetId, local);
    newCoord = packageLoopCoord(idxLocal, local);
    delta = forwardLoopDelta(prevCoord, newCoord);
    if delta > 0
        accum = accum + delta;
    end
    prevCoord = newCoord;
end

idxLocal = findPackageIndex(targetId, local);
if idxLocal > 0
    projectedProgress = st.circ_progress_pos + ...
        forwardLoopDelta(baseCoord, packageLoopCoord(idxLocal, local));
    backtrack = st.circ_best_progress - projectedProgress;
    if backtrack > 0.030
        score = score - 60000 - 100000 * backtrack;
    end
    if prevBelt(sourceBelt) == targetBelt && ...
            projectedProgress <= st.circ_best_progress + 0.010
        score = score - 45000;
    end
    score = score + 2000 * accum;
    score = score + 120 * min(topGap(floor, neededGapBelt, local), physicalCornerWidth());
    score = score + 30 * fullTopGapCount(floor, local);
    if sourceBelt == targetBelt && targetBelt ~= 4
        score = score - 10000;
    elseif local.pkg_belt(idxLocal) == sourceBelt
        score = score - 200;
    end
else
    score = -inf;
end
end

function count = fullTopGapCount(floor, st)
count = 0;
for b = 1:4
    if topGap(floor, b, st) >= physicalCornerWidth() - st.TOL
        count = count + 1;
    end
end
end

function [floor, sourceBelt, receiverBelt] = chooseB4BlockerReversePlan(idx, st)
floor = 0;
sourceBelt = 0;
receiverBelt = 0;
return;
if idx <= 0 || st.pkg_belt(idx) ~= 4
    return;
end
if ~targetHasB4BlockerAhead(idx, st)
    return;
end
floor = st.pkg_floor(idx);
if reverseRedistributionCanCreateGap(floor, 4, st)
    sourceBelt = 4;
    receiverBelt = 3;
else
    [sourceBelt, receiverBelt] = chooseB4BlockerReceiverRoomPlan(floor, st);
    if sourceBelt <= 0
        floor = 0;
    end
end
end

function [sourceBelt, receiverBelt] = chooseB4BlockerReceiverRoomPlan(floor, st)
sourceBelt = 0;
receiverBelt = 0;

if reverseRedistributionCanCreateGap(floor, 3, st)
    candidate = simulateOneReverseRedistribution(st, floor, 3);
    if reverseRedistributionCanCreateGap(floor, 4, candidate)
        sourceBelt = 3;
        receiverBelt = 2;
        return;
    end
end

if reverseRedistributionCanCreateGap(floor, 2, st)
    candidate = simulateOneReverseRedistribution(st, floor, 2);
    if reverseRedistributionCanCreateGap(floor, 3, candidate)
        sourceBelt = 2;
        receiverBelt = 1;
    end
end
end

function candidate = simulateOneReverseRedistribution(st, floor, sourceBelt)
receiverBelt = prevBelt(sourceBelt);
idx = topPackageOnBelt(floor, sourceBelt, st);
if idx <= 0
    candidate = st;
    return;
end
candidate = compactReceiverAndReverseOne(st, floor, sourceBelt, receiverBelt, idx);
end

function flag = targetHasB4BlockerAhead(idx, st)
flag = 0;
targetLen = axisLengthForBelt(4, st.pkg_long(idx), st.pkg_short(idx));
targetTail = st.pkg_pos(idx) - targetLen/2;
for i = 1:st.MAX_PKG
    if i ~= idx && st.pkg_active(i) > 0.5 && ...
            st.pkg_floor(i) == st.pkg_floor(idx) && st.pkg_belt(i) == 4
        len = axisLengthForBelt(4, st.pkg_long(i), st.pkg_short(i));
        tail = st.pkg_pos(i) - len/2;
        if tail < targetTail - st.TOL
            flag = 1;
            return;
        end
    end
end
end

function flag = reverseRedistributionCanCreateGap(floor, sourceBelt, st)
flag = 0;
return;
idx = topPackageOnBelt(floor, sourceBelt, st);
if idx <= 0
    return;
end
receiver = prevBelt(sourceBelt);
if beltHasTransferOverhang(floor, sourceBelt, st) || beltHasTransferOverhang(floor, receiver, st)
    return;
end
sourceLen = axisLengthForBelt(sourceBelt, st.pkg_long(idx), st.pkg_short(idx));
receiverLen = axisLengthForBelt(receiver, st.pkg_long(idx), st.pkg_short(idx));
sourceAfter = beltTotalAxisLength(floor, sourceBelt, st) - sourceLen;
receiverAfter = beltTotalAxisLength(floor, receiver, st) + receiverLen;
if sourceAfter > beltLength(sourceBelt) - physicalCornerWidth() + st.TOL
    return;
end
if receiverAfter > beltLength(receiver) + st.TOL
    return;
end
candidate = compactReceiverAndReverseOne(st, floor, sourceBelt, receiver, idx);
flag = detectOverlap(candidate) <= 0.5;
end

function stOut = compactReceiverAndReverseOne(st, floor, sourceBelt, receiver, idx)
stOut = st;
end

function flag = beltCanCompactToFullTopGap(floor, belt, st)
flag = 0;
if beltPackageCount(floor, belt, st) <= 0
    return;
end
if beltHasTransferOverhang(floor, belt, st)
    return;
end
flag = beltTotalAxisLength(floor, belt, st) <= beltLength(belt) - physicalCornerWidth() + st.TOL;
end

function n = beltPackageCount(floor, belt, st)
n = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        n = n + 1;
    end
end
end

function idx = topPackageOnBelt(floor, belt, st)
idx = 0;
bestTail = inf;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        tail = st.pkg_pos(i) - len/2;
        if tail < bestTail
            bestTail = tail;
            idx = i;
        end
    end
end
end

function idx = bottomPackageOnBelt(floor, belt, st)
idx = 0;
bestFront = -inf;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        front = st.pkg_pos(i) + len/2;
        if front > bestFront
            bestFront = front;
            idx = i;
        end
    end
end
end

function total = beltTotalAxisLength(floor, belt, st)
total = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        total = total + axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
    end
end
end

function flag = beltHasTransferOverhang(floor, belt, st)
flag = movingBeltWouldRotateInbound(floor, belt, st);
if flag
    return;
end
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        tail = st.pkg_pos(i) - len/2;
        front = st.pkg_pos(i) + len/2;
        if tail < -st.TOL || front > beltLength(belt) + st.TOL
            flag = 1;
            return;
        end
    end
end
end

function [stOut, travel, ok] = compactBeltToTop(st, floor, belt)
stOut = st;
travel = 0;
ok = 0;
[idxs, n] = beltPackageIndices(floor, belt, st);
if n <= 0 || beltHasTransferOverhang(floor, belt, st)
    return;
end
idxs = sortBeltIndicesByPosition(idxs, n, st);
cursor = 0;
for k = 1:n
    i = idxs(k);
    len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
    desired = cursor + len/2;
    stOut.pkg_pos(i) = desired;
    cursor = cursor + len;
end
ok = cursor <= beltLength(belt) + st.TOL;
if ok
    travel = guaranteedCompactTravel(floor, belt, st);
end
end

function [stOut, travel, ok] = compactBeltToBottom(st, floor, belt)
stOut = st;
travel = 0;
ok = 0;
[idxs, n] = beltPackageIndices(floor, belt, st);
if n <= 0 || beltHasTransferOverhang(floor, belt, st)
    return;
end
idxs = sortBeltIndicesByPosition(idxs, n, st);
total = beltTotalAxisLength(floor, belt, st);
cursor = beltLength(belt) - total;
if cursor < -st.TOL
    return;
end
for k = 1:n
    i = idxs(k);
    len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
    desired = cursor + len/2;
    stOut.pkg_pos(i) = desired;
    cursor = cursor + len;
end
ok = cursor <= beltLength(belt) + st.TOL;
if ok
    travel = guaranteedCompactTravel(floor, belt, st);
end
end

function travel = guaranteedCompactTravel(floor, belt, st)
travel = max(0, beltLength(belt) - beltTotalAxisLength(floor, belt, st));
end

function [idxs, n] = beltPackageIndices(floor, belt, st)
idxs = zeros(st.MAX_PKG,1);
n = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        n = n + 1;
        idxs(n) = i;
    end
end
idxs = idxs(1:n);
end

function idxs = sortBeltIndicesByPosition(idxs, n, st)
positions = zeros(n,1);
for k = 1:n
    positions(k) = st.pkg_pos(idxs(k));
end
[~, order] = sort(positions);
idxs = idxs(order);
end

function [belt, d] = chooseUnloadTokenCirculationMove(floor, idx, step, st)
belt = 0;
d = 0;

[outBelt, outD] = realOutboundCompletionMove(floor, step, st);
if outD > 5.0e-5
    belt = outBelt;
    d = outD;
    return;
end

targetBelt = st.pkg_belt(idx);
neededGapBelt = nextBelt(targetBelt);

[belt, d] = chooseTargetForwardLapMove(floor, idx, step, st);
if d > 5.0e-5
    return;
end

[belt, d] = chooseB4BlockerGapSeedMove(floor, idx, step, st);
if d > 5.0e-5
    return;
end

[belt, d] = chooseTargetBeltGapConsolidation(floor, step, st);
if d > 5.0e-5
    return;
end

[belt, d] = chooseFullGapChaseMove(floor, neededGapBelt, step, st);
if d > 5.0e-5
    return;
end

[belt, d] = chooseDownstreamGapCreationMove(floor, neededGapBelt, targetBelt, step, st);
if d > 5.0e-5
    return;
end

[belt, d] = chooseGapConsolidationMove(floor, step, st);
if d > 5.0e-5
    return;
end

[belt, d] = chooseGreedySafeLapMove(floor, step, st);
if d > 5.0e-5
    return;
end

belt = 0;
d = 0;
end

function [belt, d] = chooseB4BlockerGapSeedMove(floor, idx, step, st)
belt = 0;
d = 0;
if idx <= 0 || st.pkg_belt(idx) ~= 4 || ~targetHasB4BlockerAhead(idx, st)
    return;
end
gap = topGap(floor, 4, st);
if gap >= physicalCornerWidth() - st.TOL
    return;
end
if movingBeltWouldRotateInbound(floor, 4, st)
    return;
end
need = physicalCornerWidth() - gap;
candidateD = min([step, need, noHandoffForwardMargin(floor, 4, st)]);
candidateD = safeForwardMoveDistance(floor, 4, candidateD, st, 0);
if candidateD > st.TOL
    belt = 4;
    d = candidateD;
end
end

function [belt, d] = realOutboundCompletionMove(floor, step, st)
belt = 0;
d = 0;
bestNeed = inf;
overhangTol = st.TOL;
for b = 1:4
    for i = 1:st.MAX_PKG
        if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == b
            len = axisLengthForBelt(b, st.pkg_long(i), st.pkg_short(i));
            tail = st.pkg_pos(i) - len/2;
            front = st.pkg_pos(i) + len/2;
            overhang = front - beltLength(b);
            if overhang > overhangTol && tail < beltLength(b) - st.TOL && ...
                    topGap(floor, nextBelt(b), st) >= physicalCornerWidth() - st.TOL
                need = beltLength(b) - tail + st.TOL;
                if need < bestNeed && ~movingBeltWouldRotateInbound(floor, b, st)
                    bestNeed = need;
                    belt = b;
                    d = min(step, need);
                end
            end
        end
    end
end
end

function [belt, dist] = chooseSimpleSafeCirculationMove(floor, desiredBelt, step, st)
order = zeros(4,1);
order(1) = desiredBelt;
for i = 2:4
    order(i) = nextBelt(order(i-1));
end
for i = 1:4
    b = order(i);
    if movingBeltWouldRotateInbound(floor, b, st)
        continue;
    end
    d = safeForwardMoveDistance(floor, b, step, st, 1);
    if d > 5.0e-5
        belt = b;
        dist = d;
        return;
    end
end
belt = desiredBelt;
dist = 0;
end

function flag = platformIsFree(st)
flag = st.mode == 0 && st.temp_buffer_active <= 0.5 && ...
    platformAtFloor(st, 1) && st.pusher_pos <= st.TOL;
end

function st = clearLoadPrepareState(st)
st.load_prepare_required = 0;
st.load_prepare_steps = 0;
end

function st = clearCurrentLoadForFinalize(st)
st.load_id = 0;
st.load_floor = 0;
st.load_belt = 0;
st.load_long = 0;
st.load_short = 0;
st.load_height = 0;
st.load_yaw = 0;
st.load_yaw0 = 0;
st.load_from_platform_contact = 0;
st = clearLoadPrepareState(st);
end

function [st, floor, belt] = chooseLoadTargetForPackage(preferredFloor, longSide, shortSide, st)
floor = 0;
belt = 0;

[st, floor, belt] = chooseLookaheadLoadTarget(preferredFloor, longSide, shortSide, st);
if belt > 0
    return;
end

[floor, belt, stageIdx] = chooseBestFitPreB4LoadTarget(preferredFloor, longSide, shortSide, st);
if belt > 0
    st.load_stage_idx(floor) = stageIdx;
    return;
end

[floor, belt, stageIdx] = chooseBestFitB4LoadTarget(preferredFloor, longSide, shortSide, st);
if belt > 0
    st.load_stage_idx(floor) = stageIdx;
    return;
end

[floor, belt, stageIdx] = chooseResidualPreB4BackfillTarget(preferredFloor, longSide, shortSide, st);
if belt > 0
    st.load_stage_idx(floor) = stageIdx;
    st.load_prepare_required = 1;
    st.load_prepare_steps = 0;
end
end

function [st, floor, belt] = chooseLookaheadLoadTarget(preferredFloor, longSide, shortSide, st)
floor = 0;
belt = 0;
bestScore = inf;
bestCandidate = emptyLoadCandidate();

% Keep B4's 250 mm circulation reserve as the last loading region. Normal
% pre-B4 loading remains first; only after it is exhausted do we try
% reachable pre-B4 residual backfill before falling back to B4.
tierGroups = {1, 2, 3};
for groupIdx = 1:numel(tierGroups)
    bestScore = inf;
    bestCandidate = emptyLoadCandidate();
    floor = 0;
    belt = 0;
    candidates = repmat(emptyLoadCandidate(), 0, 1);
    tiers = tierGroups{groupIdx};
    for ti = 1:numel(tiers)
        tierCandidates = collectLookaheadLoadCandidates( ...
            preferredFloor, longSide, shortSide, st, tiers(ti));
        candidates = [candidates; tierCandidates(:)]; %#ok<AGROW>
    end
    if isempty(candidates)
        continue;
    end
    for k = 1:numel(candidates)
        cand = candidates(k);
        [after, ok, moveCost] = simulateVirtualLoadCandidate(st, cand, longSide, shortSide, st.load_height);
        if ~ok
            continue;
        end
        score = scoreLookaheadCandidate(after, cand, preferredFloor, moveCost);
        if score < bestScore
            bestScore = score;
            bestCandidate = cand;
            floor = cand.floor;
            belt = cand.belt;
        end
    end
    if belt > 0
        st.load_stage_idx(floor) = bestCandidate.stageIdx;
        st.load_prepare_required = double(bestCandidate.prepareRequired);
        st.load_prepare_steps = 0;
        return;
    end
end
end

function candidates = collectLookaheadLoadCandidates(preferredFloor, longSide, shortSide, st, tier)
candidates = repmat(emptyLoadCandidate(), 0, 1);
if tier == 1
    for offset = 0:st.FLOOR_COUNT-1
        f = 1 + mod(preferredFloor - 1 + offset, st.FLOOR_COUNT);
        [candidateBelt, candidateStage] = firstFittingLoadStage(f, longSide, shortSide, st);
        if candidateBelt <= 0 || candidateBelt == 4
            continue;
        end
        newLen = axisLengthForBelt(candidateBelt, longSide, shortSide);
        used = reservedBeltLoadUsed(f, candidateBelt, st);
        if ~loadStageCanAcceptLength(candidateBelt, used, newLen, st)
            continue;
        end
        spare = loadStageResidualLength(candidateBelt, used, newLen);
        candidates(end+1) = makeLoadCandidate(f, candidateBelt, candidateStage, 0, spare, tier); %#ok<AGROW>
    end
elseif tier == 2
    order = loadOrder();
    for offset = 0:st.FLOOR_COUNT-1
        f = 1 + mod(preferredFloor - 1 + offset, st.FLOOR_COUNT);
        for oi = 1:numel(order)
            b = order(oi);
            if b == 4
                continue;
            end
            newLen = axisLengthForBelt(b, longSide, shortSide);
            used = reservedBeltLoadUsed(f, b, st);
            if ~loadStageCanAcceptLength(b, used, newLen, st)
                continue;
            end
            spare = loadStageResidualLength(b, used, newLen);
            if spare < -st.TOL
                continue;
            end
            [canPrepare, ~] = canPrepareBackfillRouteGaps(f, b, st);
            if canPrepare
                prepareRequired = double(~preBackfillRouteGapsReady(f, b, st));
                candidates(end+1) = makeLoadCandidate(f, b, oi, prepareRequired, spare, tier); %#ok<AGROW>
            end
        end
    end
else
    for offset = 0:st.FLOOR_COUNT-1
        f = 1 + mod(preferredFloor - 1 + offset, st.FLOOR_COUNT);
        [candidateBelt, candidateStage] = firstFittingLoadStage(f, longSide, shortSide, st);
        if candidateBelt ~= 4
            continue;
        end
        newLen = axisLengthForBelt(4, longSide, shortSide);
        used = reservedBeltLoadUsed(f, 4, st);
        if ~loadStageCanAcceptLength(4, used, newLen, st)
            continue;
        end
        spare = loadStageResidualLength(4, used, newLen);
        candidates(end+1) = makeLoadCandidate(f, 4, candidateStage, 0, spare, tier); %#ok<AGROW>
    end
end
end

function cand = emptyLoadCandidate()
cand = struct('floor', 0, 'belt', 0, 'stageIdx', 0, ...
    'prepareRequired', 0, 'spare', inf, 'tier', 0);
end

function cand = makeLoadCandidate(floor, belt, stageIdx, prepareRequired, spare, tier)
cand = emptyLoadCandidate();
cand.floor = floor;
cand.belt = belt;
cand.stageIdx = stageIdx;
cand.prepareRequired = prepareRequired;
cand.spare = spare;
cand.tier = tier;
end

function [after, ok, moveCost] = simulateVirtualLoadCandidate(st, cand, longSide, shortSide, height)
after = clearStepMotorCommands(st);
ok = false;
moveCost = 0;
if cand.floor <= 0 || cand.belt <= 0 || st.next_id > st.MAX_PKG
    return;
end

after.load_id = after.next_id;
after.load_floor = cand.floor;
after.load_belt = cand.belt;
after.load_long = longSide;
after.load_short = shortSide;
after.load_height = height;
after.load_shift_remaining = -1;
after.load_close_after_shift = 0;
after.load_prepare_required = double(cand.prepareRequired);
after.load_prepare_steps = 0;
after.load_stage_idx(cand.floor) = cand.stageIdx;

plannerStep = min(0.080, max(0.008, after.BELT_SPEED * after.Ts * 80));
if cand.prepareRequired > 0.5
    for iter = 1:160
        if preBackfillRouteGapsReady(cand.floor, cand.belt, after)
            break;
        end
        before = clearStepMotorCommands(after);
        [candidate, moved] = advancePreBackfillGapPrepareAction(before, cand.floor, cand.belt, plannerStep);
        if ~moved || detectOverlap(candidate) > 0.5 || detectRotationRisk(candidate) > 0.5
            after = candidate;
            return;
        end
        moveCost = moveCost + virtualMoveCost(before, candidate);
        after = candidate;
    end
    if ~preBackfillRouteGapsReady(cand.floor, cand.belt, after)
        return;
    end
end

after.mode = 3;
for iter = 1:160
    before = clearStepMotorCommands(after);
    candidate = advanceB4TopClearance(before, plannerStep);
    if detectOverlap(candidate) > 0.5 || detectRotationRisk(candidate) > 0.5
        after = candidate;
        return;
    end
    moveCost = moveCost + virtualMoveCost(before, candidate);
    after = candidate;
    if after.mode == 4
        break;
    end
    if virtualStatesSame(before, after)
        return;
    end
end
if after.mode ~= 4
    return;
end

after = clearStepMotorCommands(after);
after = commitPushedPackage(after);
if detectOverlap(after) > 0.5 || detectRotationRisk(after) > 0.5
    return;
end
after.mode = 0;
[after, settled, routeCost] = settleVirtualLoadRoutes(after, plannerStep, 420);
moveCost = moveCost + routeCost;
if ~settled || detectOverlap(after) > 0.5
    return;
end
after = clearCurrentLoadForFinalize(after);
after.mode = 0;
after.last_message = "VIRTUAL LOAD SETTLED";
ok = true;
end

function [stOut, settled, moveCost] = settleVirtualLoadRoutes(st, step, maxIter)
stOut = st;
settled = false;
moveCost = 0;
for iter = 1:maxIter
    if ~hasPendingLoadRoute(stOut)
        settled = true;
        return;
    end
    before = clearStepMotorCommands(stOut);
    candidate = advanceBackgroundLoadRoutes(before, step);
    if detectOverlap(candidate) > 0.5 || detectRotationRisk(candidate) > 0.5
        stOut = candidate;
        return;
    end
    if virtualStatesSame(before, candidate)
        stOut = candidate;
        return;
    end
    moveCost = moveCost + virtualMoveCost(before, candidate);
    stOut = candidate;
end
settled = ~hasPendingLoadRoute(stOut);
end

function score = scoreLookaheadCandidate(after, cand, preferredFloor, moveCost)
preferredPenalty = double(cand.floor ~= preferredFloor) * 0.050;
orderPenalty = cand.stageIdx * 5.0e-4;
b4Penalty = double(cand.belt == 4) * 5.0e-4;
floorBalance = floorBalanceCandidatePenalty(cand.floor, after) * 0.080;
preparePenalty = double(cand.prepareRequired > 0.5) * 0.001;
score = max(0, cand.spare) + preferredPenalty + orderPenalty + ...
    b4Penalty + floorBalance + preparePenalty;
score = score + 0.005 * layoutPackingScore(after);
score = score - 0.030 * genericFutureFitScore(after);
score = score + 0.00005 * moveCost;
end

function penalty = floorBalanceCandidatePenalty(floor, st)
fractions = zeros(st.FLOOR_COUNT, 1);
for f = 1:st.FLOOR_COUNT
    fractions(f) = floorLoadFraction(f, st);
end
meanLoad = mean(fractions);
minLoad = min(fractions);
thisLoad = fractions(floor);
penalty = max(0, thisLoad - minLoad) + 0.5 * max(0, thisLoad - meanLoad);
end

function score = genericFutureFitScore(st)
score = 0;
for f = 1:st.FLOOR_COUNT
    for b = 1:4
        spare = futureLoadSpareLength(b, reservedBeltLoadUsed(f, b, st), st);
        if spare <= st.TOL
            continue;
        end
        score = score + standardBoxFitValue(b, spare);
    end
end
end

function value = standardBoxFitValue(belt, spare)
longTypes = [0.110, 0.135, 0.170, 0.205, 0.240];
shortTypes = [0.095, 0.090, 0.125, 0.155, 0.190];
value = 0;
for k = 1:numel(longTypes)
    if belt == 1 || belt == 3
        len = longTypes(k);
    else
        len = shortTypes(k);
    end
    if spare + 1.0e-9 >= len
        value = value + len;
    end
end
end

function score = layoutPackingScore(st)
score = 0;
for f = 1:st.FLOOR_COUNT
    for b = 1:4
        spare = max(0, futureLoadSpareLength(b, reservedBeltLoadUsed(f, b, st), st));
        score = score + spare * spare;
        score = score + 40 * blockedReceiverGapPenalty(f, b, st);
    end
end
for f = 1:st.FLOOR_COUNT
    score = score + 0.1 * max(0, physicalCornerWidth() - topGap(f, 4, st));
end
end

function spare = futureLoadSpareLength(belt, used, st)
if belt == 4
    spare = b4UsableLoadLength() - used;
    return;
end
if used <= beltLength(belt) - physicalCornerWidth() + st.TOL
    spare = beltLength(belt) - used;
else
    spare = 0;
end
end

function penalty = blockedReceiverGapPenalty(floor, belt, st)
gapDeficit = max(0, physicalCornerWidth() - topGap(floor, belt, st));
if gapDeficit <= st.TOL
    penalty = 0;
    return;
end
compactLimit = beltLength(belt) - physicalCornerWidth();
overLimit = max(0, beltTotalAxisLength(floor, belt, st) - compactLimit);
penalty = gapDeficit * overLimit;
if belt == 1
    penalty = penalty * 2.5;
elseif belt == 2
    penalty = penalty * 1.5;
end
end

function used = totalActiveAxisUsed(st)
used = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) > 0 && st.pkg_belt(i) > 0
        used = used + axisLengthForBelt(st.pkg_belt(i), st.pkg_long(i), st.pkg_short(i));
    end
end
end

function cost = virtualMoveCost(before, after)
delta = abs(after.pkg_pos - before.pkg_pos);
cost = sum(delta(before.pkg_active > 0.5 | after.pkg_active > 0.5));
end

function flag = virtualStatesSame(before, after)
flag = ~any(abs(after.pkg_pos - before.pkg_pos) > 1.0e-9) && ...
    ~any(after.pkg_belt ~= before.pkg_belt) && ...
    ~any(after.pkg_route_phase ~= before.pkg_route_phase) && ...
    ~any(abs(after.pkg_shift_remaining - before.pkg_shift_remaining) > 1.0e-9) && ...
    ~any(after.pkg_close_after_shift ~= before.pkg_close_after_shift) && ...
    ~any(after.load_stage_idx ~= before.load_stage_idx) && ...
    after.total_loaded == before.total_loaded && ...
    after.next_id == before.next_id && ...
    after.mode == before.mode;
end

function [floor, belt, stageIdx] = chooseBestFitPreB4LoadTarget(preferredFloor, longSide, shortSide, st)
floor = 0;
belt = 0;
stageIdx = 0;
bestScore = inf;
for offset = 0:st.FLOOR_COUNT-1
    f = 1 + mod(preferredFloor - 1 + offset, st.FLOOR_COUNT);
    [candidateBelt, candidateStage] = firstFittingLoadStage(f, longSide, shortSide, st);
    if candidateBelt <= 0 || candidateBelt == 4
        continue;
    end
    newLen = axisLengthForBelt(candidateBelt, longSide, shortSide);
    used = reservedBeltLoadUsed(f, candidateBelt, st);
    if ~loadStageCanAcceptLength(candidateBelt, used, newLen, st)
        continue;
    end
    spare = loadStageResidualLength(candidateBelt, used, newLen);
    score = loadBestFitScore(spare, f, candidateBelt, preferredFloor, candidateStage, st);
    if score < bestScore
        bestScore = score;
        floor = f;
        belt = candidateBelt;
        stageIdx = candidateStage;
    end
end
end

function [floor, belt, stageIdx] = chooseBestFitB4LoadTarget(preferredFloor, longSide, shortSide, st)
floor = 0;
belt = 0;
stageIdx = 0;
bestScore = inf;
for offset = 0:st.FLOOR_COUNT-1
    f = 1 + mod(preferredFloor - 1 + offset, st.FLOOR_COUNT);
    [candidateBelt, candidateStage] = firstFittingLoadStage(f, longSide, shortSide, st);
    if candidateBelt ~= 4
        continue;
    end
    newLen = axisLengthForBelt(4, longSide, shortSide);
    used = reservedBeltLoadUsed(f, 4, st);
    if ~loadStageCanAcceptLength(4, used, newLen, st)
        continue;
    end
    spare = loadStageResidualLength(4, used, newLen);
    score = loadBestFitScore(spare, f, 4, preferredFloor, candidateStage, st);
    if score < bestScore
        bestScore = score;
        floor = f;
        belt = 4;
        stageIdx = candidateStage;
    end
end
end

function [floor, belt, stageIdx] = chooseResidualPreB4BackfillTarget(preferredFloor, longSide, shortSide, st)
floor = 0;
belt = 0;
stageIdx = 0;
bestScore = inf;
order = loadOrder();
for offset = 0:st.FLOOR_COUNT-1
    f = 1 + mod(preferredFloor - 1 + offset, st.FLOOR_COUNT);
    for oi = 1:numel(order)
        b = order(oi);
        if b == 4
            continue;
        end
        newLen = axisLengthForBelt(b, longSide, shortSide);
        used = reservedBeltLoadUsed(f, b, st);
        if ~loadStageCanAcceptLength(b, used, newLen, st)
            continue;
        end
        spare = loadStageResidualLength(b, used, newLen);
        if spare < -st.TOL
            continue;
        end
        [canPrepare, prepareScore] = canPrepareBackfillRouteGaps(f, b, st);
        if ~canPrepare
            continue;
        end
        score = loadBestFitScore(spare, f, b, preferredFloor, oi, st) - 0.001 - prepareScore;
        if score < bestScore
            bestScore = score;
            floor = f;
            belt = b;
            stageIdx = st.load_stage_idx(f);
        end
    end
end
end

function flag = b4TopGapCanBeMadeWithoutHandoff(floor, st)
need = max(0, physicalCornerWidth() - topGap(floor, 4, st));
flag = noHandoffForwardMargin(floor, 4, st) + st.TOL >= need;
end

function flag = residualBackfillRouteHasFullGaps(floor, targetBelt, st)
flag = false;
b = 1;
while true
    if topGap(floor, b, st) < physicalCornerWidth() - st.TOL
        return;
    end
    if b == targetBelt
        flag = true;
        return;
    end
    b = nextBelt(b);
    if b == 4
        return;
    end
end
end

function [ok, prepareScore] = canPrepareBackfillRouteGaps(floor, targetBelt, st)
ok = false;
prepareScore = 0;
if targetBelt <= 0 || targetBelt == 4
    return;
end

local = clearStepMotorCommands(st);
plannerStep = min(0.060, max(0.006, st.BELT_SPEED * st.Ts * 60));
startScore = preBackfillRouteGapScore(floor, targetBelt, local);
for iter = 1:180
    if preBackfillRouteGapsReady(floor, targetBelt, local)
        ok = true;
        prepareScore = min(0.020, 0.00005 * iter) + ...
            0.0001 * preBackfillRouteGapScore(floor, targetBelt, local);
        return;
    end
    local = clearStepMotorCommands(local);
    [candidate, moved] = advancePreBackfillGapPrepareAction(local, floor, targetBelt, plannerStep);
    if ~moved || detectOverlap(candidate) > 0.5
        break;
    end
    if abs(preBackfillRouteGapScore(floor, targetBelt, candidate) - ...
            preBackfillRouteGapScore(floor, targetBelt, local)) <= 1.0e-8 && ...
            ~any(abs(candidate.pkg_pos - local.pkg_pos) > 1.0e-9)
        break;
    end
    local = candidate;
end

if preBackfillRouteGapsReady(floor, targetBelt, local) && detectOverlap(local) <= 0.5
    ok = true;
end
if ok
    finalScore = preBackfillRouteGapScore(floor, targetBelt, local);
    prepareScore = max(0, min(0.020, (finalScore - startScore) * 0.001));
end
end

function st = clearStepMotorCommands(st)
st.active_floor = 0;
st.active_belt = 0;
st.motor_dir = 0;
st.encoder_delta = 0;
st.motor_cmd = zeros(st.FLOOR_COUNT * 4,1);
st.encoder_delta_vec = zeros(st.FLOOR_COUNT * 4,1);
end

function flag = preBackfillRouteGapsReady(floor, targetBelt, st)
flag = false;
if movingBeltWouldRotateInbound(floor, 4, st)
    return;
end
gaps = preBackfillGapBelts(targetBelt);
for k = 1:numel(gaps)
    if topGap(floor, gaps(k), st) < physicalCornerWidth() - st.TOL
        return;
    end
end
flag = true;
end

function belt = preBackfillNeededGapBelt(floor, targetBelt, st)
belt = 0;
gaps = preBackfillGapBelts(targetBelt);
bestGap = inf;
for k = 1:numel(gaps)
    b = gaps(k);
    gap = topGap(floor, b, st);
    if gap < physicalCornerWidth() - st.TOL && gap < bestGap
        bestGap = gap;
        belt = b;
    end
end
if belt <= 0
    return;
end

% If a partial gap can be completed safely, prefer that belt over the
% smallest-gap belt. This helps convert near-ready receiver gaps first.
bestNeed = inf;
for k = 1:numel(gaps)
    b = gaps(k);
    gap = topGap(floor, b, st);
    need = physicalCornerWidth() - gap;
    if need <= st.TOL
        continue;
    end
    if noHandoffForwardMargin(floor, b, st) > st.TOL || beltCanCompactToFullTopGap(floor, b, st)
        if need < bestNeed
            bestNeed = need;
            belt = b;
        end
    end
end
end

function gaps = preBackfillGapBelts(targetBelt)
if targetBelt == 4
    gaps = 4;
    return;
end
gaps = zeros(4,1);
n = 1;
gaps(n) = 4;
b = 1;
while n < 4
    n = n + 1;
    gaps(n) = b;
    if b == targetBelt
        break;
    end
    b = nextBelt(b);
    if b == 4
        break;
    end
end
gaps = gaps(1:n);
end

function score = preBackfillRouteGapScore(floor, targetBelt, st)
score = 0;
gaps = preBackfillGapBelts(targetBelt);
for k = 1:numel(gaps)
    score = score + min(topGap(floor, gaps(k), st), physicalCornerWidth());
end
score = score + 0.01 * fullTopGapCount(floor, st);
end

function [belt, stageIdx] = firstFittingLoadStage(floor, longSide, shortSide, st)
order = loadOrder();
belt = 0;
stageIdx = 0;
idx = st.load_stage_idx(floor);
while idx <= numel(order)
    b = order(idx);
    newLen = axisLengthForBelt(b, longSide, shortSide);
    if b == 4
        if b4TopLoadCanFit(floor, newLen, st)
            belt = b;
            stageIdx = idx;
        end
        return;
    end
    if loadStageCanAcceptLength(b, reservedBeltLoadUsed(floor, b, st), newLen, st)
        belt = b;
        stageIdx = idx;
        return;
    end
    idx = idx + 1;
end
end

function score = loadBestFitScore(spare, floor, belt, preferredFloor, orderIndex, st)
preferredPenalty = double(floor ~= preferredFloor) * 0.050;
orderPenalty = orderIndex * 5.0e-4;
b4Penalty = double(belt == 4) * 5.0e-4;
floorBalance = floorBalanceCandidatePenalty(floor, st) * 0.080;
score = max(0, spare) + preferredPenalty + orderPenalty + b4Penalty + floorBalance;
end

function [st, belt] = chooseLoadBelt(floor, longSide, shortSide, st)
order = loadOrder();
belt = 0;
while st.load_stage_idx(floor) <= numel(order)
    b = order(st.load_stage_idx(floor));
    newLen = axisLengthForBelt(b, longSide, shortSide);
    if b == 4
        if b4TopLoadCanFit(floor, newLen, st)
            belt = b;
            return;
        end
        return;
    else
        if loadStageCanAcceptLength(b, reservedBeltLoadUsed(floor, b, st), newLen, st)
            belt = b;
            return;
        end
    end
    st.load_stage_idx(floor) = st.load_stage_idx(floor) + 1;
end
end

function flag = b4TopLoadCanFit(floor, newLen, st)
flag = reservedBeltLoadUsed(floor, 4, st) + newLen <= ...
    b4UsableLoadLength() + st.TOL;
end

function len = b4UsableLoadLength()
len = beltLength(4) - physicalCornerWidth();
end

function st = closeLoadStage(st, floor, belt)
order = loadOrder();
idx = st.load_stage_idx(floor);
if idx <= numel(order) && order(idx) == belt
    st.load_stage_idx(floor) = idx + 1;
end
end

function order = loadOrder()
order = [3; 2; 1; 4];
end

function st = commitPushedPackage(st)
idx = st.next_id;
st.pkg_id(idx) = st.load_id;
st.pkg_floor(idx) = st.load_floor;
st.pkg_belt(idx) = 4;
if isfield(st, 'load_from_platform_contact') && st.load_from_platform_contact > 0.5
    st.pkg_pos(idx) = platformEntryPositionForB4(st.load_long, st.load_short);
elseif st.load_belt == 4
    st.pkg_pos(idx) = platformEntryPositionForB4(st.load_long, st.load_short);
else
    st.pkg_pos(idx) = incomingEntryPosition(4, st.load_long, st.load_short);
end
st.pkg_long(idx) = st.load_long;
st.pkg_short(idx) = st.load_short;
st.pkg_height(idx) = st.load_height;
st.pkg_active(idx) = 1;
st.pkg_aligned(idx) = 1;
st.pkg_target_belt(idx) = st.load_belt;
if st.load_belt == 4
    st.pkg_route_phase(idx) = 0;
else
    st.pkg_route_phase(idx) = 1;
end
st.pkg_shift_remaining(idx) = -1;
st.pkg_close_after_shift(idx) = 0;
st.total_loaded = st.total_loaded + 1;
st.next_id = st.next_id + 1;
if isfield(st, 'load_from_platform_contact')
    st.load_from_platform_contact = 0;
end
end

function st = registerMotorMove(st, floor, belt, dist, dir)
idx = sensorIndex(floor, belt);
st.motor_cmd(idx) = dir;
st.encoder_delta_vec(idx) = st.encoder_delta_vec(idx) + dist;
if st.active_floor == 0
    st.active_floor = floor;
    st.active_belt = belt;
    st.motor_dir = dir;
    st.encoder_delta = dist;
end
end

function S = snapshotState(st)
ids = st.pkg_id;
floors = st.pkg_floor;
belts = st.pkg_belt;
pos = st.pkg_pos;
boxLong = st.pkg_long;
boxShort = st.pkg_short;
boxHeight = st.pkg_height;
x = zeros(st.MAX_PKG,1);
y = -10 * ones(st.MAX_PKG,1);
yaw = zeros(st.MAX_PKG,1);
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5
        [x(i), y(i)] = beltXY(st.pkg_floor(i), st.pkg_belt(i), st.pkg_pos(i), ...
            st.pkg_long(i), st.pkg_short(i), st.pkg_id(i), st.pkg_aligned(i));
    end
end

targetFloor = 0;
targetBelt = 0;
targetId = 0;
if isLoadMode(st.mode) && st.load_id > 0
    targetId = st.load_id;
    targetFloor = st.load_floor;
    targetBelt = st.load_belt;
elseif st.pending_unload_id > 0
    targetId = st.pending_unload_id;
    idx = findPackageIndex(targetId, st);
    if idx > 0
        targetFloor = st.pkg_floor(idx);
        targetBelt = st.pkg_belt(idx);
    else
        targetFloor = targetFloorFromId(targetId, st.FLOOR_COUNT);
        targetBelt = 4;
    end
elseif st.current_target_id > 0
    targetId = st.current_target_id;
    idx = findPackageIndex(targetId, st);
    if idx > 0
        targetFloor = st.pkg_floor(idx);
        targetBelt = st.pkg_belt(idx);
    else
        targetFloor = targetFloorFromId(targetId, st.FLOOR_COUNT);
        targetBelt = 4;
    end
end

[tofGap, tofEmpty] = sensorTofState(ids, floors, belts, pos, boxLong, boxShort, st.MAX_PKG, st.FLOOR_COUNT);
for f = 1:st.FLOOR_COUNT
    if ~b4TofBarrierReadyDown(st, f)
        idx = sensorIndex(f, 4);
        tofGap(idx) = NaN;
        tofEmpty(idx) = 0;
    end
end
noHandoffMargin = zeros(st.FLOOR_COUNT * 4,1);
for f = 1:st.FLOOR_COUNT
    for b = 1:4
        noHandoffMargin(sensorIndex(f,b)) = noHandoffForwardMargin(f, b, st);
    end
end
displayActiveFloor = st.active_floor;
displayActiveBelt = st.active_belt;
displayMotorDir = st.motor_dir;
if st.mode == 0 && ~any(abs(st.motor_cmd) > 0.5)
    displayActiveFloor = 0;
    displayActiveBelt = 0;
    displayMotorDir = 0;
end

S.ids = ids;
S.floors = floors;
S.belts = belts;
S.pkgTargetBelts = st.pkg_target_belt;
S.pos = pos;
S.x = x;
S.y = y;
S.yaw = yaw;
S.boxLong = boxLong;
S.boxShort = boxShort;
S.boxHeight = boxHeight;
S.waitIds = st.wait_id;
S.waitFloors = st.wait_floor;
S.waitPos = st.wait_pos;
S.waitLong = st.wait_long;
S.waitShort = st.wait_short;
S.waitHeight = st.wait_height;
S.waitActive = st.wait_active;
S.waitTotal = st.wait_total;
S.phase = st.mode;
S.statusCode = statusFromMode(st.mode);
if st.mode == 0 && hasPendingLoadRoute(st)
    S.statusCode = 1;
end
if st.mode == 0 && displayMotorDir ~= 0
    S.statusCode = 2;
end
S.activeFloor = displayActiveFloor;
S.activeBelt = displayActiveBelt;
S.targetId = targetId;
S.targetFloor = targetFloor;
S.targetBelt = targetBelt;
S.circCompleteTargetId = st.circ_complete_target_id;
S.circCompleteFloor = st.circ_complete_floor;
S.pendingUnloadId = st.pending_unload_id;
S.targetYoloModeCount = st.target_yolo_mode_count;
S.targetYoloF1Seen = st.target_yolo_f1_seen;
S.targetYoloAlignedSeen = st.target_yolo_aligned_seen;
S.targetYoloLastId = st.target_yolo_last_id;
S.loadedCount = st.total_loaded;
S.unloadedCount = st.total_unloaded;
S.collisionFlag = detectOverlap(st);
S.rotationFlag = detectRotationRisk(st);
S.compactGapFlag = detectInternalGap(st);
S.dtEncoder = st.belt_encoder;
S.dtEncoderDelta = st.encoder_delta_vec;
S.dtTofGap = tofGap;
S.dtTofEmpty = tofEmpty;
S.dtNoHandoffMargin = noHandoffMargin;
S.dtMotorCmd = st.motor_cmd;
S.dtPlatformStepCmd = st.platform_step_cmd;
S.dtPusherStepCmd = st.pusher_step_cmd;
S.b4TofBarrierPos = st.b4_tof_barrier_pos;
S.b4TofBarrierTarget = st.b4_tof_barrier_target;
S.b4TofBarrierServoDeg = st.b4_tof_barrier_servo_cmd_deg;
S.b4TofBarrierMoving = st.b4_tof_barrier_moving;
S.b4TofBarrierWaitCount = st.b4_tof_barrier_wait_count;
S.b4TofBarrierFault = st.b4_tof_barrier_fault;
S.b4TofBarrierFaultCount = st.b4_tof_barrier_fault_count;
S.b4TofBarrierLastFault = st.b4_tof_barrier_last_fault;
S.platformFloor = st.platform_floor;
S.platformTargetFloor = st.platform_target_floor;
S.platformZ = st.platform_z;
S.pusherPosition = st.pusher_pos;
S.pusherTravel = st.PUSHER_TRAVEL;
S.waitSidePusherPosition = st.wait_side_pusher_pos;
S.waitSidePusherTravel = st.wait_side_pusher_travel;
S.dtWaitSidePusherStepCmd = st.wait_side_pusher_step_cmd;
directWaitPusherActive = st.mode == 20 && st.load_floor > 0 && platformAtFloor(st, st.load_floor);
targetWaitPusherActive = st.mode == 24 && st.stage_wait_area_floor > 0 && platformAtFloor(st, st.stage_wait_area_floor);
S.waitSidePusherActive = double(directWaitPusherActive || st.mode == 21 || ...
    targetWaitPusherActive || st.mode == 25);
S.waitTransferActive = double(st.mode == 20 || st.mode == 24);
S.tempBufferActive = st.temp_buffer_active;
S.tempBufferId = st.temp_buffer_id;
S.tempBufferFloor = st.temp_buffer_floor;
S.tempReinsertFloor = st.temp_reinsert_floor;
S.tempReinsertTargetBelt = st.temp_reinsert_target_belt;
S.tempUnloadCount = st.temp_unload_count;
S.tempReinsertCount = st.temp_reinsert_count;
S.dbOrderRevision = st.db_order_revision;
S.currentYaw = 0;
S.currentYaw0 = 0;
S.beltSpeedMps = st.BELT_SPEED;
S.platformSpeedMps = st.PLATFORM_SPEED;
S.pusherSpeedMps = st.PUSHER_SPEED;
S.currentPackageId = targetId;
S.currentLong = 0;
S.currentShort = 0;
S.currentHeight = 0;
S.platformBoxActive = 0;
S.platformBoxFloor = st.platform_floor;
S.platformBoxLane = st.load_lane;
S.pusherActive = double(st.mode == 4 || st.mode == 5 || st.mode == 17 || st.mode == 18);
if isLoadMode(st.mode) && st.load_id > 0
    S.currentLong = st.load_long;
    S.currentShort = st.load_short;
    S.currentHeight = st.load_height;
    S.currentYaw = st.load_yaw;
    S.currentYaw0 = st.load_yaw0;
    S.platformBoxActive = double(st.mode == 1 || st.mode == 3 || st.mode == 4 || ...
        st.mode == 19 || st.mode == 20 || st.mode == 26);
    S.platformBoxFloor = st.platform_floor;
    S.platformBoxLane = st.load_lane;
elseif ((st.mode >= 22 && st.mode <= 25) || st.mode == 27) && st.stage_wait_id > 0
    S.currentPackageId = st.stage_wait_id;
    S.currentLong = st.load_long;
    S.currentShort = st.load_short;
    S.currentHeight = st.load_height;
    S.currentYaw = st.load_yaw;
    S.currentYaw0 = st.load_yaw0;
    S.platformBoxActive = double(st.mode == 23 || st.mode == 24 || st.mode == 27);
    S.platformBoxFloor = st.platform_floor;
    S.platformBoxLane = st.load_lane;
elseif st.temp_buffer_active > 0.5 && st.temp_buffer_id > 0
    S.currentPackageId = st.temp_buffer_id;
    S.currentLong = st.temp_buffer_long;
    S.currentShort = st.temp_buffer_short;
    S.currentHeight = st.temp_buffer_height;
    S.currentYaw = 0;
    S.currentYaw0 = 0;
    S.platformBoxActive = double(st.mode == 15 || st.mode == 16 || st.mode == 17);
    S.platformBoxFloor = st.platform_floor;
    S.platformBoxLane = platformLaneFactor(st.temp_buffer_id);
elseif targetId > 0
    idx = findPackageIndex(targetId, st);
    if idx > 0
        S.currentLong = st.pkg_long(idx);
        S.currentShort = st.pkg_short(idx);
        S.currentHeight = st.pkg_height(idx);
    else
        [S.currentLong, S.currentShort, S.currentHeight] = packageDims(targetId);
    end
end
S.message = st.last_message;
S.isIdle = st.mode == 0 && ~hasPendingLoadRoute(st) && st.pending_unload_id <= 0;
S.platformReadyForLoad = platformIsFree(st);
S.maxPkg = st.MAX_PKG;
S.floorCapacityM = fullLoopLength();
S.loadLockFraction = st.LOAD_LOCK_FRACTION;
S.circAccumM = st.circ_accum;
S.circLapPercent = st.circ_accum / fullLoopLength();
end

function flag = isLoadMode(mode)
flag = mode == 1 || mode == 3 || mode == 4 || mode == 5 || ...
    mode == 10 || mode == 11 || mode == 12 || mode == 19 || ...
    mode == 20 || mode == 21 || mode == 26;
end

function s = statusFromMode(mode)
if mode == 0
    s = 0;
elseif isLoadMode(mode)
    s = 1;
elseif mode == 6 || mode == 13 || mode == 14 || mode == 15
    s = 2;
elseif mode == 16 || mode == 17 || mode == 18
    s = 2;
elseif mode == 8 || mode == 9 || mode == 22 || mode == 23 || mode == 24 || mode == 25 || mode == 27
    s = 3;
else
    s = 4;
end
end

function idx = sensorIndex(floor, belt)
idx = (floor - 1) * 4 + belt;
end

function [gapVec, emptyVec] = sensorTofState(ids, floors, belts, pos, boxLong, boxShort, maxPkg, floorCount)
gapVec = zeros(floorCount * 4,1);
emptyVec = zeros(floorCount * 4,1);
for f = 1:floorCount
    for b = 1:4
        gap = beltLength(b);
        for i = 1:maxPkg
            if ids(i) > 0 && floors(i) == f && belts(i) == b
                len = axisLengthForBelt(b, boxLong(i), boxShort(i));
                tail = pos(i) - len/2;
                gap = min(gap, tail);
            end
        end
        gap = max(gap, 0);
        idx = sensorIndex(f, b);
        gapVec(idx) = gap;
        emptyVec(idx) = double(gap >= physicalCornerWidth() - 1.0e-6);
    end
end
end

function st = advancePlatform(st)
targetZ = floorHeightForTwin(st.platform_target_floor);
err = targetZ - st.platform_z;
maxMove = st.PLATFORM_SPEED * st.Ts;
if abs(err) <= maxMove
    move = err;
    st.platform_z = targetZ;
else
    move = sign(err) * maxMove;
    st.platform_z = st.platform_z + move;
end
if abs(move) > st.TOL
    st.platform_step_cmd = round(move * st.PLATFORM_STEPS_PER_M);
end
st.platform_floor = nearestFloorFromZ(st.platform_z);
end

function st = advanceLoadYaw(st)
yawStep = st.ALIGN_YAW_SPEED * st.Ts;
if abs(st.load_yaw) <= yawStep
    st.load_yaw = 0;
else
    st.load_yaw = st.load_yaw - sign(st.load_yaw) * yawStep;
end
end

function flag = platformAtFloor(st, floor)
flag = abs(st.platform_z - floorHeightForTwin(floor)) <= 2.0e-3;
end

function z = floorHeightForTwin(floor)
cfg = parcel_manual_config();
idx = min(max(1, round(floor)), numel(cfg.floorHeightsM));
z = cfg.floorHeightsM(idx);
end

function floor = nearestFloorFromZ(z)
cfg = parcel_manual_config();
heights = zeros(1, cfg.floorCount);
for i = 1:cfg.floorCount
    heights(i) = floorHeightForTwin(i);
end
[~, floor] = min(abs(heights - z));
end

function yaw = randomLoadYaw(id)
seed = mod(id * 22695477 + 1, 20001);
yawDeg = -30 + 60 * (seed / 20000);
yaw = yawDeg * pi / 180;
end

function [longSide, shortSide, h] = packageDims(id)
cfg = parcel_manual_config();
k = boxTypeForId(id);
a = cfg.packageSizeM(k, 1) * cfg.packageScale;
b = cfg.packageSizeM(k, 2) * cfg.packageScale;
h = cfg.packageSizeM(k, 3) * cfg.packageScale;
longSide = max(a, b);
shortSide = min(a, b);
end

function k = boxTypeForId(id)
cfg = parcel_manual_config();
seed = mod(id * cfg.boxTypeSeedA + cfg.boxTypeSeedB, 2147483647);
if max(cfg.boxTypeWeights) - min(cfg.boxTypeWeights) <= 1.0e-12
    k = mod(seed, numel(cfg.boxTypeWeights)) + 1;
    return;
end
u = double(seed) / 2147483647;
edges = cumsum(cfg.boxTypeWeights(:)') / sum(cfg.boxTypeWeights);
k = find(u <= edges, 1, 'first');
if isempty(k)
    k = numel(edges);
end
end

function L = beltLength(belt)
cfg = parcel_manual_config();
idx = min(max(1, round(belt)), numel(cfg.beltLengthM));
L = cfg.beltLengthM(idx);
end

function L = fullLoopLength()
L = beltLength(1) + beltLength(2) + beltLength(3) + beltLength(4);
end

function len = axisLengthForBelt(belt, longSide, shortSide)
if belt == 1 || belt == 3
    len = longSide;
else
    len = shortSide;
end
end

function nb = nextBelt(belt)
if belt == 4
    nb = 1;
elseif belt == 1
    nb = 2;
elseif belt == 2
    nb = 3;
else
    nb = 4;
end
end

function pb = prevBelt(belt)
if belt == 4
    pb = 3;
elseif belt == 3
    pb = 2;
elseif belt == 2
    pb = 1;
else
    pb = 4;
end
end

function coord = packageLoopCoord(idx, st)
coord = loopBeltStart(st.pkg_belt(idx)) + st.pkg_pos(idx);
coord = mod(coord, fullLoopLength());
end

function start = loopBeltStart(belt)
if belt == 4
    start = 0;
elseif belt == 1
    start = beltLength(4);
elseif belt == 2
    start = beltLength(4) + beltLength(1);
else
    start = beltLength(4) + beltLength(1) + beltLength(2);
end
end

function d = safeForwardMoveDistance(floor, belt, desired, st, allowHandoff)
d = desired;
L = beltLength(belt);
nb = nextBelt(belt);
receiverGap = topGap(floor, nb, st);
cornerReserve = physicalCornerWidth();
approachLimit = max(0, L - cornerReserve);
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        tail = st.pkg_pos(i) - len/2;
        front = st.pkg_pos(i) + len/2;
        if allowHandoff <= 0.5
            d = min(d, max(0, L - front));
        elseif receiverGap < cornerReserve - st.TOL && front + d > approachLimit + st.TOL
            d = min(d, max(0, approachLimit - front - 1.0e-5));
        elseif front + d > L + st.TOL
            if receiverGap < cornerReserve - st.TOL
                d = min(d, max(0, L - front - 1.0e-5));
            end
        elseif front > L + st.TOL && tail < L - st.TOL && receiverGap < cornerReserve - st.TOL
            d = 0;
        end
    end
end
d = max(0, d);
d = collisionLimitedForwardDistance(floor, belt, d, st);
end

function safeD = collisionLimitedForwardDistance(floor, belt, desired, st)
safeD = max(0, desired);
if safeD <= st.TOL
    return;
end
if ~forwardMoveWouldOverlap(floor, belt, safeD, st)
    return;
end
if forwardMoveWouldOverlap(floor, belt, 0, st)
    safeD = 0;
    return;
end
lo = 0;
hi = safeD;
for iter = 1:14
    mid = (lo + hi) / 2;
    if forwardMoveWouldOverlap(floor, belt, mid, st)
        hi = mid;
    else
        lo = mid;
    end
end
safeD = max(0, lo - 0.003);
end

function flag = forwardMoveWouldOverlap(floor, belt, dist, st)
candidate = moveBeltForward(st, floor, belt, dist);
flag = detectOverlap(candidate) > 0.5;
end

function margin = noHandoffForwardMargin(floor, belt, st)
margin = beltLength(belt);
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        front = st.pkg_pos(i) + len/2;
        margin = min(margin, beltLength(belt) - front);
    end
end
margin = max(margin, 0);
end

function st = moveBeltForward(st, floor, belt, dist)
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        st.pkg_pos(i) = st.pkg_pos(i) + dist;
    end
end
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        tail = st.pkg_pos(i) - len/2;
        if tail >= beltLength(belt) - st.TOL
            st = transferPackageToNextBelt(st, belt, i);
        end
    end
end
end

function st = moveBeltReverse(st, floor, belt, dist)
dist = min(dist, reverseNoHandoffMargin(floor, belt, st));
if dist <= st.TOL
    return;
end
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        st.pkg_pos(i) = st.pkg_pos(i) - dist;
    end
end
end

function margin = reverseNoHandoffMargin(floor, belt, st)
margin = inf;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        front = st.pkg_pos(i) + len/2;
        margin = min(margin, max(0, front - st.TOL));
    end
end
if isinf(margin)
    margin = 0;
end
end

function st = transferPackageToNextBelt(st, belt, idx)
nb = nextBelt(belt);
st.pkg_belt(idx) = nb;
st.pkg_pos(idx) = incomingEntryPosition(nb, st.pkg_long(idx), st.pkg_short(idx));
st.pkg_aligned(idx) = 1;
end

function pkgPos = moveBeltSigned(st, floor, belt, dist)
pkgPos = st.pkg_pos;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        pkgPos(i) = pkgPos(i) + dist;
    end
end
end

function pos = incomingEntryPosition(belt, longSide, shortSide)
entryAxis = axisLengthForBelt(belt, longSide, shortSide);
pos = max(entryAxis / 2, physicalCornerWidth() - entryAxis / 2);
end

function pos = platformEntryPositionForB4(longSide, shortSide)
pos = axisLengthForBelt(4, longSide, shortSide) / 2;
end

function gap = topGap(floor, belt, st)
gap = beltLength(belt);
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
        len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        gap = min(gap, st.pkg_pos(i) - len/2);
    end
end
gap = max(gap, 0);
end

function c = physicalCornerWidth()
cfg = parcel_manual_config();
c = cfg.cornerGapM;
end

function used = floorLoadUsed(floor, st)
used = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor
        used = used + axisLengthForBelt(st.pkg_belt(i), st.pkg_long(i), st.pkg_short(i));
    end
end
end

function used = b4LoadUsed(floor, st)
used = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == 4
        used = used + axisLengthForBelt(4, st.pkg_long(i), st.pkg_short(i));
    end
end
end

function used = reservedBeltLoadUsed(floor, belt, st)
used = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor
        targetBelt = st.pkg_target_belt(i);
        if targetBelt <= 0
            targetBelt = st.pkg_belt(i);
        end
        if targetBelt == belt
            used = used + axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
        end
    end
end
end

function fraction = floorLoadFraction(floor, st)
fraction = floorLoadUsed(floor, st) / fullLoopLength();
end

function idx = findPackageIndex(id, st)
idx = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) > 0.5 && st.pkg_id(i) == id
        idx = i;
        return;
    end
end
end

function floor = targetFloorFromId(id, floorCount)
floor = 1 + mod(id - 1, floorCount);
end

function [x, y] = beltXY(floor, belt, s, longSide, shortSide, id, aligned)
cfg = parcel_manual_config();
beltWidth = cfg.beltWidthM;
L1 = beltLength(1);
L2 = beltLength(2);
L3 = beltLength(3);
L4 = beltLength(4);
verticalSpan = max(L2, L4);
platformY = -0.35;
stopperX = -0.15;
stopperY = platformY + 0.30;
Ax = stopperX - 0.03/2;
Ay = stopperY;
floorOffset = (floor - 2) * 1.7;
belt3RightX = Ax;
belt3LeftX = belt3RightX - L3;
belt3Y = Ay + floorOffset;
belt3ConnectY = belt3Y + beltWidth/2;
belt4LeftX = Ax;
belt4RightX = Ax + beltWidth;
belt4StartY = belt3ConnectY - beltWidth;
belt4EndY = belt4StartY + L4;
belt1RightX = belt4RightX;
belt1LeftX = belt1RightX - L1;
belt1Y = belt3ConnectY + verticalSpan - beltWidth/2;
belt2X = belt1LeftX - beltWidth/2;
belt2TopY = belt3ConnectY + verticalSpan;
belt4X = (belt4LeftX + belt4RightX) / 2;
mirrorCenterX = (belt4X + belt2X) / 2;
b3RightM = 2 * mirrorCenterX - belt3LeftX;
b4LeftM = 2 * mirrorCenterX - belt4RightX;
b4RightM = 2 * mirrorCenterX - belt4LeftX;
b1LeftM = 2 * mirrorCenterX - belt1RightX;
b2XM = 2 * mirrorCenterX - belt2X;
b2LeftM = b2XM - beltWidth/2;
displayPad = max([longSide, shortSide, 0.10]) + 0.05;
sc = min(max(s, -displayPad), beltLength(belt) + displayPad);
lane = platformLaneFactor(id);
if belt == 4
    if aligned > 0.5
        x = b4RightM - longSide/2;
    else
        x = lateralCenter(b4LeftM, b4RightM, longSide, lane);
    end
    y = belt4StartY + sc;
elseif belt == 1
    x = b1LeftM + sc;
    if aligned > 0.5
        y = belt1Y - beltWidth/2 + shortSide/2;
    else
        y = lateralCenter(belt1Y - beltWidth/2, belt1Y + beltWidth/2, shortSide, lane);
    end
elseif belt == 2
    if aligned > 0.5
        x = b2LeftM + longSide/2;
    else
        x = lateralCenter(b2LeftM, b2LeftM + beltWidth, longSide, lane);
    end
    y = belt2TopY - sc;
elseif belt == 3
    x = b3RightM - sc;
    y = belt3Y + beltWidth/2 - shortSide/2;
else
    x = 0;
    y = -10;
end
end

function center = lateralCenter(lowEdge, highEdge, crossSize, lane)
lo = lowEdge + crossSize/2;
hi = highEdge - crossSize/2;
if hi < lo
    center = (lowEdge + highEdge) / 2;
else
    center = lo + lane * (hi - lo);
end
end

function r = platformLaneFactor(id)
seed = mod(id * 1664525 + 1013904223, 10000);
r = seed / 9999;
r = min(max(r, 0.08), 0.92);
end

function flag = detectOverlap(st)
flag = 0;
for i = 1:st.MAX_PKG
    if st.pkg_active(i) <= 0.5
        continue;
    end
    for j = i+1:st.MAX_PKG
        if st.pkg_active(j) > 0.5 && st.pkg_floor(i) == st.pkg_floor(j)
            if packageFootprintsOverlap(i, j, st)
                flag = 1;
                return;
            end
        end
    end
end
end

function flag = packageFootprintsOverlap(i, j, st)
[xi, yi] = beltXY(st.pkg_floor(i), st.pkg_belt(i), st.pkg_pos(i), ...
    st.pkg_long(i), st.pkg_short(i), st.pkg_id(i), st.pkg_aligned(i));
[xj, yj] = beltXY(st.pkg_floor(j), st.pkg_belt(j), st.pkg_pos(j), ...
    st.pkg_long(j), st.pkg_short(j), st.pkg_id(j), st.pkg_aligned(j));
wi = max(st.pkg_long(i), 0.06);
hi = max(st.pkg_short(i), 0.05);
wj = max(st.pkg_long(j), 0.06);
hj = max(st.pkg_short(j), 0.05);
tol = 0.010;
flag = abs(xi - xj) < (wi + wj) / 2 - tol && ...
    abs(yi - yj) < (hi + hj) / 2 - tol;
end

function flag = detectRotationRisk(st)
flag = 0;
for floor = 1:st.FLOOR_COUNT
    for belt = 1:4
        nb = nextBelt(belt);
        nbIdx = sensorIndex(floor, nb);
        if nbIdx > numel(st.motor_cmd) || abs(st.motor_cmd(nbIdx)) <= 0.5
            continue;
        end
        for i = 1:st.MAX_PKG
            if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
                len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
                tail = st.pkg_pos(i) - len/2;
                front = st.pkg_pos(i) + len/2;
                if front > beltLength(belt) + st.TOL && tail < beltLength(belt) - st.TOL
                    flag = 1;
                    return;
                end
            end
        end
    end
end
end

function flag = detectInternalGap(st)
flag = 0;
for floor = 1:st.FLOOR_COUNT
    for belt = 1:4
        starts = zeros(st.MAX_PKG,1);
        ends = zeros(st.MAX_PKG,1);
        n = 0;
        for i = 1:st.MAX_PKG
            if st.pkg_active(i) > 0.5 && st.pkg_floor(i) == floor && st.pkg_belt(i) == belt
                n = n + 1;
                len = axisLengthForBelt(belt, st.pkg_long(i), st.pkg_short(i));
                starts(n) = st.pkg_pos(i) - len/2;
                ends(n) = st.pkg_pos(i) + len/2;
            end
        end
        if n > 1
            [starts, order] = sort(starts(1:n));
            ends = ends(order);
            for k = 1:n-1
                if starts(k+1) - ends(k) > 0.003
                    flag = 1;
                    return;
                end
            end
        end
    end
end
end
