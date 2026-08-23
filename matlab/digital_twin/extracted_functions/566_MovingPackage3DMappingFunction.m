function [moving_pkg_T, moving_pkg_R, moving_pkg_S, moving_arrive_done, moving_arrive_floor] ...
    = MovingPackage3DMappingFunction( ...
    mission_phase, state_id, platform_z_3d, rotator_yaw, pusher_pos, platform_y, box_yaw, load_done, rotation_done, visual_count, target_floor)% MovingPackage3DMappingFunction
% ?숈쟻 ?앸같 1媛쒕? ?뚮옯?????몄뀛 ??Belt4 ??Belt1 ??Belt2 ??Belt3/?ㅽ넗?????꾩튂源뚯? ?대룞?쒗궓??
%
% 二쇱쓽:
% - spawn_pkg ?놁씠 ?숈옉?섎뒗 踰꾩쟾
% - load_done?쇰줈 諛뺤뒪瑜??④린吏 ?딆쓬
% - moving_arrive_done??1???섎뒗 ?쒓컙 VisualArrivalCounterFunction??visual_count瑜?利앷??쒗궡
% - Package3DMappingFunction? visual_count ?댄븯???뺤쟻 諛뺤뒪留??쒖떆??%
% 異쒕젰:
% moving_pkg_T = [x y z]
% moving_pkg_R = [roll pitch yaw]
% moving_pkg_S = [sx sy sz]
% moving_arrive_done = ?꾩갑 ?쒓컙 1?꾩뒪

% =========================
% Persistent ?곹깭
% =========================
persistent active
persistent latched_yaw
persistent yaw_locked
persistent prev_rotation_done
persistent pushed_distance_max
persistent path_started
persistent path_s
persistent current_yaw_rad
persistent latched_pkg_z
persistent waiting_for_next_cycle
persistent visual_count_at_start
persistent target_slot_latched
persistent box_detached
persistent detach_z
persistent target_floor_latched
persistent smooth_yaw_initialized

if isempty(smooth_yaw_initialized)
    smooth_yaw_initialized = false;
end

if isempty(target_floor_latched)
    target_floor_latched = 2;
end

if isempty(box_detached)
    box_detached = false;
end

if isempty(detach_z)
    detach_z = 0;
end

if isempty(active)
    active = false;
end

if isempty(latched_yaw)
    latched_yaw = 0;
end

if isempty(yaw_locked)
    yaw_locked = false;
end

if isempty(prev_rotation_done)
    prev_rotation_done = 0;
end

if isempty(pushed_distance_max)
    pushed_distance_max = 0;
end

if isempty(path_started)
    path_started = false;
end

if isempty(path_s)
    path_s = 0;
end

if isempty(current_yaw_rad)
    current_yaw_rad = pi/2;
end

if isempty(latched_pkg_z)
    latched_pkg_z = 0;
end

if isempty(waiting_for_next_cycle)
    waiting_for_next_cycle = false;
end

if isempty(visual_count_at_start)
    visual_count_at_start = 0;
end

if isempty(target_slot_latched)
    target_slot_latched = 1;
end

% 湲곕낯 異쒕젰
moving_arrive_done = 0;
moving_arrive_floor = 0;
rotation_done_rising = (rotation_done > 0.5) && (prev_rotation_done <= 0.5);
debug_active = 0;
debug_waiting = 0;
debug_start_allowed = 0;
debug_target_slot = target_slot_latched;
debug_visual_count = visual_count;
debug_mission_phase = mission_phase;

% =========================
% 諛뺤뒪 ?ш린
% =========================
pkg_short = 0.12;
pkg_long  = 0.16;
pkg_h     = 0.08;

moving_pkg_S = [pkg_short, pkg_long, pkg_h];

% =========================
% ?뚮옯??湲곗? ?꾩튂
% =========================
platform_x = 0.0;
pkg_home_x = platform_x;
pkg_home_y = platform_y;

platform_thickness = 0.04;
pkg_center_z_now = platform_z_3d + platform_thickness/2 + pkg_h/2;

hide_z = -10.0;

% =========================
% ?대깽??媛먯?
% =========================
rotation_done_rising = (rotation_done > 0.5) && (prev_rotation_done <= 0.5);

% =========================
% ?뚮옯?????뺣젹 紐⑺몴 yaw
% yaw = pi/2?대㈃ 諛뺤뒪 吏㏃? 蹂 0.12m媛 +y 諛⑺뼢, 利??몄뀛 吏꾪뻾諛⑺뼢???ν븿
% =========================
platform_aligned_yaw_rad = pi/2;

% =========================
% ?ㅼ쓬 ?ъ씠???湲??댁젣
% =========================
% MovingPackage媛 ?꾩갑?댁꽌 visual_count媛 利앷?????
% ?몄뀛/?뚯쟾/?깅줉 ?좏샇媛 珥덇린?붾릺硫??ㅼ쓬 諛뺤뒪 ?앹꽦 ?덉슜
if waiting_for_next_cycle
    same_floor_count_increased = (target_floor == target_floor_latched) && ...
                                 (visual_count > visual_count_at_start);

    floor_changed = (target_floor ~= target_floor_latched);

    if (same_floor_count_increased || floor_changed) && ...
       (pusher_pos < 0.005) && ...
       (rotation_done < 0.5) && ...
       (load_done < 0.5)
        waiting_for_next_cycle = false;
    end
end

% =========================
% ??諛뺤뒪 ?쒖옉 議곌굔
% =========================
% spawn_pkg媛 ?놁쑝誘濡??꾩옱??state? 珥덇린 議곌굔?쇰줈 ?쒖옉??異붿젙?쒕떎.
% 異뷀썑 spawn_pkg瑜?留뚮뱾硫???遺遺꾩쓣 spawn rising edge濡?諛붽씀??寃껋씠 媛???덉젙?곸씠??
start_allowed = (mission_phase == 0) && ...
                (active == false) && ...
                (waiting_for_next_cycle == false) && ...
                (pusher_pos < 0.005) && ...
                (rotation_done < 0.5) && ...
                (load_done < 0.5) && ...
                (state_id < 7);
if start_allowed
    active = true;
    latched_yaw = box_yaw;
    yaw_locked = false;
    pushed_distance_max = 0;
    path_started = false;
    path_s = 0;
    target_floor_latched = target_floor;

    box_detached = false;
    detach_z = pkg_center_z_now;

    current_yaw_rad = platform_aligned_yaw_rad + latched_yaw * pi / 180;
    smooth_yaw_initialized = true;
    latched_pkg_z = pkg_center_z_now;

    visual_count_at_start = visual_count;

    target_slot_latched = visual_count + 1;
    if target_slot_latched < 1
        target_slot_latched = 1;
    end
    if target_slot_latched > 6
        target_slot_latched = 6;
    end
end

% =========================
% ?뚯쟾 ?꾨즺 ??yaw 怨좎젙
% =========================
if active && rotation_done_rising
    yaw_locked = true;
end

% =========================
% ?몄뀛 理쒕? ?대룞嫄곕━ ???% =========================
if active
    if pusher_pos > pushed_distance_max
        pushed_distance_max = pusher_pos;
    end
end

% =========================
% ?몄뀛 ?꾨즺 ??寃쎈줈 ?대룞 ?쒖옉
% =========================
pusher_finished = pushed_distance_max >= 0.22;

if active && pusher_finished && box_detached == false
    box_detached = true;

    % ???쒓컙遺??諛뺤뒪???뚮옯?쇨낵 蹂꾧컻濡??吏곸씤??
    % ?곕씪???꾩옱 ?믪씠瑜?怨좎젙 ??ν븳??
    detach_z = pkg_center_z_now;
    latched_pkg_z = pkg_center_z_now;
end

if active && pusher_finished && path_started == false
    path_started = true;
    path_s = 0;

    yaw_locked = true;
    current_yaw_rad = platform_aligned_yaw_rad;
end

% =========================
% 寃쎈줈 ?뚮씪誘명꽣
% =========================
[path_total_length, seg1_end, seg2_end, seg3_end] = getPathLengths(platform_y, target_slot_latched);

% ?대룞 ?띾룄
path_speed_per_step = 0.012;

if active && path_started
    if path_s < path_total_length
        path_s = path_s + path_speed_per_step;
    end

    if path_s > path_total_length
        path_s = path_total_length;
    end
end

% =========================
% ?섏감/?꾨즺 phase?먯꽌??媛뺤젣 ?④?
% =========================
%if mission_phase > 0
%    active = false;
%    yaw_locked = false;
%    pushed_distance_max = 0;
%    path_started = false;
%    path_s = 0;
%    waiting_for_next_cycle = true;
%    box_detached = false;
%    detach_z = 0;
%end

% =========================
% 異쒕젰 怨꾩궛
% =========================
if active

    if path_started
        [x, y, target_yaw_rad] = getPathPose(path_s, platform_y, target_slot_latched, seg1_end, seg2_end, seg3_end);

        % ?곗꽑 鍮숆?鍮숆? ?꾨뒗 臾몄젣 諛⑹?瑜??꾪빐 yaw??紐⑺몴媛믪쑝濡?吏곸젒 吏?뺥븳??
        % ?덉젙????遺?쒕윭???뚯쟾???ㅼ떆 異붽??섎㈃ ?쒕떎.
        current_yaw_rad = target_yaw_rad;

        z = latched_pkg_z;

        moving_pkg_T = [x, y, z];
        moving_pkg_R = [0, 0, current_yaw_rad];

        if path_s >= path_total_length
            moving_arrive_done = 1;
            moving_arrive_floor = target_floor_latched;

            active = false;
            yaw_locked = false;
            pushed_distance_max = 0;
            path_started = false;
            path_s = 0;
            box_detached = false;
            detach_z = 0;

            waiting_for_next_cycle = true;
        end

    else
        % ?뚮옯???? ?뚯쟾???? ?몄뀛 ?대룞 援ш컙
        push_visual_gain = 1.0;
        push_y = push_visual_gain * pushed_distance_max;

        x = pkg_home_x;
        y = pkg_home_y + push_y;
        if box_detached
            z = detach_z;
        else
            z = pkg_center_z_now;
        end

        moving_pkg_T = [x, y, z];

        % =========================
        % ?뚮옯????諛뺤뒪 yaw 遺?쒕윭??紐⑺몴媛?異붿쥌
        % =========================
        % 以묒슂:
        % - rotator_yaw瑜?吏곸젒 ?곕씪媛硫?紐⑺몴媛곸쓣 吏?섏낀?ㅺ? 蹂댁젙?섎뒗 ?꾩긽???앷릿??
        % - ?쒓컖?붿슜 諛뺤뒪 yaw????긽 理쒖쥌 理쒖냼湲몄씠 紐⑺몴媛?platform_aligned_yaw_rad)留?諛붾씪蹂닿쾶 ?쒕떎.
        % - ?꾩옱 yaw?먯꽌 紐⑺몴 yaw源뚯? ??step???묎렐?섎?濡??덈? 紐⑺몴媛곸쓣 ?섏뼱媛吏 ?딅뒗??

        Ts_yaw = 0.05;
        yaw_speed = 1.2;     % rad/s, ??鍮좊Ⅴ寃??먰븯硫?1.5~2.0
        yaw_step = yaw_speed * Ts_yaw;

        desired_yaw_rad = platform_aligned_yaw_rad;

        yaw_err = wrapToPiLocal(desired_yaw_rad - current_yaw_rad);

        if abs(yaw_err) <= yaw_step
            current_yaw_rad = desired_yaw_rad;
        else
            if yaw_err > 0
                current_yaw_rad = current_yaw_rad + yaw_step;
            else
                current_yaw_rad = current_yaw_rad - yaw_step;
            end
        end

        moving_pkg_R = [0, 0, current_yaw_rad];
    end

else
    moving_pkg_T = [0, 0, hide_z];
    moving_pkg_R = [0, 0, 0];
end


% =========================
% ?댁쟾媛????% =========================
prev_rotation_done = rotation_done;

end


function [path_total_length, seg1_end, seg2_end, seg3_end] = getPathLengths(platform_y_local, target_slot)
% 寃쎈줈 湲몄씠 怨꾩궛

belt_width = 0.25;
pkg_short = 0.12;
pkg_visual_gap = 0.005;
slot_pitch = pkg_short + pkg_visual_gap;

L1 = 0.51;
L3 = 0.51;
L4 = 1.11;

max_on_belt3 = 4;

stopper_x = -0.15;
stopper_y = platform_y_local + 0.30;
stopper_thick_x = 0.03;

A_x = stopper_x - stopper_thick_x/2;
A_y = stopper_y;

side_y = 1;
side_x = 1;

% Belt3
belt3_center_y = A_y;
belt3_connect_edge_y = belt3_center_y + side_y * belt_width/2;

% Belt4
if side_x > 0
    belt4_left_x  = A_x;
    belt4_right_x = A_x + belt_width;
else
    belt4_left_x  = A_x - belt_width;
    belt4_right_x = A_x;
end

belt4_start_y_nominal = belt3_connect_edge_y;
belt4_platform_extend = 0.25;

belt4_start_y = belt4_start_y_nominal - side_y * belt4_platform_extend;
belt4_end_y   = belt4_start_y_nominal + side_y * L4;

seg_belt4 = abs(belt4_end_y - belt4_start_y);

% Belt1
if side_x > 0
    belt4_outer_x = belt4_right_x;
else
    belt4_outer_x = belt4_left_x;
end

belt1_right_x = belt4_outer_x;
belt1_left_x  = belt1_right_x - L1;
seg_belt1 = abs(belt1_right_x - belt1_left_x);

% Belt2
belt1_inner_edge_y = belt4_end_y;
belt1_outer_edge_y = belt1_inner_edge_y + side_y * belt_width;

belt2_start_y = belt1_outer_edge_y;
belt2_end_y   = belt3_connect_edge_y;
seg_belt2_full = abs(belt2_start_y - belt2_end_y);

if target_slot <= max_on_belt3
    % 紐⑺몴媛 Belt3 ?꾩뿉 ?덉쓬
    stopper_center_gap = pkg_short / 2;
    target_gap = stopper_center_gap + slot_pitch * (target_slot - 1);

    seg_belt3 = L3 - target_gap;

    if seg_belt3 < 0
        seg_belt3 = 0;
    end

    seg1_end = seg_belt4;
    seg2_end = seg_belt4 + seg_belt1;
    seg3_end = seg_belt4 + seg_belt1 + seg_belt2_full;
    path_total_length = seg_belt4 + seg_belt1 + seg_belt2_full + seg_belt3;

else
    % 紐⑺몴媛 Belt2 ?꾩뿉 ?덉쓬
    % Belt4? Belt1??吏????
    % Belt2 ?쒖옉?먯뿉??紐⑺몴 ?꾩튂源뚯? ?ㅼ젣 嫄곕━留뚰겮 ?대룞?댁빞 ?쒕떎.
    k = target_slot - max_on_belt3;

    belt2_entry_gap = 0.04;

    % Package3DMappingFunction???뺤쟻 諛뺤뒪 ?꾩튂? ?숈씪??紐⑺몴 y
    % 6踰덉㎏ 諛뺤뒪媛 5踰덉㎏ 諛뺤뒪 ?곸뿭??移⑤쾾??蹂댁씠??寃껋쓣 留됯린 ?꾪븳 ?숈쟻 寃쎈줈 蹂댁젙
    % k = 1 : 5踰덉㎏ 諛뺤뒪
    % k = 2 : 6踰덉㎏ 諛뺤뒪
    moving_stop_margin = 0.01;

    target_y_on_belt2 = belt2_end_y + side_y * (belt2_entry_gap + slot_pitch * (k - 1) + moving_stop_margin);

    % 湲곗〈 ?ㅻ쪟:
    % seg_belt2_partial = belt2_entry_gap;
    % ?대젃寃??섎㈃ 0.04m留??대룞?댁꽌 ?쒓컙?대룞泥섎읆 蹂댁엫.
    %
    % ?섏젙:
    % Belt2 ?쒖옉?먯뿉??target_y_on_belt2源뚯????ㅼ젣 嫄곕━ ?ъ슜
    seg_belt2_partial = abs(belt2_start_y - target_y_on_belt2);

    seg1_end = seg_belt4;
    seg2_end = seg_belt4 + seg_belt1;
    seg3_end = seg2_end + seg_belt2_partial;
    path_total_length = seg_belt4 + seg_belt1 + seg_belt2_partial;
end

end


function [x, y, yaw_rad] = getPathPose(s, platform_y_local, target_slot, seg1_end, seg2_end, seg3_end)
% 寃쎈줈???꾩튂? yaw 怨꾩궛

belt_width = 0.25;
pkg_short = 0.12;
pkg_visual_gap = 0.005;
slot_pitch = pkg_short + pkg_visual_gap;

L1 = 0.51;
L3 = 0.51;
L4 = 1.11;

max_on_belt3 = 4;

stopper_x = -0.15;
stopper_y = platform_y_local + 0.30;
stopper_thick_x = 0.03;

A_x = stopper_x - stopper_thick_x/2;
A_y = stopper_y;

side_y = 1;
side_x = 1;

% Belt3
belt3_right_x = A_x;
belt3_left_x  = belt3_right_x - L3;
belt3_center_y = A_y;
belt3_connect_edge_y = belt3_center_y + side_y * belt_width/2;

% Belt4
if side_x > 0
    belt4_left_x  = A_x;
    belt4_right_x = A_x + belt_width;
else
    belt4_left_x  = A_x - belt_width;
    belt4_right_x = A_x;
end

belt4_center_x = (belt4_left_x + belt4_right_x)/2;

belt4_start_y_nominal = belt3_connect_edge_y;
belt4_platform_extend = 0.25;

belt4_start_y = belt4_start_y_nominal - side_y * belt4_platform_extend;
belt4_end_y   = belt4_start_y_nominal + side_y * L4;

if side_x > 0
    belt4_outer_x = belt4_right_x;
else
    belt4_outer_x = belt4_left_x;
end

% Belt1
belt1_right_x = belt4_outer_x;
belt1_left_x  = belt1_right_x - L1;

belt1_inner_edge_y = belt4_end_y;
belt1_center_y = belt1_inner_edge_y + side_y * belt_width/2;
belt1_outer_edge_y = belt1_inner_edge_y + side_y * belt_width;

% Belt2
belt2_right_x = belt1_left_x;
belt2_left_x  = belt2_right_x - belt_width;
belt2_center_x = (belt2_left_x + belt2_right_x)/2;

belt2_start_y = belt1_outer_edge_y;
belt2_end_y   = belt3_connect_edge_y;

% target ?꾩튂
if target_slot <= max_on_belt3
    stopper_center_gap = pkg_short / 2;
    target_gap = stopper_center_gap + slot_pitch * (target_slot - 1);

    target_x = belt3_right_x - target_gap;
    target_y = belt3_center_y;
else
    k = target_slot - max_on_belt3;
    belt2_entry_gap = 0.04;

    % ?숈쟻 諛뺤뒪媛 ?뺤쟻 諛뺤뒪 履쎌쓣 ?댁쭩 移⑤쾾??蹂댁씠??寃껋쓣 留됯린 ?꾪븳 ?쒓컖 蹂댁젙
    % 媛믪씠 ?덈Т ?щ㈃ 6踰?諛뺤뒪媛 ?ㅼ뿉 ?덈Т ?⑥뼱??蹂댁씠誘濡?0.01遺???쒖옉
    % 6踰덉㎏ 諛뺤뒪媛 5踰덉㎏ 諛뺤뒪 ?곸뿭??移⑤쾾??蹂댁씠??寃껋쓣 留됯린 ?꾪븳 ?숈쟻 寃쎈줈 蹂댁젙
    % k = 1 : 5踰덉㎏ 諛뺤뒪
    % k = 2 : 6踰덉㎏ 諛뺤뒪
    moving_stop_margin = 0.01;

    target_x = belt2_center_x;
    target_y = belt2_end_y + side_y * (belt2_entry_gap + slot_pitch * (k - 1) + moving_stop_margin);
end

% Segment 1: Belt4
if s <= seg1_end
    ratio = safeRatio(s, seg1_end);

    x = belt4_center_x;
    y = belt4_start_y + ratio * (belt4_end_y - belt4_start_y);

    % Belt4: 吏㏃? 蹂??y諛⑺뼢
    yaw_rad = pi/2;

% Segment 2: Belt1
elseif s <= seg2_end
    local_s = s - seg1_end;
    seg_len = seg2_end - seg1_end;
    ratio = safeRatio(local_s, seg_len);

    x = belt1_right_x - ratio * (belt1_right_x - belt1_left_x);
    y = belt1_center_y;

    % Belt1: 吏㏃? 蹂??x諛⑺뼢
    yaw_rad = 0;

% Segment 3: Belt2 ?먮뒗 Belt2 ?쇰?
elseif s <= seg3_end
    local_s = s - seg2_end;
    seg_len = seg3_end - seg2_end;
    ratio = safeRatio(local_s, seg_len);

    if target_slot <= max_on_belt3
        % Belt2 ?꾩껜 ?대룞
        x = belt2_center_x;
        y = belt2_start_y + ratio * (belt2_end_y - belt2_start_y);
    else
        % Belt2 ??紐⑺몴 ?꾩튂源뚯?留??대룞
        x = belt2_center_x;
        y = belt2_start_y + ratio * (target_y - belt2_start_y);
    end

    % Belt2: 吏㏃? 蹂??y諛⑺뼢
    yaw_rad = pi/2;

% Segment 4: Belt3
else
    local_s = s - seg3_end;

    % Belt3 吏꾩엯?먯? belt3_left_x, 紐⑺몴??target_x
    seg_len = abs(target_x - belt3_left_x);
    ratio = safeRatio(local_s, seg_len);

    x = belt3_left_x + ratio * (target_x - belt3_left_x);
    y = belt3_center_y;

    % Belt3: 吏㏃? 蹂??x諛⑺뼢
    yaw_rad = 0;
end

end


function r = safeRatio(a, b)
% 0~1 鍮꾩쑉 怨꾩궛

if b <= 0
    r = 1;
else
    r = a / b;
end

if r < 0
    r = 0;
end

if r > 1
    r = 1;
end

end

function a = wrapToPiLocal(a)
% wrapToPiLocal
% angle??-pi ~ pi 踰붿쐞濡??뺢퇋??
while a > pi
    a = a - 2*pi;
end

while a < -pi
    a = a + 2*pi;
end

end