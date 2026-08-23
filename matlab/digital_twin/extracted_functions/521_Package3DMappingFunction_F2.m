function [pkg1_T, pkg2_T, pkg3_T, pkg4_T, pkg5_T, ...
          pkg1_R, pkg2_R, pkg3_R, pkg4_R, pkg5_R, ...
          pkg1_S, pkg2_S, pkg3_S, pkg4_S, pkg5_S, ...
          pkg6_T, pkg6_R, pkg6_S, ...
          pkg1_Color, pkg2_Color, pkg3_Color, pkg4_Color, pkg5_Color, pkg6_Color] = ...
          Package3DMappingFunction_F2(floor2_visual_id, floor2_visible, floor2_color_flag, platform_y, floor2_visual_count, ...
                                      target_unload_id, highlight_enable, target_package_on_belt4, ...
                                      package_on_platform_from_floor, platform_z, current_unload_id, floor2_visual_path, ...
                                      target_list, target_count, belt4_reverse_progress, belt4_reverse_phase)
% Package3DMappingFunction_F2
% 2痢??뺤쟻/?곗냽 ?대룞 諛뺤뒪 ?쒖떆 ?⑥닔
%
% 以묒슂:
% - ?댁젣 F2??FloorStorageUpdate/floor2_queue ?먮뒗 PackageSlotVisualizer/floor2_slot_id瑜?吏곸젒 ?곗? ?딅뒗??
% - Floor2VisualStateFunction????ν븳 floor2_visual_id, floor2_visual_path瑜?湲곗??쇰줈 ?쒖떆?쒕떎.
% - visual_shift?????⑥닔??吏곸젒 ?ㅼ뼱?ㅼ? ?딅뒗??
%
% ?낅젰:
% floor2_visual_id               [6x1]
% floor2_visible                 [50x1]  % 湲곗〈 ?ы듃 ?좎???% floor2_color_flag              [50x1]  % 湲곗〈 ?ы듃 ?좎???% platform_y                     [1x1]
% floor2_visual_count            [1x1]
% target_unload_id               [1x1]  % 2痢??섏감怨듦컙 以鍮????% highlight_enable               [1x1]
% target_package_on_belt4        [1x1]
% package_on_platform_from_floor [1x1]
% platform_z                     [1x1]
% current_unload_id              [1x1]  % ?꾩옱 ?ㅼ젣 ?섏감 ???% floor2_visual_path             [6x1]

% =========================
% 諛뺤뒪 ?ш린
% =========================
pkg_short = 0.12;
pkg_long  = 0.16;
pkg_h     = 0.08;

pkg_scale = [pkg_short, pkg_long, pkg_h];

% =========================
% ?됱긽 ?ㅼ젙
% =========================
normal_color = [0.75, 0.45, 0.20];
target_color = [1.0, 1.0, 0.0];

% =========================
% 2痢??믪씠
% =========================
z_floor2 = 0.27;
pkg_center_z = z_floor2 + pkg_h / 2;

hide_z = -10.0;

% =========================
% 而⑤쿋?댁뼱 湲곗?媛?% =========================
belt_width = 0.25;

L1 = 0.51;
L3 = 0.51;
L4 = 1.11;

stopper_x = -0.15;
stopper_y = platform_y + 0.30;
stopper_thick_x = 0.03;

A_x = stopper_x - stopper_thick_x / 2;
A_y = stopper_y;

side_y = 1;
side_x = 1;

% =========================
% Belt3 geometry
% =========================
belt3_right_x = A_x;
belt3_center_y = A_y;
belt3_connect_edge_y = belt3_center_y + side_y * belt_width / 2;

% =========================
% Belt4 geometry
% =========================
if side_x > 0
    belt4_left_x  = A_x;
    belt4_right_x = A_x + belt_width;
else
    belt4_left_x  = A_x - belt_width;
    belt4_right_x = A_x;
end

belt4_center_x = (belt4_left_x + belt4_right_x) / 2;

belt4_start_y_nominal = belt3_connect_edge_y;
belt4_platform_extend = 0.25;

belt4_start_y = belt4_start_y_nominal - side_y * belt4_platform_extend;
belt4_end_y = belt4_start_y_nominal + side_y * L4;

if side_x > 0
    belt4_outer_x = belt4_right_x;
else
    belt4_outer_x = belt4_left_x;
end

% =========================
% ?섏감怨듦컙 醫뚰몴
% =========================
handoff_x = belt4_center_x;
handoff_y = belt4_start_y + side_y * (pkg_short / 2 + 0.02);
handoff_z = pkg_center_z;
handoff_yaw = pi / 2;

% Belt4 ?욎そ 理쒕? ?꾩튂 ?쒗븳
belt4_max_y = belt4_end_y - side_y * (pkg_short / 2 + 0.02);

% =========================
% ?뚮옯?????뚯닔 諛뺤뒪 醫뚰몴
% =========================
platform_pkg_x = 0.0;
platform_pkg_y = platform_y ;
platform_pkg_z = platform_z + pkg_h / 2;
platform_pkg_yaw = pi / 2;

% =========================
% Belt1 geometry
% =========================
belt1_right_x = belt4_outer_x;
belt1_left_x  = belt1_right_x - L1;

belt1_inner_edge_y = belt4_end_y;
belt1_outer_edge_y = belt1_inner_edge_y + side_y * belt_width;

% =========================
% Belt2 geometry
% =========================
belt2_right_x = belt1_left_x;
belt2_left_x  = belt2_right_x - belt_width;
belt2_center_x = (belt2_left_x + belt2_right_x) / 2;

belt2_end_y = belt3_connect_edge_y;

% =========================
% ?щ’ 諛곗튂 ?뚮씪誘명꽣
% =========================
max_on_belt3 = 4;

pkg_visual_gap = 0.005;
slot_pitch = pkg_short + pkg_visual_gap;

stopper_center_gap = pkg_short / 2;
belt2_entry_gap = 0.04;

% =========================
% 湲곗〈 ?낅젰 ?ы듃 ?ъ슜 泥섎━
% =========================
% floor2_visible, floor2_color_flag??湲곗〈 紐⑤뜽 ?곌껐???좎??섍린 ?꾪븳 ?낅젰?대떎.
% ?꾩옱 F2??floor2_visual_id/path 湲곗??쇰줈 ?숈옉?쒕떎.
dummy_keep = floor2_visible(1) + floor2_color_flag(1);
dummy_keep = dummy_keep + 0;

% =========================
% ?쒖떆 ?щ?
% =========================
visible1 = (floor2_visual_id(1) > 0);
visible2 = (floor2_visual_id(2) > 0);
visible3 = (floor2_visual_id(3) > 0);
visible4 = (floor2_visual_id(4) > 0);
visible5 = (floor2_visual_id(5) > 0);
visible6 = (floor2_visual_id(6) > 0);
% =========================
% 媛?諛뺤뒪 ?앹꽦
% =========================
[pkg1_T, pkg1_R, pkg1_S, pkg1_Color] = makePkg(floor2_visual_id(1), floor2_visual_path(1), visible1, ...
    belt3_right_x, belt3_center_y, belt2_center_x, belt2_end_y, ...
    belt4_center_x, belt4_max_y, ...
    slot_pitch, stopper_center_gap, belt2_entry_gap, max_on_belt3, ...
    pkg_center_z, hide_z, pkg_scale, side_y, ...
    target_unload_id, highlight_enable, target_package_on_belt4, ...
    handoff_x, handoff_y, handoff_z, handoff_yaw, ...
    package_on_platform_from_floor, platform_pkg_x, platform_pkg_y, platform_pkg_z, platform_pkg_yaw, ...
    current_unload_id, normal_color, target_color, target_list, target_count, belt4_reverse_progress, belt4_reverse_phase);

[pkg2_T, pkg2_R, pkg2_S, pkg2_Color] = makePkg(floor2_visual_id(2), floor2_visual_path(2), visible2, ...
    belt3_right_x, belt3_center_y, belt2_center_x, belt2_end_y, ...
    belt4_center_x, belt4_max_y, ...
    slot_pitch, stopper_center_gap, belt2_entry_gap, max_on_belt3, ...
    pkg_center_z, hide_z, pkg_scale, side_y, ...
    target_unload_id, highlight_enable, target_package_on_belt4, ...
    handoff_x, handoff_y, handoff_z, handoff_yaw, ...
    package_on_platform_from_floor, platform_pkg_x, platform_pkg_y, platform_pkg_z, platform_pkg_yaw, ...
    current_unload_id, normal_color, target_color, target_list, target_count, belt4_reverse_progress, belt4_reverse_phase);

[pkg3_T, pkg3_R, pkg3_S, pkg3_Color] = makePkg(floor2_visual_id(3), floor2_visual_path(3), visible3, ...
    belt3_right_x, belt3_center_y, belt2_center_x, belt2_end_y, ...
    belt4_center_x, belt4_max_y, ...
    slot_pitch, stopper_center_gap, belt2_entry_gap, max_on_belt3, ...
    pkg_center_z, hide_z, pkg_scale, side_y, ...
    target_unload_id, highlight_enable, target_package_on_belt4, ...
    handoff_x, handoff_y, handoff_z, handoff_yaw, ...
    package_on_platform_from_floor, platform_pkg_x, platform_pkg_y, platform_pkg_z, platform_pkg_yaw, ...
    current_unload_id, normal_color, target_color, target_list, target_count, belt4_reverse_progress, belt4_reverse_phase);

[pkg4_T, pkg4_R, pkg4_S, pkg4_Color] = makePkg(floor2_visual_id(4), floor2_visual_path(4), visible4, ...
    belt3_right_x, belt3_center_y, belt2_center_x, belt2_end_y, ...
    belt4_center_x, belt4_max_y, ...
    slot_pitch, stopper_center_gap, belt2_entry_gap, max_on_belt3, ...
    pkg_center_z, hide_z, pkg_scale, side_y, ...
    target_unload_id, highlight_enable, target_package_on_belt4, ...
    handoff_x, handoff_y, handoff_z, handoff_yaw, ...
    package_on_platform_from_floor, platform_pkg_x, platform_pkg_y, platform_pkg_z, platform_pkg_yaw, ...
    current_unload_id, normal_color, target_color, target_list, target_count, belt4_reverse_progress, belt4_reverse_phase);

[pkg5_T, pkg5_R, pkg5_S, pkg5_Color] = makePkg(floor2_visual_id(5), floor2_visual_path(5), visible5, ...
    belt3_right_x, belt3_center_y, belt2_center_x, belt2_end_y, ...
    belt4_center_x, belt4_max_y, ...
    slot_pitch, stopper_center_gap, belt2_entry_gap, max_on_belt3, ...
    pkg_center_z, hide_z, pkg_scale, side_y, ...
    target_unload_id, highlight_enable, target_package_on_belt4, ...
    handoff_x, handoff_y, handoff_z, handoff_yaw, ...
    package_on_platform_from_floor, platform_pkg_x, platform_pkg_y, platform_pkg_z, platform_pkg_yaw, ...
    current_unload_id, normal_color, target_color, target_list, target_count, belt4_reverse_progress, belt4_reverse_phase);

[pkg6_T, pkg6_R, pkg6_S, pkg6_Color] = makePkg(floor2_visual_id(6), floor2_visual_path(6), visible6, ...
    belt3_right_x, belt3_center_y, belt2_center_x, belt2_end_y, ...
    belt4_center_x, belt4_max_y, ...
    slot_pitch, stopper_center_gap, belt2_entry_gap, max_on_belt3, ...
    pkg_center_z, hide_z, pkg_scale, side_y, ...
    target_unload_id, highlight_enable, target_package_on_belt4, ...
    handoff_x, handoff_y, handoff_z, handoff_yaw, ...
    package_on_platform_from_floor, platform_pkg_x, platform_pkg_y, platform_pkg_z, platform_pkg_yaw, ...
    current_unload_id, normal_color, target_color, target_list, target_count, belt4_reverse_progress, belt4_reverse_phase);

end


function [T, R, S, C] = makePkg(package_id, visual_path, visible, ...
                                 belt3_right_x, belt3_y, ...
                                 belt2_x, belt2_entry_y, ...
                                 belt4_center_x, belt4_max_y, ...
                                 slot_pitch, stopper_center_gap, belt2_entry_gap, ...
                                 max_on_belt3, pkg_center_z, hide_z, pkg_scale, side_y, ...
                                 target_unload_id, highlight_enable, target_package_on_belt4, ...
                                 handoff_x, handoff_y, handoff_z, handoff_yaw, ...
                                 package_on_platform_from_floor, platform_pkg_x, platform_pkg_y, platform_pkg_z, platform_pkg_yaw, ...
                                 current_unload_id, normal_color, target_color, target_list, target_count, belt4_reverse_progress, belt4_reverse_phase);

C = normal_color;

% ?곸감 ?꾨즺 ?꾩뿉留?target_list ?덉쓽 ?섏감 ????꾩껜瑜??몃??됱쑝濡??쒖떆
if highlight_enable > 0.5
    if isUnloadTarget(package_id, target_list, target_count) > 0.5
        C = target_color;
    end
end

% ?뚮옯???꾩뿉 ?щ씪媛??꾩옱 ?섏감 ??곷룄 ?몃????좎?
if package_on_platform_from_floor > 0.5 && package_id == current_unload_id && package_id > 0
    C = target_color;
end

if visible > 0.5 && package_id > 0

    % 1?쒖쐞: Belt4 ??쉶?꾩쑝濡??꾩옱 ?섏감???諛뺤뒪瑜??뚮옯?쇨퉴吏 諛湲?    if belt4_reverse_phase == 1 && belt4_reverse_progress > 0 && package_id == current_unload_id && package_id > 0

        p = belt4_reverse_progress;

        if p < 0
            p = 0;
        end

        if p > 1
            p = 1;
        end

        push_to_x = platform_pkg_x;

        % 諛⑺뼢??諛섎?硫?-瑜?+濡?諛붽퓭????        push_to_y = handoff_y - side_y * 0.40;

        push_to_z = platform_pkg_z;

        x = handoff_x + p * (push_to_x - handoff_x);
        y = handoff_y + p * (push_to_y - handoff_y);
        z = handoff_z + p * (push_to_z - handoff_z);

        T = [x, y, z];
        R = [0, 0, platform_pkg_yaw];
        S = pkg_scale;
        C = target_color;
        return;
    end

    % 2?쒖쐞: ?뚮옯???꾩뿉 ?꾩쟾???щ씪媛??곹깭
    if package_on_platform_from_floor > 0.5 && package_id == current_unload_id && package_id > 0
        T = [platform_pkg_x, platform_pkg_y, platform_pkg_z];
        R = [0, 0, platform_pkg_yaw];
        S = pkg_scale;
        C = target_color;
        return;
    end

    % 3?쒖쐞: visual_path ?꾩튂
    [x, y, yaw] = getContinuousPose(visual_path, ...
        belt3_right_x, belt3_y, ...
        belt2_x, belt2_entry_y, ...
        belt4_center_x, belt4_max_y, ...
        slot_pitch, stopper_center_gap, belt2_entry_gap, ...
        max_on_belt3, side_y, ...
        handoff_x, handoff_y, handoff_yaw);

    T = [x, y, pkg_center_z];
    R = [0, 0, yaw];
    S = pkg_scale;

else
    T = [0, 0, hide_z];
    R = [0, 0, 0];
    S = pkg_scale;
end

end


function [x, y, yaw] = getContinuousPose(path_pos, ...
                                         belt3_right_x, belt3_y, ...
                                         belt2_x, belt2_entry_y, ...
                                         belt4_center_x, belt4_max_y, ...
                                         slot_pitch, stopper_center_gap, belt2_entry_gap, ...
                                         max_on_belt3, side_y, ...
                                         handoff_x, handoff_y, handoff_yaw)
% getContinuousPose
%
% path_pos 湲곗?:
% path_pos = 4 : 湲곗〈 slot4 ?꾩튂
% path_pos = 3 : 湲곗〈 slot3 ?꾩튂
% path_pos = 2 : 湲곗〈 slot2 ?꾩튂
% path_pos = 1 : 湲곗〈 slot1 ?꾩튂
% path_pos = 0 : ?섏감怨듦컙
% path_pos < 0 : ?섏감怨듦컙蹂대떎 ?욎そ Belt4 ??
upper_idx = ceil(path_pos);
lower_idx = floor(path_pos);

if upper_idx == lower_idx
    [x, y, yaw] = getIntegerPose(upper_idx, ...
        belt3_right_x, belt3_y, ...
        belt2_x, belt2_entry_y, ...
        belt4_center_x, belt4_max_y, ...
        slot_pitch, stopper_center_gap, belt2_entry_gap, ...
        max_on_belt3, side_y, ...
        handoff_x, handoff_y, handoff_yaw);
else
    alpha = upper_idx - path_pos;

    if alpha < 0
        alpha = 0;
    end

    if alpha > 1
        alpha = 1;
    end

    [x_upper, y_upper, yaw_upper] = getIntegerPose(upper_idx, ...
        belt3_right_x, belt3_y, ...
        belt2_x, belt2_entry_y, ...
        belt4_center_x, belt4_max_y, ...
        slot_pitch, stopper_center_gap, belt2_entry_gap, ...
        max_on_belt3, side_y, ...
        handoff_x, handoff_y, handoff_yaw);

    [x_lower, y_lower, yaw_lower] = getIntegerPose(lower_idx, ...
        belt3_right_x, belt3_y, ...
        belt2_x, belt2_entry_y, ...
        belt4_center_x, belt4_max_y, ...
        slot_pitch, stopper_center_gap, belt2_entry_gap, ...
        max_on_belt3, side_y, ...
        handoff_x, handoff_y, handoff_yaw);

    x = x_upper + alpha * (x_lower - x_upper);
    y = y_upper + alpha * (y_lower - y_upper);

    if alpha < 0.5
        yaw = yaw_upper;
    else
        yaw = yaw_lower;
    end
end

end


function [x, y, yaw] = getIntegerPose(idx, ...
                                      belt3_right_x, belt3_y, ...
                                      belt2_x, belt2_entry_y, ...
                                      belt4_center_x, belt4_max_y, ...
                                      slot_pitch, stopper_center_gap, belt2_entry_gap, ...
                                      max_on_belt3, side_y, ...
                                      handoff_x, handoff_y, handoff_yaw)
% getIntegerPose
%
% idx >= 1:
%   湲곗〈 Belt3/Belt2 ?щ’ ?꾩튂
%
% idx == 0:
%   ?섏감怨듦컙
%
% idx < 0:
%   ?섏감怨듦컙蹂대떎 ???욎そ??Belt4 ?꾩튂

if idx <= 0

    x = belt4_center_x;
    y = handoff_y + side_y * (-idx) * slot_pitch;

    if side_y > 0
        if y > belt4_max_y
            y = belt4_max_y;
        end
    else
        if y < belt4_max_y
            y = belt4_max_y;
        end
    end

    yaw = handoff_yaw;

elseif idx <= max_on_belt3

    d = stopper_center_gap + slot_pitch * (idx - 1);

    x = belt3_right_x - d;
    y = belt3_y;

    yaw = 0;

else

    k = idx - max_on_belt3;

    x = belt2_x;
    y = belt2_entry_y + side_y * (belt2_entry_gap + slot_pitch * (k - 1));

    yaw = pi / 2;

end

end

function flag = isUnloadTarget(package_id, target_list, target_count)
% isUnloadTarget
%
% target_list ?덉뿉 package_id媛 ?덉쑝硫?1, ?놁쑝硫?0 諛섑솚
%
% target_list  [20x1]
% target_count [1x1]

flag = 0;

if package_id <= 0
    return;
end

n = target_count;

if n < 0
    n = 0;
end

if n > 20
    n = 20;
end

for i = 1:20
    if i <= n
        if target_list(i) == package_id
            flag = 1;
        end
    end
end

end