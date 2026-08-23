function [floor1_used_length, floor2_used_length, floor3_used_length, floor1_count, floor2_count, floor3_count, floor1_queue, floor2_queue, floor3_queue, target_package_length] = FloorStorageUpdate(load_done, target_floor, box_insert_length, package_id, unload_done, target_package_id, target_unload_floor)
% FloorStorageUpdate
% ?곸감 ?꾨즺 ??
%   1) ?좏깮??痢?used_length 利앷?
%   2) ?좏깮??痢?count 利앷?
%   3) ?좏깮??痢?queue ?ㅼ뿉 package_id ???%   4) package_id蹂?box_insert_length ???%
% ?섏감 ?꾨즺 ??
%   1) ???痢?queue?먯꽌 target_package_id ??젣
%   2) ???痢?count 媛먯냼
%   3) target_package_id????λ맂 湲몄씠留뚰겮 used_length 媛먯냼
%
% queue??column vector [QUEUE_MAX x 1] ?뺥깭濡??듭씪?쒕떎.

persistent f1_used f2_used f3_used
persistent f1_count f2_count f3_count
persistent q1 q2 q3
persistent package_lengths
persistent prev_load_done prev_unload_done

QUEUE_MAX = 50;
PACKAGE_MAX = 1000;

if isempty(f1_used)
    % 珥덇린 ?곸옱 湲몄씠
    f1_used = 0.40;
    f2_used = 0.45;
    f3_used = 0.20;

    % ?대쾲 ?쒕??덉씠?섏뿉???덈줈 ?곸감???앸같 媛쒖닔
    f1_count = 0;
    f2_count = 0;
    f3_count = 0;

    % 痢듬퀎 ?? column vector濡??듭씪
    q1 = zeros(QUEUE_MAX, 1);
    q2 = zeros(QUEUE_MAX, 1);
    q3 = zeros(QUEUE_MAX, 1);

    % package_id蹂??앸같 吏꾪뻾諛⑺뼢 湲몄씠 ???    package_lengths = zeros(PACKAGE_MAX, 1);

    prev_load_done = 0;
    prev_unload_done = 0;
end

% ?곸듅 ?먯? 媛먯?
load_rising_edge = (prev_load_done == 0) && (load_done == 1);
unload_rising_edge = (prev_unload_done == 0) && (unload_done == 1);

% ?꾩옱 target_package_id????λ맂 湲몄씠 議고쉶
target_idx = int32(target_package_id);
target_package_length = 0;

if target_idx >= 1 && target_idx <= PACKAGE_MAX
    target_package_length = package_lengths(target_idx);
end

% =========================
% ?곸감 ?낅뜲?댄듃
% =========================
if load_rising_edge
    pkg_idx = int32(package_id);

    % package_id蹂?湲몄씠 ???    if pkg_idx >= 1 && pkg_idx <= PACKAGE_MAX
        package_lengths(pkg_idx) = box_insert_length;
    end

    if target_floor == 1
        if f1_count < QUEUE_MAX
            f1_count = f1_count + 1;
            q1(f1_count) = package_id;
            f1_used = f1_used + box_insert_length;
        end

    elseif target_floor == 2
        if f2_count < QUEUE_MAX
            f2_count = f2_count + 1;
            q2(f2_count) = package_id;
            f2_used = f2_used + box_insert_length;
        end

    elseif target_floor == 3
        if f3_count < QUEUE_MAX
            f3_count = f3_count + 1;
            q3(f3_count) = package_id;
            f3_used = f3_used + box_insert_length;
        end
    end
end

% =========================
% ?섏감 ?낅뜲?댄듃
% =========================
if unload_rising_edge
    target_idx_for_unload = int32(target_package_id);
    removed_length = 0;

    if target_idx_for_unload >= 1 && target_idx_for_unload <= PACKAGE_MAX
        removed_length = package_lengths(target_idx_for_unload);
    end

    % ?뱀떆 湲몄씠媛 ??λ릺???덉? ?딆쑝硫??덉쟾?섍쾶 ?꾩옱 box_insert_length ?ъ슜
    if removed_length <= 0
        removed_length = box_insert_length;
    end

    if target_unload_floor == 1
        [q1, f1_count, removed] = removePackageFromQueue(q1, f1_count, target_package_id, QUEUE_MAX);

        if removed == 1
            f1_used = f1_used - removed_length;

            if f1_used < 0
                f1_used = 0;
            end

            if target_idx_for_unload >= 1 && target_idx_for_unload <= PACKAGE_MAX
                package_lengths(target_idx_for_unload) = 0;
            end
        end

    elseif target_unload_floor == 2
        [q2, f2_count, removed] = removePackageFromQueue(q2, f2_count, target_package_id, QUEUE_MAX);

        if removed == 1
            f2_used = f2_used - removed_length;

            if f2_used < 0
                f2_used = 0;
            end

            if target_idx_for_unload >= 1 && target_idx_for_unload <= PACKAGE_MAX
                package_lengths(target_idx_for_unload) = 0;
            end
        end

    elseif target_unload_floor == 3
        [q3, f3_count, removed] = removePackageFromQueue(q3, f3_count, target_package_id, QUEUE_MAX);

        if removed == 1
            f3_used = f3_used - removed_length;

            if f3_used < 0
                f3_used = 0;
            end

            if target_idx_for_unload >= 1 && target_idx_for_unload <= PACKAGE_MAX
                package_lengths(target_idx_for_unload) = 0;
            end
        end
    end
end

prev_load_done = load_done;
prev_unload_done = unload_done;

% ?섏감 ?낅뜲?댄듃 ?댄썑 target_package_length ?ㅼ떆 怨꾩궛
target_idx = int32(target_package_id);
target_package_length = 0;

if target_idx >= 1 && target_idx <= PACKAGE_MAX
    target_package_length = package_lengths(target_idx);
end

floor1_used_length = f1_used;
floor2_used_length = f2_used;
floor3_used_length = f3_used;

floor1_count = f1_count;
floor2_count = f2_count;
floor3_count = f3_count;

floor1_queue = q1;
floor2_queue = q2;
floor3_queue = q3;


function [q_out, count_out, removed] = removePackageFromQueue(q_in, count_in, target_id, queue_max)
% q_in?먯꽌 target_id瑜?李얠븘 ??젣?섍퀬, ?ㅼそ ?먯냼?ㅼ쓣 ?욎쑝濡??밴릿??

q_out = q_in;
count_out = count_in;
removed = 0;

remove_idx = 0;

for i = 1:queue_max
    if q_out(i) == target_id
        remove_idx = i;
        removed = 1;
        break;
    end
end

if removed == 1
    % ??젣 ?꾩튂遺????移몄뵫 ?욎쑝濡??밴?
    for i = remove_idx:(queue_max - 1)
        q_out(i) = q_out(i + 1);
    end

    % 留덉?留?移?鍮꾩슦湲?    q_out(queue_max) = 0;

    % count 媛먯냼
    if count_out > 0
        count_out = count_out - 1;
    end
end