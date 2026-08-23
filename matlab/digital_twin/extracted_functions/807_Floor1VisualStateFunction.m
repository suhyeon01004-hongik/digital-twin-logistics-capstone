function [floor1_visual_id, floor1_visual_path, floor1_visual_count, belt4_reverse_progress, belt4_reverse_phase] = ...
    Floor1VisualStateFunction(floor1_queue, visual_shift_f1, target_id_f1, target_on_belt4_f1, ...
                              current_unload_id, package_on_platform_from_floor, unload_done, ...
                              belt4_reverse_cmd, belt4_restore_cmd)
% Floor1VisualStateFunction
%
% 紐⑹쟻:
% - 1痢?諛뺤뒪?ㅼ쓽 ?쒓컖??踰⑦듃 ?꾩튂瑜??곕줈 ??ν븳??
% - floor1_queue???쇰━ queue?대?濡??섏감 ?꾨즺 ???뺤텞?????덈떎.
% - visual state???ㅼ젣 踰⑦듃 ???꾩튂瑜??좎??쒕떎.
%
% ?듭떖:
% - queue???녿떎???댁쑀濡?visual state?먯꽌 ?쒓굅?섏? ?딅뒗??
% - ?ㅼ젣 ?섏감 ?꾨즺??carried_id留?unload_done ?곸듅?ｌ??먯꽌 ??젣?쒕떎.
% - Belt4 reverse/restore??1痢??섏감 ??곸씪 ?뚮쭔 ?숈옉?쒕떎.
% - reverse_progress??persistent濡??좎??섏뼱???섎ŉ 留??ㅽ뀦 珥덇린?뷀븯硫????쒕떎.

MAX_VISUAL = 5;
QUEUE_MAX = 50;

persistent ids
persistent paths
persistent prev_shift
persistent initialized
persistent carried_id
persistent prev_unload_done

persistent reverse_progress
persistent reverse_phase
persistent prev_reverse_cmd
persistent prev_restore_cmd

if isempty(initialized)
    ids = zeros(MAX_VISUAL, 1);
    paths = zeros(MAX_VISUAL, 1);
    prev_shift = 0;
    initialized = 1;

    carried_id = 0;
    prev_unload_done = 0;

    reverse_progress = 0;
    reverse_phase = 0;
    prev_reverse_cmd = 0;
    prev_restore_cmd = 0;
end

% =========================
% 湲곕낯 異쒕젰 珥덇린??% =========================
floor1_visual_id = zeros(MAX_VISUAL, 1);
floor1_visual_path = zeros(MAX_VISUAL, 1);
floor1_visual_count = 0;

belt4_reverse_progress = reverse_progress;
belt4_reverse_phase = reverse_phase;

% =========================
% 1. queue???덈줈 ?앷릿 package瑜?visual state???깅줉
% =========================
for q = 1:QUEUE_MAX
    pkg_id = floor1_queue(q);

    if pkg_id > 0
        exists = 0;

        for k = 1:MAX_VISUAL
            if ids(k) == pkg_id
                exists = 1;
            end
        end

        if exists < 0.5
            free_idx = 0;

            for k = 1:MAX_VISUAL
                if ids(k) <= 0 && free_idx == 0
                    free_idx = k;
                end
            end

            if free_idx > 0
                ids(free_idx) = pkg_id;
                paths(free_idx) = q;
            end
        end
    end
end

% =========================
% 2. queue?먯꽌 ?щ씪吏?package ?쒓굅 湲덉?
% =========================
% floor1_queue???쇰━ queue?대?濡??섏감 ?꾨즺 ???뺤텞?????덈떎.
% visual state???ㅼ젣 踰⑦듃 ???꾩튂瑜??쒗쁽?섎?濡?
% queue???녿떎???댁쑀留뚯쑝濡???젣?섏? ?딅뒗??

% =========================
% 3. ?뚮옯???꾩뿉 ?щ씪媛?package ID 湲곗뼲
% =========================
if package_on_platform_from_floor > 0.5 && current_unload_id > 0
    carried_id = current_unload_id;
end

% =========================
% 4. unload_done ?곸듅?ｌ??먯꽌 carried_id ?쒓굅
% =========================
unload_done_rise = 0;

if unload_done > 0.5 && prev_unload_done <= 0.5
    unload_done_rise = 1;
end

if unload_done_rise > 0.5 && carried_id > 0
    for k = 1:MAX_VISUAL
        if ids(k) == carried_id
            ids(k) = 0;
            paths(k) = 0;
        end
    end

    carried_id = 0;
end

prev_unload_done = unload_done;

% =========================
% 5. visual_shift 蹂?붾웾?쇰줈 紐⑤뱺 諛뺤뒪 ?대룞
% =========================
delta = visual_shift_f1 - prev_shift;

% target ?꾪솚 ??visual_shift媛 由ъ뀑?섎㈃ ?ㅻ줈 ?뚯븘媛吏 ?딄쾶 ?쒕떎.
if delta < 0
    delta = 0;
    prev_shift = visual_shift_f1;
end

% ?덈Т ???먰봽 諛⑹?
if delta > 1
    delta = 1;
end

% target???섏감怨듦컙??吏??怨쇰룄?섍쾶 ?대룞?섏? ?딅룄濡??쒗븳
if delta > 0
    target_path = 999;

    if target_id_f1 > 0
        for k = 1:MAX_VISUAL
            if ids(k) == target_id_f1
                target_path = paths(k);
            end
        end
    end

    if target_path <= 0
        delta = 0;
    elseif delta > target_path
        delta = target_path;
    end
end

% 以鍮??꾨즺 ?곹깭?먯꽌?????댁긽 ?섏감怨듦컙 諛⑺뼢 ?쒗솚 ?대룞???섏? ?딆쓬
if target_on_belt4_f1 > 0.5
    delta = 0;
end

if delta > 0
    for k = 1:MAX_VISUAL
        if ids(k) > 0
            paths(k) = paths(k) - delta;
        end
    end
end

if visual_shift_f1 >= prev_shift
    prev_shift = visual_shift_f1;
end

% =========================
% 6. Belt4 reverse/restore ?숈옉
% =========================
% reverse_progress = 0 : Belt4 ?먯쐞移?% reverse_progress = 1 : ?섏감 ???諛뺤뒪媛 ?뚮옯???꾨줈 異⑸텇??諛由??곹깭
%
% 以묒슂:
% - belt4_reverse_cmd????痢?怨듯넻 ?좏샇?????덉쑝誘濡?
%   1痢??섏감 ??곸씪 ?뚮쭔 local_reverse_cmd瑜??덉슜?쒕떎.
% - restore??reverse_progress媛 ?ㅼ젣濡?0蹂대떎 ???뚮쭔 ?덉슜?쒕떎.
% - reverse_cmd媛 吏㏐쾶 ?ㅼ뼱???reverse_phase媛 latch?섏뼱 1源뚯? 吏꾪뻾?쒕떎.

Ts_reverse = 0.05;
reverse_speed = 0.30;
reverse_step = reverse_speed * Ts_reverse;

is_current_floor1_unload = 0;

if target_id_f1 > 0 && current_unload_id > 0
    if target_id_f1 == current_unload_id
        is_current_floor1_unload = 1;
    end
end

local_reverse_cmd = 0;
local_restore_cmd = 0;

% 1痢???곸씠 ?섏감怨듦컙???덇퀬, ?꾩옱 ?섏감 ??곷룄 1痢?target???뚮쭔 reverse ?쒖옉
if belt4_reverse_cmd > 0.5 && is_current_floor1_unload > 0.5 && target_on_belt4_f1 > 0.5
    local_reverse_cmd = 1;
end

% reverse_progress媛 ?ㅼ젣濡?吏꾪뻾??痢듬쭔 restore ?덉슜
if belt4_restore_cmd > 0.5 && reverse_progress > 0
    local_restore_cmd = 1;
end

reverse_cmd_rise = 0;
restore_cmd_rise = 0;

if local_reverse_cmd > 0.5 && prev_reverse_cmd <= 0.5
    reverse_cmd_rise = 1;
end

if local_restore_cmd > 0.5 && prev_restore_cmd <= 0.5
    restore_cmd_rise = 1;
end

prev_reverse_cmd = local_reverse_cmd;
prev_restore_cmd = local_restore_cmd;

if reverse_cmd_rise > 0.5
    reverse_phase = 1;
end

if restore_cmd_rise > 0.5
    reverse_phase = 2;
end

if reverse_phase == 1

    reverse_progress = reverse_progress + reverse_step;

    if reverse_progress >= 1
        reverse_progress = 1;
        reverse_phase = 0;
    end

elseif reverse_phase == 2

    reverse_progress = reverse_progress - reverse_step;

    if reverse_progress <= 0
        reverse_progress = 0;
        reverse_phase = 0;
    end

end

belt4_reverse_progress = reverse_progress;
belt4_reverse_phase = reverse_phase;

% =========================
% 7. 異쒕젰 ?뺣━
% =========================
count = 0;

% ?섏감 ??곸씠 ?꾨땶 Belt4 ??諛뺤뒪?ㅼ쓽 ??쉶???쒖떆 ?대룞??% ?덈Т ?ш쾶 二쇰㈃ path媛 0???섏뼱 Belt3 ?꾩튂濡??섏뼱媛 蹂댁씤??
NON_TARGET_REVERSE_TRAVEL = 1.5;

for k = 1:MAX_VISUAL
    floor1_visual_id(k) = ids(k);

    display_path = paths(k);

    if reverse_progress > 0
        if ids(k) > 0 && paths(k) <= 0

            % ?꾩옱 ?섏감 ??곸? F1 makePkg?먯꽌 handoff -> platform?쇰줈 蹂꾨룄 ?쒖떆?쒕떎.
            % ?ш린?쒕뒗 ?섏감 ??곸씠 ?꾨땶 Belt4 ??諛뺤뒪留??댁쭩 諛?덈떎媛 蹂듦??쒗궓??
            if ids(k) ~= current_unload_id
                display_path = paths(k) + reverse_progress * NON_TARGET_REVERSE_TRAVEL;

                % path > 0???섎㈃ Belt3 ?щ’?쇰줈 ?댁꽍?섎?濡?諛⑹??쒕떎.
                if display_path > -0.05
                    display_path = -0.05;
                end
            end
        end
    end

    floor1_visual_path(k) = display_path;

    if ids(k) > 0
        count = count + 1;
    end
end

floor1_visual_count = count;

end