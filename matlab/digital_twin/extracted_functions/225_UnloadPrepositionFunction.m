function [preposition_active, sensor_count, target_package_on_belt4, unload_floor1_conv_cmd, unload_floor2_conv_cmd, unload_floor3_conv_cmd] = UnloadPrepositionFunction(unload_request, vehicle_stopped, target_found, target_unload_floor, target_package_index)
% UnloadPrepositionFunction
%
% 諛곗넚 以??섏감 以鍮??④퀎 紐⑤뜽
%
% 紐⑹쟻:
% - ?곸감 ?꾨즺 ??unload_request媛 1???섎㈃ ?섏감 以鍮?媛???곹깭媛 ?쒕떎.
% - 李⑤웾???뺤감 以?vehicle_stopped == 1)???뚮쭔 ???痢?而⑤쿋?댁뼱瑜??쒗솚?쒗궓??
% - 紐⑤뱺 ?앸같???쒖꽌瑜??좎???梨??④퍡 ?쒗솚?쒕떎怨?媛?뺥븳??
% - target_package_index ?꾩튂???앸같媛 slot 1, 利?Belt4 ?곷떒/?뚮옯?????섏감怨듦컙???ㅻ㈃
%   target_package_on_belt4 = 1 濡?留뚮뱺??
%
% ?듭떖:
% - target_package_index = 1?대㈃ ?대? ?섏감怨듦컙???덈뒗 寃껋쑝濡??먮떒?쒕떎.
% - target_package_index = 4?대㈃ 3移??쒗솚?섎㈃ ?섏감怨듦컙???⑤떎.
%
% 異쒕젰:
% preposition_active       : ?섏감 以鍮??쒗솚 以묒씠硫?1
% sensor_count             : ?꾩옱源뚯? ?쒗솚??移???% target_package_on_belt4  : ????앸같媛 ?섏감怨듦컙??以鍮꾨릺硫?1
% unload_floorN_conv_cmd   : ???痢??쒗솚 而⑤쿋?댁뼱 援щ룞 紐낅졊

persistent count
persistent active
persistent done
persistent step_counter
persistent prev_unload_request
persistent last_target_floor
persistent last_target_index

% 紐?怨꾩궛 step留덈떎 ?앸같 1媛쒓? ?쇱꽌/?섏감怨듦컙 湲곗??쇰줈 ?대룞?덈떎怨?蹂쇱?
% 媛믪씠 ?묒쓣?섎줉 ?쒗솚 以鍮꾧? 鍮좊Ⅴ寃?蹂댁엫
COUNT_INTERVAL = 20;

if isempty(count)
    count = 0;
    active = 0;
    done = 0;
    step_counter = 0;
    prev_unload_request = 0;
    last_target_floor = 0;
    last_target_index = 0;
end

% 湲곕낯 異쒕젰
preposition_active = active;
sensor_count = count;
target_package_on_belt4 = done;

unload_floor1_conv_cmd = 0;
unload_floor2_conv_cmd = 0;
unload_floor3_conv_cmd = 0;

% ?섏감 ?붿껌 ?곸듅 ?먯? 媛먯?
request_rising = (prev_unload_request == 0) && (unload_request == 1);

% ??곸씠 諛붾뚯뿀?붿????뺤씤
target_changed = (target_unload_floor ~= last_target_floor) || ...
                 (target_package_index ~= last_target_index);

% ???섏감 ?붿껌???ㅼ뼱?ㅺ굅????곸씠 諛붾뚮㈃ 珥덇린??if request_rising || target_changed
    count = 0;
    active = 0;
    done = 0;
    step_counter = 0;

    last_target_floor = target_unload_floor;
    last_target_index = target_package_index;
end

prev_unload_request = unload_request;

% ?좏슚?섏? ?딆? 議곌굔?대㈃ ?뺤?
if unload_request < 0.5 || target_found < 0.5 || target_unload_floor < 1 || target_unload_floor > 3 || target_package_index < 1
    active = 0;
    done = 0;
    count = 0;
    step_counter = 0;

    preposition_active = active;
    sensor_count = count;
    target_package_on_belt4 = done;
    return;
end

% target_package_index媛 1?대㈃ ?대? Belt4 ?곷떒/?뚮옯?????섏감怨듦컙???덈떎怨?蹂몃떎.
required_shift = target_package_index - 1;

if required_shift < 0
    required_shift = 0;
end

% ?대? ?꾩슂??留뚰겮 ?쒗솚?덉쑝硫?以鍮??꾨즺
if count >= required_shift
    active = 0;
    done = 1;

    unload_floor1_conv_cmd = 0;
    unload_floor2_conv_cmd = 0;
    unload_floor3_conv_cmd = 0;

    preposition_active = active;
    sensor_count = count;
    target_package_on_belt4 = done;
    return;
end

% ?꾩쭅 以鍮꾧? ???앸궗怨? 李⑤웾???뺤감 以묒씠硫??쒗솚 ?쒖옉/?좎?
if vehicle_stopped > 0.5
    active = 1;
else
    % 諛곗넚 以묒씠吏留??뺤감 ?곹깭媛 ?꾨땲硫??꾩옱 ?꾩튂?먯꽌 硫덉땄
    active = 0;
end

% ?섏감 以鍮??숈옉
if active == 1 && done == 0

    % ???痢?而⑤쿋?댁뼱留??쒗솚
    if target_unload_floor == 1
        unload_floor1_conv_cmd = 1;
        unload_floor2_conv_cmd = 0;
        unload_floor3_conv_cmd = 0;

    elseif target_unload_floor == 2
        unload_floor1_conv_cmd = 0;
        unload_floor2_conv_cmd = 1;
        unload_floor3_conv_cmd = 0;

    elseif target_unload_floor == 3
        unload_floor1_conv_cmd = 0;
        unload_floor2_conv_cmd = 0;
        unload_floor3_conv_cmd = 1;
    end

    % 媛???쇱꽌/?쒗솚 移댁슫??    step_counter = step_counter + 1;

    if step_counter >= COUNT_INTERVAL
        step_counter = 0;
        count = count + 1;
    end

    % ?꾩슂???쒗솚 ?섏뿉 ?꾨떖?섎㈃ 以鍮??꾨즺
    if count >= required_shift
        done = 1;
        active = 0;

        unload_floor1_conv_cmd = 0;
        unload_floor2_conv_cmd = 0;
        unload_floor3_conv_cmd = 0;
    end
end

preposition_active = active;
sensor_count = count;
target_package_on_belt4 = done;

end