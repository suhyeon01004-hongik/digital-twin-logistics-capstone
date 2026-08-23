function [unload_floor1_conv_cmd, unload_floor2_conv_cmd, unload_floor3_conv_cmd, ...
          sensor_count_f1, sensor_count_f2, sensor_count_f3, ...
          target_on_belt4_f1, target_on_belt4_f2, target_on_belt4_f3, ...
          preposition_active_f1, preposition_active_f2, preposition_active_f3, ...
          visual_shift_f1, visual_shift_f2, visual_shift_f3] = ...
          MultiFloorUnloadPrepositionFunction( ...
          mission_phase, vehicle_stopped, delivery_arrived, ...
          target_index_f1, found_f1, target_id_f1, ...
          target_index_f2, found_f2, target_id_f2, ...
          target_index_f3, found_f3, target_id_f3, ...
          unload_state_id_for_preposition)
% MultiFloorUnloadPrepositionFunction
%
% 紐⑹쟻:
% - 諛곗넚 以??뺤감 ?쒓컙??媛?痢?而⑤쿋?댁뼱瑜??낅┰?곸쑝濡??쒗솚?쒖폒
%   媛?痢듭쓽 ?섏감 ???1媛쒖뵫??Belt4 ?곷떒/?뚮옯?????섏감怨듦컙?쇰줈 誘몃━ ?대룞?쒗궓??
%
% 異붽? 紐⑹쟻:
% - 媛숈? 痢듭뿉 ?섏감 ??곸씠 ?щ윭 媛??덉쓣 ??
%   泥?踰덉㎏ ?섏감 ???ㅼ쓬 ??곷룄 ?ㅼ떆 ?섏감怨듦컙?쇰줈 以鍮꾪븷 ???덇쾶 ?쒕떎.
%
% ?대쾲 異붽?:
% - visual_shift_f1/f2/f3 異쒕젰
% - sensor_count???뺤닔 移??대룞??% - visual_shift???뺤닔 移??대룞??+ ?꾩옱 移댁슫???ъ씠??吏꾪뻾瑜?%
% ?듭떖 洹쒖튃:
% - mission_phase == 1 ???뚮쭔 ?섏감 以鍮?媛??% - vehicle_stopped == 1 ???뚮쭔 ?섏감 以鍮?媛??% - 諛곗넚 以?delivery_arrived == 0)?먮뒗 ?뺤감 以묒씠硫?以鍮?媛??% - 諛곕떖 ?μ냼 ?꾩갑 ??delivery_arrived == 1)?먮뒗 UnloadController媛 IDLE???뚮쭔 以鍮?媛??% - ?ㅼ젣 ?섏감 以?unload_state_id 1~7)?먮뒗 以鍮?而⑤쿋?댁뼱 ?뺤?
% - 媛?痢?sensor_count???낅┰?곸쑝濡?愿由?% - queue ?먯껜???뚯쟾?쒗궎吏 ?딄퀬, ?쒖떆??sensor_count/visual_shift留?利앷??쒗궡
%
% 以묒슂:
% - target_index = 1 ?대씪怨??댁꽌 利됱떆 ?섏감怨듦컙???덈떎怨?蹂댁? ?딅뒗??
% - ?ㅼ젣 ?쒖뒪?쒖쓽 移댁슫?낆? "泥?踰덉㎏ ?앸같 ?듦낵 = count 1"?대떎.
% - ?곕씪???꾩슂??count??target_index - 1 ???꾨땲??target_index ?대떎.

persistent count_f1 count_f2 count_f3
persistent step_f1 step_f2 step_f3
persistent done_f1 done_f2 done_f3
persistent prev_target_id_f1 prev_target_id_f2 prev_target_id_f3

if isempty(count_f1)
    count_f1 = 0;
    count_f2 = 0;
    count_f3 = 0;

    step_f1 = 0;
    step_f2 = 0;
    step_f3 = 0;

    done_f1 = 0;
    done_f2 = 0;
    done_f3 = 0;

    prev_target_id_f1 = 0;
    prev_target_id_f2 = 0;
    prev_target_id_f3 = 0;
end

% =========================
% 議곗젙 ?뚮씪誘명꽣
% =========================
% Sample time = 0.05 湲곗?
% COUNT_INTERVAL = 20?대㈃ ??1珥덈쭏??sensor_count媛 1 利앷?
COUNT_INTERVAL = 60;

% =========================
% 湲곕낯 異쒕젰
% =========================
unload_floor1_conv_cmd = 0;
unload_floor2_conv_cmd = 0;
unload_floor3_conv_cmd = 0;

target_on_belt4_f1 = 0;
target_on_belt4_f2 = 0;
target_on_belt4_f3 = 0;

preposition_active_f1 = 0;
preposition_active_f2 = 0;
preposition_active_f3 = 0;

% =========================
% target 蹂寃?媛먯? 諛?痢듬퀎 移댁슫??由ъ뀑
% =========================
if target_id_f1 ~= prev_target_id_f1
    count_f1 = 0;
    step_f1 = 0;
    done_f1 = 0;
    prev_target_id_f1 = target_id_f1;
end

if target_id_f2 ~= prev_target_id_f2
    count_f2 = 0;
    step_f2 = 0;
    done_f2 = 0;
    prev_target_id_f2 = target_id_f2;
end

if target_id_f3 ~= prev_target_id_f3
    count_f3 = 0;
    step_f3 = 0;
    done_f3 = 0;
    prev_target_id_f3 = target_id_f3;
end

% target???놁쑝硫??대떦 痢?以鍮??곹깭 由ъ뀑
if found_f1 < 0.5 || target_id_f1 <= 0
    count_f1 = 0;
    step_f1 = 0;
    done_f1 = 0;
end

if found_f2 < 0.5 || target_id_f2 <= 0
    count_f2 = 0;
    step_f2 = 0;
    done_f2 = 0;
end

if found_f3 < 0.5 || target_id_f3 <= 0
    count_f3 = 0;
    step_f3 = 0;
    done_f3 = 0;
end

% =========================
% ?꾩슂??sensor count 怨꾩궛
% =========================
need_f1 = target_index_f1;
need_f2 = target_index_f2;
need_f3 = target_index_f3;

if need_f1 < 0
    need_f1 = 0;
end

if need_f2 < 0
    need_f2 = 0;
end

if need_f3 < 0
    need_f3 = 0;
end

if found_f1 < 0.5 || target_id_f1 <= 0
    need_f1 = 0;
end

if found_f2 < 0.5 || target_id_f2 <= 0
    need_f2 = 0;
end

if found_f3 < 0.5 || target_id_f3 <= 0
    need_f3 = 0;
end

% =========================
% 以鍮??꾨즺 ?먯젙
% =========================
if found_f1 > 0.5 && target_id_f1 > 0
    if count_f1 >= need_f1
        done_f1 = 1;
    end
end

if found_f2 > 0.5 && target_id_f2 > 0
    if count_f2 >= need_f2
        done_f2 = 1;
    end
end

if found_f3 > 0.5 && target_id_f3 > 0
    if count_f3 >= need_f3
        done_f3 = 1;
    end
end

% =========================
% ?섏감 以鍮??덉슜 議곌굔
% =========================
% 諛곗넚 以?
%   delivery_arrived = 0
%   vehicle_stopped = 1
%   ??以鍮?媛??%
% 諛곕떖 ?μ냼 ?꾩갑 ??
%   delivery_arrived = 1
%   unload_state_id_for_preposition = 0
%   ???ㅼ젣 ?섏감 以묒씠 ?꾨땺 ?뚮쭔 ?ㅼ쓬 ???以鍮?媛??%
% ?ㅼ젣 ?섏감 以?
%   unload_state_id_for_preposition ~= 0
%   ??以鍮?而⑤쿋?댁뼱 ?뺤?
prep_allowed = 0;

if mission_phase == 1 && vehicle_stopped > 0.5

    if delivery_arrived <= 0.5
        prep_allowed = 1;

    else
        if unload_state_id_for_preposition <= 0.5
            prep_allowed = 1;
        else
            prep_allowed = 0;
        end
    end

end

% =========================
% 1痢?以鍮?% =========================
if found_f1 > 0.5 && target_id_f1 > 0 && done_f1 < 0.5
    if prep_allowed > 0.5
        preposition_active_f1 = 1;
        unload_floor1_conv_cmd = 1;

        step_f1 = step_f1 + 1;

        if step_f1 >= COUNT_INTERVAL
            step_f1 = 0;
            count_f1 = count_f1 + 1;
        end
    end
end

% =========================
% 2痢?以鍮?% =========================
if found_f2 > 0.5 && target_id_f2 > 0 && done_f2 < 0.5
    if prep_allowed > 0.5
        preposition_active_f2 = 1;
        unload_floor2_conv_cmd = 1;

        step_f2 = step_f2 + 1;

        if step_f2 >= COUNT_INTERVAL
            step_f2 = 0;
            count_f2 = count_f2 + 1;
        end
    end
end

% =========================
% 3痢?以鍮?% =========================
if found_f3 > 0.5 && target_id_f3 > 0 && done_f3 < 0.5
    if prep_allowed > 0.5
        preposition_active_f3 = 1;
        unload_floor3_conv_cmd = 1;

        step_f3 = step_f3 + 1;

        if step_f3 >= COUNT_INTERVAL
            step_f3 = 0;
            count_f3 = count_f3 + 1;
        end
    end
end

% =========================
% 以鍮??꾨즺 ?ы뙋??% =========================
if found_f1 > 0.5 && target_id_f1 > 0
    if count_f1 >= need_f1
        done_f1 = 1;
    end
end

if found_f2 > 0.5 && target_id_f2 > 0
    if count_f2 >= need_f2
        done_f2 = 1;
    end
end

if found_f3 > 0.5 && target_id_f3 > 0
    if count_f3 >= need_f3
        done_f3 = 1;
    end
end

% =========================
% 異쒕젰
% =========================
sensor_count_f1 = count_f1;
sensor_count_f2 = count_f2;
sensor_count_f3 = count_f3;

if done_f1 > 0.5 && found_f1 > 0.5 && target_id_f1 > 0
    target_on_belt4_f1 = 1;
else
    target_on_belt4_f1 = 0;
end

if done_f2 > 0.5 && found_f2 > 0.5 && target_id_f2 > 0
    target_on_belt4_f2 = 1;
else
    target_on_belt4_f2 = 0;
end

if done_f3 > 0.5 && found_f3 > 0.5 && target_id_f3 > 0
    target_on_belt4_f3 = 1;
else
    target_on_belt4_f3 = 0;
end

% =========================
% ?곗냽 ?대룞 ?쒓컖?붿슜 異쒕젰
% =========================
% done ?곹깭媛 ?꾨땲怨? ?ㅼ젣濡?以鍮?active 以묒씪 ?뚮뒗
% count + step/COUNT_INTERVAL 媛믪쓣 ?대낫?몃떎.
%
% done ?곹깭?먯꽌??step???⑥븘 ?덉뼱???뺤닔 count ?꾩튂濡?怨좎젙?쒕떎.
if done_f1 < 0.5 && preposition_active_f1 > 0.5
    visual_shift_f1 = count_f1 + step_f1 / COUNT_INTERVAL;
else
    visual_shift_f1 = count_f1;
end

if done_f2 < 0.5 && preposition_active_f2 > 0.5
    visual_shift_f2 = count_f2 + step_f2 / COUNT_INTERVAL;
else
    visual_shift_f2 = count_f2;
end

if done_f3 < 0.5 && preposition_active_f3 > 0.5
    visual_shift_f3 = count_f3 + step_f3 / COUNT_INTERVAL;
else
    visual_shift_f3 = count_f3;
end

end