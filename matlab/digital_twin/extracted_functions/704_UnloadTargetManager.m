function [target_package_id, current_target_index, unload_all_done] = ...
    UnloadTargetManager(target_list, target_count, unload_done, mission_phase)
% UnloadTargetManager
%
% 紐⑹쟻:
% - ?щ윭 媛쒖쓽 ?섏감 ???package_id瑜??쒖감?곸쑝濡??좏깮?쒕떎.
% - unload_done??諛쒖깮?섎㈃ ?ㅼ쓬 ?섏감 ??곸쑝濡??섏뼱媛꾨떎.
% - 紐⑤뱺 ????섏감媛 ?앸굹硫?unload_all_done = 1??異쒕젰?쒕떎.
%
% ?낅젰:
% target_list   : ?섏감 ???package_id 紐⑸줉, ?? [4; 7; 0; 0; ...]
% target_count  : ?ㅼ젣 ?섏감??媛쒖닔
% unload_done   : UnloadController???섏감 ?꾨즺 ?좏샇
% mission_phase : MissionModeFunction???꾩옱 ?④퀎
%
% 異쒕젰:
% target_package_id     : ?꾩옱 ?섏감 ???package_id
% current_target_index  : ?꾩옱 紐?踰덉㎏ ?섏감 ??곸씤吏
% unload_all_done       : 紐⑤뱺 ?섏감 ????꾨즺 ?щ?

persistent idx
persistent prev_unload_done

MAX_TARGETS = 20;

if isempty(idx)
    idx = int32(1);
    prev_unload_done = 0;
end

% 湲곕낯 異쒕젰
target_package_id = 0;
current_target_index = double(idx);
unload_all_done = 0;

% target_count ?뺣━
count = int32(target_count);

if count < 0
    count = int32(0);
end

if count > MAX_TARGETS
    count = int32(MAX_TARGETS);
end

% ?섏감 ?④퀎媛 ?꾨땲硫?泥?踰덉㎏ ??곸쑝濡??湲?if mission_phase < 1
    idx = int32(1);
    prev_unload_done = unload_done;

    if count >= 1
        target_package_id = target_list(1);
    else
        target_package_id = 0;
    end

    current_target_index = double(idx);
    unload_all_done = 0;
    return;
end

% ?섏감????곸씠 ?놁쑝硫??꾨즺
if count == 0
    target_package_id = 0;
    current_target_index = 0;
    unload_all_done = 1;
    prev_unload_done = unload_done;
    return;
end

% idx 踰붿쐞 蹂댁젙
if idx < 1
    idx = int32(1);
end

if idx > count
    target_package_id = 0;
    current_target_index = double(idx);
    unload_all_done = 1;
    prev_unload_done = unload_done;
    return;
end

% ?꾩옱 target 異쒕젰
% 以묒슂:
% unload_done??媛숈? step???ㅼ뼱?????step?먯꽌???꾩쭅 ?꾩옱 target??異쒕젰?쒕떎.
% 洹몃옒??FloorStorageUpdate媛 ?꾩옱 target??蹂닿퀬 ?뺥솗????젣?????덈떎.
target_package_id = target_list(idx);
current_target_index = double(idx);
unload_all_done = 0;

% unload_done ?곸듅 ?먯? 媛먯?
unload_rising = (prev_unload_done == 0) && (unload_done == 1);

% ?섏감 ?꾨즺?섎㈃ ?ㅼ쓬 step遺???ㅼ쓬 target?쇰줈 ?섏뼱媛?꾨줉 idx 利앷?
if unload_rising
    idx = idx + 1;

    if idx > count
        unload_all_done = 1;
    end
end

prev_unload_done = unload_done;

end