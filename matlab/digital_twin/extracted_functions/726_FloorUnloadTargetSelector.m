function [target_id_f1, target_index_f1, found_f1, ...
          target_id_f2, target_index_f2, found_f2, ...
          target_id_f3, target_index_f3, found_f3] = ...
          FloorUnloadTargetSelector(target_list, target_count, floor1_queue, floor2_queue, floor3_queue)
% FloorUnloadTargetSelector
%
% 紐⑹쟻:
% - ?꾩껜 ?섏감 ???由ъ뒪??target_list)瑜?蹂닿퀬,
%   媛?痢듭뿉??癒쇱? 以鍮꾪빐?????섏감 ???package_id? queue index瑜?李얜뒗??
%
% ?듭떖 媛쒕뀗:
% - ?섏감 以鍮꾨뒗 痢듬퀎濡??낅┰ 媛?ν븯??
% - 媛?痢?而⑤쿋?댁뼱??媛쒕퀎濡??吏곸씪 ???덉쑝誘濡?
%   1痢? 2痢? 3痢?媛곴컖 ?섎굹???섏감 ??곸? ?숈떆???섏감怨듦컙?쇰줈 誘몃━ 蹂대궪 ???덈떎.
% - ?? 媛숈? 痢듭뿉 ?섏감 ??곸씠 ?щ윭 媛??덉쑝硫?洹?痢듭뿉?쒕뒗 target_list ?쒖꽌??媛???욎꽑 寃??섎굹留??좏깮?쒕떎.
%
% ?낅젰:
% target_list   [20x1] : ?섏감 ???package_id 紐⑸줉
% target_count  [1x1]  : ?ㅼ젣 ?섏감 ???媛쒖닔
% floor1_queue  [50x1] : 1痢??곸옱 queue
% floor2_queue  [50x1] : 2痢??곸옱 queue
% floor3_queue  [50x1] : 3痢??곸옱 queue
%
% 異쒕젰:
% target_id_fN     : ?대떦 痢듭뿉??癒쇱? 以鍮꾪븷 package_id
% target_index_fN  : ?대떦 package_id媛 ?꾩옱 floorN_queue?먯꽌 紐?踰덉㎏?몄?
% found_fN         : ?대떦 痢듭뿉 以鍮???곸씠 ?덉쑝硫?1, ?놁쑝硫?0

MAX_TARGETS = 20;
QUEUE_MAX = 50;

% =========================
% 湲곕낯 異쒕젰
% =========================
target_id_f1 = 0;
target_index_f1 = 0;
found_f1 = 0;

target_id_f2 = 0;
target_index_f2 = 0;
found_f2 = 0;

target_id_f3 = 0;
target_index_f3 = 0;
found_f3 = 0;

% =========================
% target_count 蹂댁젙
% =========================
count = int32(target_count);

if count < 0
    count = int32(0);
end

if count > MAX_TARGETS
    count = int32(MAX_TARGETS);
end

if count == 0
    return;
end

% =========================
% target_list ?쒖꽌?濡??먯깋
% =========================
for t = 1:count
    target_id = target_list(t);

    if target_id <= 0
        continue;
    end

    % -------------------------
    % 1痢??먯깋
    % -------------------------
    if found_f1 < 0.5
        idx1 = findPackageIndex(floor1_queue, target_id, QUEUE_MAX);

        if idx1 > 0
            target_id_f1 = target_id;
            target_index_f1 = idx1;
            found_f1 = 1;
        end
    end

    % -------------------------
    % 2痢??먯깋
    % -------------------------
    if found_f2 < 0.5
        idx2 = findPackageIndex(floor2_queue, target_id, QUEUE_MAX);

        if idx2 > 0
            target_id_f2 = target_id;
            target_index_f2 = idx2;
            found_f2 = 1;
        end
    end

    % -------------------------
    % 3痢??먯깋
    % -------------------------
    if found_f3 < 0.5
        idx3 = findPackageIndex(floor3_queue, target_id, QUEUE_MAX);

        if idx3 > 0
            target_id_f3 = target_id;
            target_index_f3 = idx3;
            found_f3 = 1;
        end
    end

    % ??痢?紐⑤몢 以鍮???곸씠 ?뺥빐議뚯쑝硫???蹂??꾩슂 ?놁쓬
    if found_f1 > 0.5 && found_f2 > 0.5 && found_f3 > 0.5
        break;
    end
end

end


function idx = findPackageIndex(queue, target_id, queue_max)
% queue ?덉뿉??target_id???꾩튂瑜?李얜뒗??
% ?놁쑝硫?0 諛섑솚.

idx = 0;

for i = 1:queue_max
    if queue(i) == target_id
        idx = i;
        return;
    end
end

end