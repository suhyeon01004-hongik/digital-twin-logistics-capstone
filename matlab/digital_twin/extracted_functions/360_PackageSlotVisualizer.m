function [floor1_slot_id, floor2_slot_id, floor3_slot_id] = ...
    PackageSlotVisualizer(floor1_queue, floor2_queue, floor3_queue, ...
                          sensor_count_f1, sensor_count_f2, sensor_count_f3)
% PackageSlotVisualizer
%
% 紐⑹쟻:
% - FloorStorageUpdate????λ맂 媛?痢?queue瑜?湲곕컲?쇰줈,
%   3D ?쒖떆??slot_id 諛곗뿴??留뚮뱺??
%
% ?듭떖:
% - ?ㅼ젣 queue ?먯껜瑜??뚯쟾?쒗궎吏 ?딅뒗??
% - ?섏감 以鍮?以?而⑤쿋?댁뼱媛 ?뚯쟾??留뚰겮留??쒓컖?붿슜 slot_id瑜??뚯쟾?쒕떎.
% - 媛?痢듭? ?낅┰?곸쑝濡??뚯쟾?쒕떎.
%
% ?낅젰:
% floor1_queue      [50x1]
% floor2_queue      [50x1]
% floor3_queue      [50x1]
% sensor_count_f1   [1x1]
% sensor_count_f2   [1x1]
% sensor_count_f3   [1x1]
%
% 異쒕젰:
% floor1_slot_id    [50x1]
% floor2_slot_id    [50x1]
% floor3_slot_id    [50x1]
%
% ??
% floor2_queue = [1 2 3 4 5 6 0 ...]
% sensor_count_f2 = 3
% floor2_slot_id = [4 5 6 1 2 3 0 ...]

QUEUE_MAX = 50;

floor1_slot_id = zeros(QUEUE_MAX, 1);
floor2_slot_id = zeros(QUEUE_MAX, 1);
floor3_slot_id = zeros(QUEUE_MAX, 1);

floor1_slot_id = rotateQueueForDisplay(floor1_queue, sensor_count_f1, QUEUE_MAX);
floor2_slot_id = rotateQueueForDisplay(floor2_queue, sensor_count_f2, QUEUE_MAX);
floor3_slot_id = rotateQueueForDisplay(floor3_queue, sensor_count_f3, QUEUE_MAX);

end


function slot_id = rotateQueueForDisplay(queue, sensor_count, queue_max)
% rotateQueueForDisplay
%
% queue ?덉쓽 ?좏슚 package_id留?異붿텧????
% sensor_count留뚰겮 ?욎뿉???ㅻ줈 ?뚯쟾?쒗궓 ?쒖떆??諛곗뿴??留뚮뱺??
%
% queue ?먯껜??諛붽씀吏 ?딅뒗??

slot_id = zeros(queue_max, 1);

% =========================
% ?좏슚 package 異붿텧
% =========================
valid = zeros(queue_max, 1);
n = 0;

for i = 1:queue_max
    if queue(i) > 0
        n = n + 1;
        valid(n) = queue(i);
    end
end

if n <= 0
    return;
end

% =========================
% sensor_count ?뺣━
% =========================
shift = int32(sensor_count);

if shift < 0
    shift = int32(0);
end

% n媛?湲곗??쇰줈 ?뚯쟾
% ?? n=6, shift=6?대㈃ ?먮옒 ?쒖꽌濡?蹂듦?
while shift >= n
    shift = shift - n;
end

% =========================
% ?뚯쟾 ?쒖떆
% =========================
% sensor_count = 0:
%   [1 2 3 4 5 6]
%
% sensor_count = 1:
%   [2 3 4 5 6 1]
%
% sensor_count = 3:
%   [4 5 6 1 2 3]
for out_idx = 1:n
    src_idx = out_idx + shift;

    while src_idx > n
        src_idx = src_idx - n;
    end

    slot_id(out_idx) = valid(src_idx);
end

end