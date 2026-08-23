function [target_floor, storage_available, floor1_remaining_length, floor2_remaining_length, floor3_remaining_length] = TargetFloorSelector(box_insert_length, floor1_used_length, floor2_used_length, floor3_used_length, floor_capacity_length)
% TargetFloorSelector
% 媛?痢듭쓽 ?⑥? ?곸옱 湲몄씠瑜?怨꾩궛?섍퀬,
% box_insert_length媛 ?ㅼ뼱媛????덈뒗 痢?以?best-fit 諛⑹떇?쇰줈 紐⑺몴痢듭쓣 ?좏깮?쒕떎.
%
% best-fit:
% ?ㅼ뼱媛????덈뒗 痢?以묒뿉???ｊ퀬 ?????⑤뒗 湲몄씠媛 媛???묒? 痢??좏깮

floor1_remaining_length = floor_capacity_length - floor1_used_length;
floor2_remaining_length = floor_capacity_length - floor2_used_length;
floor3_remaining_length = floor_capacity_length - floor3_used_length;

storage_available = 0;
target_floor = 0;

best_leftover = 1.0e9;

% 1痢?寃??if box_insert_length <= floor1_remaining_length
    leftover = floor1_remaining_length - box_insert_length;
    if leftover < best_leftover
        best_leftover = leftover;
        target_floor = 1;
        storage_available = 1;
    end
end

% 2痢?寃??if box_insert_length <= floor2_remaining_length
    leftover = floor2_remaining_length - box_insert_length;
    if leftover < best_leftover
        best_leftover = leftover;
        target_floor = 2;
        storage_available = 1;
    end
end

% 3痢?寃??if box_insert_length <= floor3_remaining_length
    leftover = floor3_remaining_length - box_insert_length;
    if leftover < best_leftover
        best_leftover = leftover;
        target_floor = 3;
        storage_available = 1;
    end
end

% ?대뵒?먮룄 紐??ｌ쑝硫?target_floor = 0
if storage_available == 0
    target_floor = 0;
end