function [floor1_visible, floor1_color_flag, ...
          floor2_visible, floor2_color_flag, ...
          floor3_visible, floor3_color_flag] = ...
          PackageSlotToXYZFunction(floor1_slot_id, floor2_slot_id, floor3_slot_id, ...
                                   target_id_f1, target_on_belt4_f1, ...
                                   target_id_f2, target_on_belt4_f2, ...
                                   target_id_f3, target_on_belt4_f3)
% PackageSlotToXYZFunction
%
% 紐⑹쟻:
% - PackageSlotVisualizer?먯꽌 留뚮뱺 floorN_slot_id瑜?諛쏆븘
%   媛?痢듭쓽 ?쒖떆 ?щ? visible怨??됱긽 flag瑜?留뚮뱺??
%
% ?낅젰:
% floor1_slot_id      [50x1]
% floor2_slot_id      [50x1]
% floor3_slot_id      [50x1]
%
% target_id_f1        [1x1]
% target_on_belt4_f1  [1x1]
% target_id_f2        [1x1]
% target_on_belt4_f2  [1x1]
% target_id_f3        [1x1]
% target_on_belt4_f3  [1x1]
%
% 異쒕젰:
% floorN_visible      [50x1]
% floorN_color_flag   [50x1]
%
% 洹쒖튃:
% - slot_id > 0 ?대㈃ visible = 1
% - ?대떦 slot_id媛 洹?痢듭쓽 target_id_fN?닿퀬 target_on_belt4_fN = 1?대㈃ color_flag = 1
% - 洹??몄뿉??color_flag = 0
%
% 二쇱쓽:
% - floorN_highlight 媛숈? [50x1] 諛곗뿴 ?낅젰?????댁긽 ?ъ슜?섏? ?딅뒗??
% - target_on_belt4_fN? scalar?대?濡?floorN_highlight(i)泥섎읆 ?몃뜳?깊븯硫????쒕떎.

QUEUE_MAX = 50;

floor1_visible = zeros(QUEUE_MAX, 1);
floor2_visible = zeros(QUEUE_MAX, 1);
floor3_visible = zeros(QUEUE_MAX, 1);

floor1_color_flag = zeros(QUEUE_MAX, 1);
floor2_color_flag = zeros(QUEUE_MAX, 1);
floor3_color_flag = zeros(QUEUE_MAX, 1);

% =========================
% 1痢?% =========================
for i = 1:QUEUE_MAX
    pkg_id = floor1_slot_id(i);

    if pkg_id > 0
        floor1_visible(i) = 1;
    else
        floor1_visible(i) = 0;
    end

    if target_on_belt4_f1 > 0.5 && target_id_f1 > 0 && pkg_id == target_id_f1
        floor1_color_flag(i) = 1;
    else
        floor1_color_flag(i) = 0;
    end
end

% =========================
% 2痢?% =========================
for i = 1:QUEUE_MAX
    pkg_id = floor2_slot_id(i);

    if pkg_id > 0
        floor2_visible(i) = 1;
    else
        floor2_visible(i) = 0;
    end

    if target_on_belt4_f2 > 0.5 && target_id_f2 > 0 && pkg_id == target_id_f2
        floor2_color_flag(i) = 1;
    else
        floor2_color_flag(i) = 0;
    end
end

% =========================
% 3痢?% =========================
for i = 1:QUEUE_MAX
    pkg_id = floor3_slot_id(i);

    if pkg_id > 0
        floor3_visible(i) = 1;
    else
        floor3_visible(i) = 0;
    end

    if target_on_belt4_f3 > 0.5 && target_id_f3 > 0 && pkg_id == target_id_f3
        floor3_color_flag(i) = 1;
    else
        floor3_color_flag(i) = 0;
    end
end

end