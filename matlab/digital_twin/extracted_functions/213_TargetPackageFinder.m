function [target_found, target_unload_floor, target_package_index] = TargetPackageFinder(target_package_id, floor1_queue, floor2_queue, floor3_queue)
% TargetPackageFinder
% target_package_id媛 媛?痢?queue 以??대뵒???덈뒗吏 李얜뒗??
%
% target_found:
%   0 = 紐?李얠쓬
%   1 = 李얠쓬
%
% target_unload_floor:
%   0 = 紐?李얠쓬
%   1 = 1痢?%   2 = 2痢?%   3 = 3痢?%
% target_package_index:
%   ?대떦 痢?queue ?덉뿉??紐?踰덉㎏?몄?
%   ?? floor2_queue = [1 2 4 0 ...]?먯꽌 target_package_id=4?대㈃ index=3

target_found = 0;
target_unload_floor = 0;
target_package_index = 0;

QUEUE_MAX = 50;

% 1痢?寃??for i = 1:QUEUE_MAX
    if floor1_queue(i) == target_package_id
        target_found = 1;
        target_unload_floor = 1;
        target_package_index = i;
        return;
    end
end

% 2痢?寃??for i = 1:QUEUE_MAX
    if floor2_queue(i) == target_package_id
        target_found = 1;
        target_unload_floor = 2;
        target_package_index = i;
        return;
    end
end

% 3痢?寃??for i = 1:QUEUE_MAX
    if floor3_queue(i) == target_package_id
        target_found = 1;
        target_unload_floor = 3;
        target_package_index = i;
        return;
    end
end