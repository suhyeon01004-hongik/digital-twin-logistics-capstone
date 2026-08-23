function [loading_enable, unload_request_auto, total_loaded_count, mission_phase] = MissionModeFunction(floor1_count, floor2_count, floor3_count, load_target_count, unload_done)
% MissionModeFunction
%
% mission_phase:
% 0 = ?곸감 ?④퀎
% 1 = ?섏감 以鍮??섏감 ?④퀎
% 2 = ?꾩껜 誘몄뀡 ?꾨즺

persistent phase
persistent prev_unload_done

if isempty(phase)
    phase = 0;
    prev_unload_done = 0;
end

total_loaded_count = floor1_count + floor2_count + floor3_count;

unload_rising_edge = (prev_unload_done == 0) && (unload_done == 1);

% ?곸감 ?꾨즺 ???섏감 ?④퀎
if phase == 0
    if total_loaded_count >= load_target_count
        phase = 1;
    end
end

% 紐⑤뱺 ?섏감 ????꾨즺 ??誘몄뀡 ?꾨즺
if phase == 1
    if unload_rising_edge
        phase = 2;
    end
end

prev_unload_done = unload_done;

if phase == 0
    loading_enable = 1;
    unload_request_auto = 0;

elseif phase == 1
    loading_enable = 0;
    unload_request_auto = 1;

else
    loading_enable = 0;
    unload_request_auto = 0;
end

mission_phase = phase;

end