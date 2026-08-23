function current_target_on_belt4 = CurrentUnloadReadySelector(target_unload_floor, target_on_belt4_f1, target_on_belt4_f2, target_on_belt4_f3)
% CurrentUnloadReadySelector
%
% ?꾩옱 ?ㅼ젣濡??섏감??target???대뒓 痢듭뿉 ?덈뒗吏 蹂닿퀬,
% 洹?痢듭쓽 以鍮??꾨즺 ?좏샇留?UnloadController濡??꾨떖?쒕떎.

current_target_on_belt4 = 0;

if target_unload_floor == 1
    current_target_on_belt4 = target_on_belt4_f1;

elseif target_unload_floor == 2
    current_target_on_belt4 = target_on_belt4_f2;

elseif target_unload_floor == 3
    current_target_on_belt4 = target_on_belt4_f3;

else
    current_target_on_belt4 = 0;
end

end