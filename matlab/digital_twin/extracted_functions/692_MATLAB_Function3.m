function [visual_count_f1, visual_count_f2, visual_count_f3, visual_count_selected] = ...
    FloorVisualArrivalCounterFunction(moving_arrive_done, moving_arrive_floor, current_target_floor)
% FloorVisualArrivalCounterFunction
%
% 紐⑹쟻:
% - MovingPackage媛 ?ㅼ젣 ?꾩갑?덉쓣 ??痢듬퀎 visual_count 利앷?
% - moving_arrive_done ?꾩뒪瑜?湲곗??쇰줈 移댁슫??% - moving_arrive_floor媛 ?뺤긽媛?1,2,3)?대㈃ 洹?媛믪쓣 ?ъ슜
% - moving_arrive_floor媛 0?대㈃ current_target_floor瑜?fallback?쇰줈 ?ъ슜
%
% ?낅젰:
% moving_arrive_done   : MovingPackage ?꾩갑 ?꾩뒪
% moving_arrive_floor  : MovingPackage媛 latch???꾩갑 痢?% current_target_floor : ?꾩옱 紐⑺몴痢?%
% 異쒕젰:
% visual_count_f1
% visual_count_f2
% visual_count_f3
% visual_count_selected

persistent count_f1 count_f2 count_f3 prev_arrive

if isempty(count_f1)
    count_f1 = 0;
    count_f2 = 0;
    count_f3 = 0;
    prev_arrive = 0;
end

arrive_now = moving_arrive_done > 0.5;
arrive_rising = arrive_now && (prev_arrive <= 0.5);

if arrive_rising

    % ?곗꽑 MovingPackage媛 ?뚮젮以 ?꾩갑 痢듭쓣 ?ъ슜
    arrived_floor = moving_arrive_floor;

    % moving_arrive_floor媛 0 ?먮뒗 鍮꾩젙?곴컪?대㈃ ?꾩옱 target_floor濡??泥?    if arrived_floor < 1 || arrived_floor > 3
        arrived_floor = current_target_floor;
    end

    if arrived_floor == 1
        count_f1 = count_f1 + 1;

    elseif arrived_floor == 2
        count_f2 = count_f2 + 1;

    elseif arrived_floor == 3
        count_f3 = count_f3 + 1;
    end
end

prev_arrive = double(arrive_now);

visual_count_f1 = count_f1;
visual_count_f2 = count_f2;
visual_count_f3 = count_f3;

if current_target_floor == 1
    visual_count_selected = count_f1;
elseif current_target_floor == 2
    visual_count_selected = count_f2;
elseif current_target_floor == 3
    visual_count_selected = count_f3;
else
    visual_count_selected = 0;
end

end