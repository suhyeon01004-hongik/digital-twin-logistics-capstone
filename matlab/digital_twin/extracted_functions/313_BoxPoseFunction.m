function box_yaw = BoxPoseFunction(load_done)
% BoxPoseFunction
% ?앸같媛 ?뚮옯???꾩뿉 ?쒕뜡?섍쾶 ?볦??ㅺ퀬 媛?뺥븯怨?
% 移대찓??YOLO媛 ?몄떇??珥덇린 yaw 媛곷룄瑜??앹꽦?쒕떎.
%
% box_yaw ?⑥쐞: deg
% 踰붿쐞: -90 deg ~ 90 deg
%
% load_done??0 -> 1濡?諛붾뚮㈃ ?ㅼ쓬 ?앸같??yaw瑜??덈줈 ?앹꽦?쒕떎.

persistent current_yaw
persistent prev_load_done

if isempty(current_yaw)
    current_yaw = -90 + 180 * rand;
    prev_load_done = 0;
end

rising_edge = (prev_load_done == 0) && (load_done == 1);

if rising_edge
    current_yaw = -90 + 180 * rand;
end

prev_load_done = load_done;

box_yaw = current_yaw;