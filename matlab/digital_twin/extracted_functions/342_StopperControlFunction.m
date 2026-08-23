function [stopper_vel, stopper_down_done, stopper_up_done] = StopperControlFunction(stopper_cmd, stopper_pos)
% StopperControlFunction
% ?ㅽ넗???꾩튂 紐⑤뜽
%
% stopper_cmd:
%   0 = ?ㅽ넗???щ┝
%   1 = ?ㅽ넗???대┝
%
% stopper_pos:
%   0 = ?꾩쟾???щ씪媛??곹깭
%   1 = ?꾩쟾???대젮媛??곹깭
%
% stopper_vel:
%   ?ㅽ넗???꾩튂 蹂???띾룄 [1/s]
%
% stopper_down_done:
%   ?ㅽ넗?쇨? ?꾩쟾???대젮媛붾뒗吏
%
% stopper_up_done:
%   ?ㅽ넗?쇨? ?꾩쟾???щ씪媛붾뒗吏

stopper_speed = 2.0;   % 1/s, 0?먯꽌 1源뚯? ??0.5珥?pos_tol = 0.01;

stopper_vel = 0;
stopper_down_done = 0;
stopper_up_done = 0;

if stopper_cmd == 1
    % ?ㅽ넗???대┝
    if stopper_pos >= 1.0 - pos_tol
        stopper_vel = 0;
        stopper_down_done = 1;
        stopper_up_done = 0;
    else
        stopper_vel = stopper_speed;
        stopper_down_done = 0;
        stopper_up_done = 0;
    end

else
    % ?ㅽ넗???щ┝
    if stopper_pos <= pos_tol
        stopper_vel = 0;
        stopper_down_done = 0;
        stopper_up_done = 1;
    else
        stopper_vel = -stopper_speed;
        stopper_down_done = 0;
        stopper_up_done = 0;
    end
end