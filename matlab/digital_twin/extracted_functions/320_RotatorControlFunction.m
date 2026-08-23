function [rotator_vel, rotation_done, rotator_home_done] = RotatorControlFunction(rotator_cmd, rotator_target_angle, rotator_angle)
% RotatorControlFunction
% rotator_cmd???곕씪 ?뚯쟾?먯쓣 ?쒖뼱?쒕떎.
%
% rotator_cmd:
%   0 = ?뺤?
%   1 = ?앸같 yaw 蹂댁젙 紐⑺몴媛?rotator_target_angle)?쇰줈 ?뚯쟾
%   2 = ?뚯쟾???먯젏 0 deg濡?蹂듦?
%
% rotation_done:
%   rotator_cmd == 1????紐⑺몴 蹂댁젙媛??꾨떖 ?щ?
%
% rotator_home_done:
%   rotator_cmd == 2????0??蹂듦? ?꾨즺 ?щ?

rotator_speed = 60;   % deg/s
angle_tol = 5.0;      % deg

rotator_vel = 0;
rotation_done = 0;
rotator_home_done = 0;

if rotator_cmd == 1
    target_angle = rotator_target_angle;
    error = target_angle - rotator_angle;

    if abs(error) <= angle_tol
        rotator_vel = 0;
        rotation_done = 1;
        rotator_home_done = 0;
    else
        if error > 0
            rotator_vel = rotator_speed;
        else
            rotator_vel = -rotator_speed;
        end
        rotation_done = 0;
        rotator_home_done = 0;
    end

elseif rotator_cmd == 2
    target_angle = 0;
    error = target_angle - rotator_angle;

    if abs(error) <= angle_tol
        rotator_vel = 0;
        rotation_done = 0;
        rotator_home_done = 1;
    else
        if error > 0
            rotator_vel = rotator_speed;
        else
            rotator_vel = -rotator_speed;
        end
        rotation_done = 0;
        rotator_home_done = 0;
    end

else
    rotator_vel = 0;
    rotation_done = 0;
    rotator_home_done = 0;
end