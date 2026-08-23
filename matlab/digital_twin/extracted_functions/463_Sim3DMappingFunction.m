function [platform_x, platform_y, platform_z_3d, platform_translation, platform_rotation, platform_scale, ...
    rotator_x, rotator_y, rotator_z, rotator_yaw, rotator_translation, rotator_rotation, rotator_scale, ...
    pusher_x, pusher_y, pusher_z, pusher_translation, pusher_rotation, pusher_scale, ...
    stopper1_translation, stopper2_translation, stopper3_translation, stopper_rotation, stopper_scale] = ...
    Sim3DMappingFunction(platform_z, rotator_angle, pusher_pos, stopper_pos, platform_fold_angle, mission_phase)
% Sim3DMappingFunction
% Simulink ?쒖뼱 ?좏샇瑜?3D 媛앹껜 ?꾩튂/?뚯쟾 ?좏샇濡?蹂?섑븳??
%
% ?낅젰:
% platform_z          : ?ㅼ젣 ?뚮옯???믪씠
% rotator_angle       : ?뚮옯?????뚯쟾??yaw 媛곷룄
% pusher_pos          : ?몄뀛 ?꾩쭊 ?꾩튂
% stopper_pos         : ?ㅽ넗???꾩튂, 1=?몄?, 0=?묓옒
% platform_fold_angle : ?뚮옯???묓옒 媛곷룄, 0=?쇱묠, pi/2=?묓옒
% mission_phase       : 0=?곸감, 1=?섏감 以鍮??섏감, 2=誘몄뀡 ?꾨즺

% =========================
% 湲곗? ?꾩튂
% =========================
platform_base_x = 0.0;
platform_base_y = -0.35;

rotator_offset_x = 0.0;
rotator_offset_y = 0.0;
rotator_offset_z = 0.02;

pusher_offset_y_home = -0.16;
pusher_offset_z = 0.055;

% =========================
% 痢??믪씠
% =========================
z_floor1 = 0.00;
z_floor2 = 0.27;
z_floor3 = 0.54;

% =========================
% ?뚮옯???꾩튂
% =========================
platform_x = platform_base_x;
platform_y = platform_base_y;
platform_z_visual_offset = -0.02;
platform_z_3d = platform_z + platform_z_visual_offset;

platform_translation = [platform_x, platform_y, platform_z_3d];

% platform_fold_angle:
% 0      = ?쇱묠
% pi/2   = ?묓옒
%
% ?묓엳??異뺤씠 ?댁긽?섎㈃ ?ш린留?諛붽씀硫???
platform_rotation = [platform_fold_angle, 0, 0];

platform_scale = [1, 1, 1];

% 嫄곗쓽 ?쇱퀜吏??곹깭?몄? ?먮떒
platform_deployed = 0;
if platform_fold_angle < 0.05
    platform_deployed = 1;
end

% =========================
% ?뚯쟾???꾩튂/?뚯쟾
% =========================
rotator_x = platform_x + rotator_offset_x;
rotator_y = platform_y + rotator_offset_y;
rotator_z = platform_z_3d + rotator_offset_z;
rotator_yaw = rotator_angle;

if platform_deployed > 0.5
    rotator_translation = [rotator_x, rotator_y, rotator_z];
    rotator_rotation    = [0, 0, rotator_yaw];
    rotator_scale       = [1, 1, 1];
else
    % ?뚮옯?쇱씠 ?묓엺 ?곹깭?먯꽌???뚯쟾???④?
    rotator_translation = [rotator_x, rotator_y, -10.0];
    rotator_rotation    = [0, 0, 0];
    rotator_scale       = [1, 1, 1];
end

% =========================
% ?몄뀛 ?꾩튂
% =========================
pusher_x = platform_x;
pusher_y = platform_y + pusher_offset_y_home + pusher_pos;
pusher_z = platform_z_3d + pusher_offset_z;

if platform_deployed > 0.5
    pusher_translation = [pusher_x, pusher_y, pusher_z];
    pusher_rotation    = [0, 0, 0];
    pusher_scale       = [1, 1, 1];
else
    % ?뚮옯?쇱씠 ?묓엺 ?곹깭?먯꽌???몄뀛 ?④?
    pusher_translation = [pusher_x, pusher_y, -10.0];
    pusher_rotation    = [0, 0, 0];
    pusher_scale       = [1, 1, 1];
end

% =========================
% 痢듬퀎 ?묒씠???ㅽ넗???꾩튂/?뚯쟾
% =========================
stopper_base_x = -0.15;
stopper_base_y = platform_y + 0.30;

stopper_z_offset_on_belt = 0.05;

stopper1_x = stopper_base_x;
stopper1_y = stopper_base_y;
stopper1_z = z_floor1 + stopper_z_offset_on_belt;

stopper2_x = stopper_base_x;
stopper2_y = stopper_base_y;
stopper2_z = z_floor2 + stopper_z_offset_on_belt;

stopper3_x = stopper_base_x;
stopper3_y = stopper_base_y;
stopper3_z = z_floor3 + stopper_z_offset_on_belt;

if mission_phase == 0
    % ?곸감 以? ?ㅽ넗???쒖떆
    stopper1_translation = [stopper1_x, stopper1_y, stopper1_z];
    stopper2_translation = [stopper2_x, stopper2_y, stopper2_z];
    stopper3_translation = [stopper3_x, stopper3_y, stopper3_z];

    stopper_fold_angle = (pi/2) * (1 - stopper_pos);
    stopper_rotation = [0, stopper_fold_angle, 0];

    stopper_scale = [1, 1, 1];

else
    % ?섏감 以鍮??섏감/?꾨즺 以? ?ㅽ넗???④?
    hide_z = -10.0;

    stopper1_translation = [stopper1_x, stopper1_y, hide_z];
    stopper2_translation = [stopper2_x, stopper2_y, hide_z];
    stopper3_translation = [stopper3_x, stopper3_y, hide_z];

    stopper_rotation = [0, pi/2, 0];
    stopper_scale = [1, 1, 1];
end

end