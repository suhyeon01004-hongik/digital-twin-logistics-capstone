function [vis_platform_z, vis_rotator_angle, vis_pusher_pos, vis_stopper_pos, ...
    vis_mission_phase, vis_load_state, vis_unload_state, ...
    vis_lift_cmd, ...
    vis_floor1_conv_cmd, vis_floor2_conv_cmd, vis_floor3_conv_cmd, ...
    vis_floor1_belt4_cmd, vis_floor2_belt4_cmd, vis_floor3_belt4_cmd, ...
    vis_floor1_x, vis_floor1_y, vis_floor1_z, vis_floor1_yaw, vis_floor1_visible, vis_floor1_color_flag, ...
    vis_floor2_x, vis_floor2_y, vis_floor2_z, vis_floor2_yaw, vis_floor2_visible, vis_floor2_color_flag, ...
    vis_floor3_x, vis_floor3_y, vis_floor3_z, vis_floor3_yaw, vis_floor3_visible, vis_floor3_color_flag] = ...
    VisualizationPacketFunction(mission_phase, state_id, unload_state_id, ...
    platform_z, rotator_angle, pusher_pos, stopper_pos, ...
    final_lift_cmd, ...
    final_floor1_conv_cmd, final_floor2_conv_cmd, final_floor3_conv_cmd, ...
    final_floor1_belt4_cmd, final_floor2_belt4_cmd, final_floor3_belt4_cmd, ...
    floor1_x, floor1_y, floor1_z, floor1_yaw, floor1_visible, floor1_color_flag, ...
    floor2_x, floor2_y, floor2_z, floor2_yaw, floor2_visible, floor2_color_flag, ...
    floor3_x, floor3_y, floor3_z, floor3_yaw, floor3_visible, floor3_color_flag)
% VisualizationPacketFunction
% 3D ?쒓컖???곕룞??理쒖쥌 異쒕젰 ?⑦궥 ?뺣━ ?⑥닔
%
% ?꾩옱 ?④퀎?먯꽌???낅젰 ?좏샇瑜?3D ?쒓컖?붿슜 ?대쫫?쇰줈 ?뺣━?댁꽌 洹몃?濡?異쒕젰?쒕떎.
% ?섏쨷??Unity UDP/TCP ?≪떊, Simulink 3D Animation, App Designer ?깆쑝濡??곌껐?????덈떎.

% =========================
% Actuator visualization
% =========================
vis_platform_z = platform_z;
vis_rotator_angle = rotator_angle;
vis_pusher_pos = pusher_pos;
vis_stopper_pos = stopper_pos;

% =========================
% Mission/state visualization
% =========================
vis_mission_phase = mission_phase;
vis_load_state = state_id;
vis_unload_state = unload_state_id;

% =========================
% Final command visualization
% =========================
vis_lift_cmd = final_lift_cmd;

vis_floor1_conv_cmd = final_floor1_conv_cmd;
vis_floor2_conv_cmd = final_floor2_conv_cmd;
vis_floor3_conv_cmd = final_floor3_conv_cmd;

vis_floor1_belt4_cmd = final_floor1_belt4_cmd;
vis_floor2_belt4_cmd = final_floor2_belt4_cmd;
vis_floor3_belt4_cmd = final_floor3_belt4_cmd;

% =========================
% Package visualization
% =========================
vis_floor1_x = floor1_x;
vis_floor1_y = floor1_y;
vis_floor1_z = floor1_z;
vis_floor1_yaw = floor1_yaw;
vis_floor1_visible = floor1_visible;
vis_floor1_color_flag = floor1_color_flag;

vis_floor2_x = floor2_x;
vis_floor2_y = floor2_y;
vis_floor2_z = floor2_z;
vis_floor2_yaw = floor2_yaw;
vis_floor2_visible = floor2_visible;
vis_floor2_color_flag = floor2_color_flag;

vis_floor3_x = floor3_x;
vis_floor3_y = floor3_y;
vis_floor3_z = floor3_z;
vis_floor3_yaw = floor3_yaw;
vis_floor3_visible = floor3_visible;
vis_floor3_color_flag = floor3_color_flag;
