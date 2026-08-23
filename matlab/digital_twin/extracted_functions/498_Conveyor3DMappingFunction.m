function [belt1_f1_T, belt2_f1_T, belt3_f1_T, belt4_f1_T, ...
          belt1_f2_T, belt2_f2_T, belt3_f2_T, belt4_f2_T, ...
          belt1_f3_T, belt2_f3_T, belt3_f3_T, belt4_f3_T, ...
          belt1_S, belt2_S, belt3_S, belt4_S, ...
          belt_R] = Conveyor3DMappingFunction_v3(platform_y)
% Conveyor3DMappingFunction_v3
% 3痢??쒗솚 而⑤쿋?댁뼱 4媛?踰⑦듃??3D 醫뚰몴 ?앹꽦
%
% 1痢듭뿉???뺤젙???뺤긽??z 諛⑺뼢?쇰줈 蹂듭젣?쒕떎.
%
% 吏꾪뻾 諛⑺뼢:
% Belt3: +x 諛⑺뼢
% Belt4: side_y 諛⑺뼢
% Belt1: -x 諛⑺뼢
% Belt2: Belt1 ?앹뿉??Belt3 履쎌쑝濡?蹂듦?
%
% 湲몄씠:
% Belt2, Belt4 = 1.11 m
% Belt1, Belt3 = 0.51 m
% ??= 0.25 m
%
% 以묒슂:
% belt4_platform_extend = 0.25濡??뺤젙

% =========================
% 湲곕낯 移섏닔
% =========================
belt_width = 0.25;      % 250 mm
belt_thick = 0.015;

L1 = 0.51;              % Belt1 length
L2 = 1.11;              % Belt2 湲곗? length
L3 = 0.51;              % Belt3 length
L4 = 1.11;              % Belt4 湲곗? length

% =========================
% 怨좎젙???ㅽ넗??Belt3 湲곗?
% =========================
stopper_x = -0.15;
stopper_y = platform_y + 0.30;
stopper_thick_x = 0.03;

% Belt3 ?앹젏 A
A_x = stopper_x - stopper_thick_x/2;
A_y = stopper_y;

% =========================
% 諛⑺뼢 ?ㅼ젙
% =========================
side_y = 1;
side_x = 1;

% =========================
% 痢??믪씠
% =========================
z_floor1 = 0.00;
z_floor2 = 0.27;
z_floor3 = 0.54;

% ?뚮옯??踰⑦듃 ?쀫㈃ 湲곗?
% 媛?痢듭뿉??踰⑦듃 ?쀫㈃???대떦 痢?z_floor? 媛숇룄濡??ㅼ젙
belt_center_z_f1 = z_floor1 - belt_thick/2;
belt_center_z_f2 = z_floor2 - belt_thick/2;
belt_center_z_f3 = z_floor3 - belt_thick/2;

% =========================================================
% Belt3: 湲곗? 踰⑦듃, ?꾩튂 怨좎젙
% 吏꾪뻾 諛⑺뼢 +x, ?ㅻⅨ履??앹씠 A??% =========================================================
belt3_right_x = A_x;
belt3_left_x  = belt3_right_x - L3;

belt3_center_x = (belt3_left_x + belt3_right_x) / 2;
belt3_center_y = A_y;

belt3_connect_edge_y = belt3_center_y + side_y * belt_width/2;

% =========================================================
% Belt4
% =========================================================
if side_x > 0
    belt4_left_x  = A_x;
    belt4_right_x = A_x + belt_width;
else
    belt4_left_x  = A_x - belt_width;
    belt4_right_x = A_x;
end

belt4_center_x = (belt4_left_x + belt4_right_x) / 2;

belt4_start_y_nominal = belt3_connect_edge_y;

% ?뺤젙媛?belt4_platform_extend = 0.25;

belt4_start_y = belt4_start_y_nominal - side_y * belt4_platform_extend;
belt4_end_y   = belt4_start_y_nominal + side_y * L4;

belt4_center_y = (belt4_start_y + belt4_end_y) / 2;
L4_actual = abs(belt4_end_y - belt4_start_y);

if side_x > 0
    belt4_outer_x = belt4_right_x;
else
    belt4_outer_x = belt4_left_x;
end

belt4_end_edge_y = belt4_end_y;

% =========================================================
% Belt1
% =========================================================
belt1_right_x = belt4_outer_x;
belt1_left_x  = belt1_right_x - L1;

belt1_center_x = (belt1_left_x + belt1_right_x) / 2;

belt1_inner_edge_y = belt4_end_edge_y;
belt1_center_y = belt1_inner_edge_y + side_y * belt_width/2;

belt1_outer_edge_y = belt1_inner_edge_y + side_y * belt_width;

% =========================================================
% Belt2
% =========================================================
belt2_right_x = belt1_left_x;
belt2_left_x  = belt2_right_x - belt_width;
belt2_center_x = (belt2_left_x + belt2_right_x) / 2;

belt2_start_y = belt1_outer_edge_y;
belt2_end_y   = belt3_connect_edge_y;
belt2_center_y = (belt2_start_y + belt2_end_y) / 2;

L2_actual = abs(belt2_start_y - belt2_end_y);

% =========================
% 1痢?Translation
% =========================
belt1_f1_T = [belt1_center_x, belt1_center_y, belt_center_z_f1];
belt2_f1_T = [belt2_center_x, belt2_center_y, belt_center_z_f1];
belt3_f1_T = [belt3_center_x, belt3_center_y, belt_center_z_f1];
belt4_f1_T = [belt4_center_x, belt4_center_y, belt_center_z_f1];

% =========================
% 2痢?Translation
% =========================
belt1_f2_T = [belt1_center_x, belt1_center_y, belt_center_z_f2];
belt2_f2_T = [belt2_center_x, belt2_center_y, belt_center_z_f2];
belt3_f2_T = [belt3_center_x, belt3_center_y, belt_center_z_f2];
belt4_f2_T = [belt4_center_x, belt4_center_y, belt_center_z_f2];

% =========================
% 3痢?Translation
% =========================
belt1_f3_T = [belt1_center_x, belt1_center_y, belt_center_z_f3];
belt2_f3_T = [belt2_center_x, belt2_center_y, belt_center_z_f3];
belt3_f3_T = [belt3_center_x, belt3_center_y, belt_center_z_f3];
belt4_f3_T = [belt4_center_x, belt4_center_y, belt_center_z_f3];

% =========================
% Scale
% 紐⑤뱺 痢듭뿉???숈씪?섍쾶 ?ъ슜
% =========================
belt1_S = [L1, belt_width, belt_thick];
belt2_S = [belt_width, L2_actual, belt_thick];
belt3_S = [L3, belt_width, belt_thick];
belt4_S = [belt_width, L4_actual, belt_thick];

belt_R = [0, 0, 0];