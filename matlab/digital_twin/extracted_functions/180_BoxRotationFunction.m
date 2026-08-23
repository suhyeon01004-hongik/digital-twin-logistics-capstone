function [box_insert_length, desired_box_yaw, rotator_target_angle] = BoxRotationFunction(box_length, box_width, box_yaw)
% BoxRotationFunction
% ?쒕뜡?섍쾶 ?볦씤 ?앸같 yaw瑜?湲곗??쇰줈,
% 而⑤쿋?댁뼱 吏꾪뻾諛⑺뼢 ?먯쑀 湲몄씠媛 理쒖냼媛 ?섎룄濡??꾩슂???뚯쟾媛곸쓣 怨꾩궛?쒕떎.
%
% ?낅젰:
%   box_length : ?앸같 湲?諛⑺뼢 湲몄씠 [m]
%   box_width  : ?앸같 吏㏃? 諛⑺뼢 湲몄씠 [m]
%   box_yaw    : 移대찓??YOLO媛 ?몄떇???꾩옱 ?앸같 yaw [deg]
%
% 異쒕젰:
%   box_insert_length     : ?뚯쟾 ??而⑤쿋?댁뼱 吏꾪뻾諛⑺뼢?쇰줈 李⑥??섎뒗 湲몄씠 [m]
%   desired_box_yaw       : 紐⑺몴 ?앸같 yaw [deg]
%   rotator_target_angle  : ?뚯쟾?먯씠 ?뚯쟾?댁빞 ?섎뒗 蹂댁젙媛?[deg]

% ?곸옱 ?⑥쑉???꾪빐 吏㏃? 蹂??吏꾪뻾諛⑺뼢???ν븯?꾨줉 ?쒕떎.
% ?ш린?쒕뒗 ?⑥닚??
% - box_length <= box_width?대㈃ ?꾩옱 0 deg 諛⑺뼢???좊━
% - box_length > box_width?대㈃ 90 deg 諛⑺뼢???좊━
%
% ?ㅼ젣濡쒕뒗 移대찓??醫뚰몴怨꾩? 而⑤쿋?댁뼱 吏꾪뻾諛⑺뼢 ?뺤쓽???곕씪
% desired_box_yaw 湲곗???議곗젙?댁빞 ?쒕떎.

if box_length <= box_width
    box_insert_length = box_length;
    desired_box_yaw = 0;
else
    box_insert_length = box_width;
    desired_box_yaw = 90;
end

% ?꾩옱 yaw?먯꽌 紐⑺몴 yaw源뚯? ?꾩슂???뚯쟾媛?raw_angle = desired_box_yaw - box_yaw;

% ?뚯쟾媛곸쓣 -90 ~ 90 deg 踰붿쐞濡??뺢퇋??rotator_target_angle = normalizeToMinus90To90(raw_angle);


function angle_out = normalizeToMinus90To90(angle_in)
% angle_in??-90 ~ 90 踰붿쐞濡??뺢퇋??
angle_out = angle_in;

while angle_out > 90
    angle_out = angle_out - 180;
end

while angle_out < -90
    angle_out = angle_out + 180;
end