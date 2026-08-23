function [final_lift_cmd, final_target_z, ...
          final_floor1_conv_cmd, final_floor2_conv_cmd, final_floor3_conv_cmd, ...
          final_floor1_belt4_cmd, final_floor2_belt4_cmd, final_floor3_belt4_cmd] = ...
          CommandMuxFunction(mission_phase, ...
          load_lift_cmd, load_target_z, load_floor1_conv_cmd, load_floor2_conv_cmd, load_floor3_conv_cmd, ...
          pre_floor1_conv_cmd, pre_floor2_conv_cmd, pre_floor3_conv_cmd, ...
          unload_lift_cmd, unload_target_z, target_unload_floor, belt4_reverse_cmd, belt4_restore_cmd)
% CommandMuxFunction
% mission_phase???곕씪 ?곸감/?섏감 紐낅졊??理쒖쥌 ?μ튂 紐낅졊?쇰줈 ?좏깮?쒕떎.
%
% mission_phase:
%   0 = ?곸감 ?④퀎
%   1 = ?섏감 ?④퀎
%   2 = 誘몄뀡 ?꾨즺
%
% final_floorN_conv_cmd:
%   1 = ?대떦 痢??꾩껜 ?쒗솚 而⑤쿋?댁뼱 ?쒓퀎諛⑺뼢 ?뚯쟾
%   0 = ?뺤?
%
% final_floorN_belt4_cmd:
%  -1 = 4踰?踰⑦듃 ??쉶?? ?앸같瑜??뚮옯?쇱쑝濡??섏감
%   0 = ?뺤?
%   1 = 4踰?踰⑦듃 ?뺣갑??蹂듦?

% 湲곕낯媛? ?꾩껜 ?뺤?
final_lift_cmd = 0;
final_target_z = 0.0;

final_floor1_conv_cmd = 0;
final_floor2_conv_cmd = 0;
final_floor3_conv_cmd = 0;

final_floor1_belt4_cmd = 0;
final_floor2_belt4_cmd = 0;
final_floor3_belt4_cmd = 0;

% =========================
% ?곸감 ?④퀎
% =========================
if mission_phase == 0
    final_lift_cmd = load_lift_cmd;
    final_target_z = load_target_z;

    final_floor1_conv_cmd = load_floor1_conv_cmd;
    final_floor2_conv_cmd = load_floor2_conv_cmd;
    final_floor3_conv_cmd = load_floor3_conv_cmd;

    % ?곸감 以묒뿉??踰⑦듃4 ?⑤룆 ??쉶??蹂듦? 紐낅졊 ?놁쓬
    final_floor1_belt4_cmd = 0;
    final_floor2_belt4_cmd = 0;
    final_floor3_belt4_cmd = 0;

% =========================
% ?섏감 ?④퀎
% =========================
elseif mission_phase == 1
    final_lift_cmd = unload_lift_cmd;
    final_target_z = unload_target_z;

    % ?섏감 以鍮꾩슜 ?꾩껜 ?쒗솚 而⑤쿋?댁뼱 紐낅졊
    final_floor1_conv_cmd = pre_floor1_conv_cmd;
    final_floor2_conv_cmd = pre_floor2_conv_cmd;
    final_floor3_conv_cmd = pre_floor3_conv_cmd;

    % 4踰?踰⑦듃 ?⑤룆 ?숈옉 以묒뿉???꾩껜 ?쒗솚 而⑤쿋?댁뼱 ?뺤?
    if belt4_reverse_cmd == 1 || belt4_restore_cmd == 1
        final_floor1_conv_cmd = 0;
        final_floor2_conv_cmd = 0;
        final_floor3_conv_cmd = 0;
    end

    % ?섏감 ?쒓컙 4踰?踰⑦듃 ?⑤룆 紐낅졊
    if target_unload_floor == 1
        if belt4_reverse_cmd == 1
            final_floor1_belt4_cmd = -1;
        elseif belt4_restore_cmd == 1
            final_floor1_belt4_cmd = 1;
        else
            final_floor1_belt4_cmd = 0;
        end

    elseif target_unload_floor == 2
        if belt4_reverse_cmd == 1
            final_floor2_belt4_cmd = -1;
        elseif belt4_restore_cmd == 1
            final_floor2_belt4_cmd = 1;
        else
            final_floor2_belt4_cmd = 0;
        end

    elseif target_unload_floor == 3
        if belt4_reverse_cmd == 1
            final_floor3_belt4_cmd = -1;
        elseif belt4_restore_cmd == 1
            final_floor3_belt4_cmd = 1;
        else
            final_floor3_belt4_cmd = 0;
        end
    end

% =========================
% 誘몄뀡 ?꾨즺 ?먮뒗 ?덉쇅
% =========================
else
    final_lift_cmd = 0;
    final_target_z = 0;
    final_pusher_cmd = 0;

    final_floor1_conv_cmd = 0;
    final_floor2_conv_cmd = 0;
    final_floor3_conv_cmd = 0;

    final_belt4_reverse_cmd = 0;
    final_belt4_restore_cmd = 0;
end
