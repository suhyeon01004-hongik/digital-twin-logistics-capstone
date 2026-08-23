function stopper_cmd_final = StopperCommandFunction(mission_phase, loading_enable)
% StopperCommandFunction
% ?ㅽ넗?쇰뒗 ?앸같 1媛쒕쭏???щ졇???대━??寃껋씠 ?꾨땲??
% ?곸감 phase ?숈븞 怨꾩냽 ?대젮? ?덇퀬,
% ?곸감媛 ?앸궃 ?ㅼ뿉留??щ씪媛꾨떎.
%
% mission_phase:
%   0 = ?곸감 ?④퀎
%   1 = ?섏감 ?④퀎
%   2 = 誘몄뀡 ?꾨즺
%
% loading_enable:
%   1 = ?곸감 ?덉슜
%   0 = ?곸감 醫낅즺

if mission_phase == 0 && loading_enable == 1
    stopper_cmd_final = 1;   % ?곸감 以? ?ㅽ넗???대┝ ?좎?
else
    stopper_cmd_final = 0;   % ?곸감 醫낅즺/?섏감/?꾨즺: ?ㅽ넗???щ┝
end