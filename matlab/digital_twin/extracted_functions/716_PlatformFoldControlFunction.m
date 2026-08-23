function [platform_fold_angle, platform_deployed] = PlatformFoldControlFunction(mission_phase, delivery_arrived, platform_z)
% PlatformFoldControlFunction
%
% ?뚮옯??諛붾떏???묓옒/?쇱묠 媛곷룄瑜?遺?쒕읇寃??앹꽦?쒕떎.
%
% ?낅젰:
% mission_phase:
%   0 = ?곸감
%   1 = ?섏감 以鍮??섏감
%   2 = 誘몄뀡 ?꾨즺
%
% delivery_arrived:
%   0 = 諛곗넚 以?/ ?섏감 以鍮?%   1 = 諛곕떖 ?μ냼 ?꾩갑 / ?ㅼ젣 ?섏감 媛??%
% platform_z:
%   ?꾩옱 ?뚮옯???믪씠
%
% 異쒕젰:
% platform_fold_angle:
%   0    = ?쇱퀜吏??곹깭
%   pi/2 = ?묓엺 ?곹깭
%
% platform_deployed:
%   1 = 嫄곗쓽 ?꾩쟾???쇱퀜吏?%   0 = ?묓옒 ?먮뒗 ?묓엳???쇱퀜吏??以?%
% ?듭떖 洹쒖튃:
% - ?곸감 以묒뿉???쇱묠
% - 諛곗넚 以??섏감 以鍮??④퀎?먯꽌???뚮옯?쇱씠 1痢듭뿉 ?꾩갑?????묓옒
% - 諛곗넚 以묒씠?쇰룄 ?뚮옯?쇱씠 1痢듭씠 ?꾨땲硫??묓엳吏 ?딄퀬 ?쇱튇 ?곹깭 ?좎?
% - 諛곕떖 ?μ냼 ?꾩갑 ???ㅼ젣 ?섏감 ?④퀎?먯꽌???쇱묠
% - 誘몄뀡 ?꾨즺 ?꾩뿉??1痢듭뿉???묓옒

persistent angle

if isempty(angle)
    angle = pi/2;   % ?쒖옉? ?묓엺 ?곹깭
end

% =========================
% 1痢??꾩갑 ?먮떒
% =========================
floor1_z = 0.00;
floor_tol = 0.005;

platform_at_floor1 = 0;

if platform_z >= floor1_z - floor_tol && platform_z <= floor1_z + floor_tol
    platform_at_floor1 = 1;
end

% =========================
% ?쇱묠/?묓옒 紐낅졊 ?먮떒
% =========================
% deploy_cmd = 1 ???쇱묠
% deploy_cmd = 0 ???묓옒
deploy_cmd = 0;

% ?곸감 ?④퀎: ?앸같瑜??뚮옯???꾩뿉 ?щ젮???섎?濡??쇱묠
if mission_phase == 0
    deploy_cmd = 1;
end

% ?섏감 以鍮??섏감 ?④퀎
if mission_phase == 1

    if delivery_arrived > 0.5
        % 諛곕떖 ?μ냼 ?꾩갑 ???ㅼ젣 ?섏감 ?④퀎???쇱묠
        deploy_cmd = 1;
    else
        % 諛곗넚 以??섏감 以鍮??④퀎
        % ?뚮옯?쇱? 1痢듭뿉 ?덉쑝硫??묎퀬,
        % 1痢듭씠 ?꾨땲硫?癒쇱? 1痢?蹂듦? 以묒씠誘濡??쇱튇 ?곹깭 ?좎?
        if platform_at_floor1 > 0.5
            deploy_cmd = 0;
        else
            deploy_cmd = 1;
        end
    end
end

% 誘몄뀡 ?꾨즺 ??if mission_phase >= 2
    if platform_at_floor1 > 0.5
        deploy_cmd = 0;
    else
        deploy_cmd = 1;
    end
end

% =========================
% 遺?쒕윭??媛곷룄 ?대룞
% =========================
% ?섑뵆???0.05 湲곗?
% 0.04 rad/step?대㈃ ??2珥??뺣룄??90???묓옒/?쇱묠
fold_step = 0.04;

if deploy_cmd > 0.5
    % ?쇱튂湲? pi/2 ??0
    angle = angle - fold_step;

    if angle < 0
        angle = 0;
    end
else
    % ?묎린: 0 ??pi/2
    angle = angle + fold_step;

    if angle > pi/2
        angle = pi/2;
    end
end

platform_fold_angle = angle;

% 嫄곗쓽 ?꾩쟾???쇱퀜吏?寃쎌슦留?deployed = 1
if angle < 0.05
    platform_deployed = 1;
else
    platform_deployed = 0;
end

end