function [box_type, box_length, box_width, box_height, package_count] = BoxSizeFunction(load_done)
% BoxSizeFunction
% ?곗냽 ?곸감???쒕뜡 ?앸같 ?ш린 ?앹꽦 ?⑥닔
%
% load_done??0 -> 1濡?諛붾뚮뒗 ?쒓컙,
% ?ㅼ쓬 ?곸감 ?ъ씠?댁뿉???ъ슜?????앸같 醫낅쪟瑜??쒕뜡 ?앹꽦?쒕떎.
%
% 異쒕젰 ?⑥쐞:
% box_length, box_width, box_height = m

persistent selected_type
persistent count
persistent prev_load_done

if isempty(selected_type)
    selected_type = randi(4);
    count = 1;
    prev_load_done = 0;
end

% load_done ?곸듅 ?먯? 媛먯?
rising_edge = (prev_load_done == 0) && (load_done == 1);

% ?곸감 ?꾨즺 ?쒓컙 ?ㅼ쓬 ?앸같 ?앹꽦
if rising_edge
    selected_type = randi(4);
    count = count + 1;
end

prev_load_done = load_done;

box_type = selected_type;
package_count = count;

% ?곗껜援??앸같 1~4???덉떆 移섏닔
% ?ㅼ젣 ?쒖옉 諛뺤뒪 移섏닔??留욎떠 ?섏쨷???섏젙 媛??% [length, width, height], ?⑥쐞: m
if box_type == 1
    real_length = 0.220;
    real_width  = 0.190;
    real_height = 0.090;
elseif box_type == 2
    real_length = 0.270;
    real_width  = 0.180;
    real_height = 0.150;
elseif box_type == 3
    real_length = 0.340;
    real_width  = 0.250;
    real_height = 0.210;
else
    real_length = 0.410;
    real_width  = 0.310;
    real_height = 0.280;
end

% 2:1 異뺤냼 ?ㅼ???scale = 0.5;

box_length = real_length * scale;
box_width  = real_width  * scale;
box_height = real_height * scale;