function [pusher_vel, push_done, pusher_home_done, push_count] = PusherControlFunction(pusher_cmd, pusher_pos)
% PusherControlFunction
% pusher_cmd:
%   0 = ?뺤?
%   1 = ?꾩쭊, ?앸같瑜?踰⑦듃4 諛⑺뼢?쇰줈 諛湲?%   2 = ?먯젏 蹂듦?

persistent count
persistent prev_push_done

if isempty(count)
    count = 0;
    prev_push_done = 0;
end

pusher_speed = 0.05;     % m/s
push_target_pos = 0.24;  % m
pos_tol = 0.002;         % m

pusher_vel = 0;
push_done = 0;
pusher_home_done = 0;

if pusher_cmd == 1
    if pusher_pos >= push_target_pos - pos_tol
        pusher_vel = 0;
        push_done = 1;
        pusher_home_done = 0;
    else
        pusher_vel = pusher_speed;
        push_done = 0;
        pusher_home_done = 0;
    end

elseif pusher_cmd == 2
    if pusher_pos <= pos_tol
        pusher_vel = 0;
        push_done = 0;
        pusher_home_done = 1;
    else
        pusher_vel = -pusher_speed;
        push_done = 0;
        pusher_home_done = 0;
    end

else
    pusher_vel = 0;
    push_done = 0;
    pusher_home_done = 0;
end

% push_done ?곸듅 ?먯? 移댁슫??if prev_push_done == 0 && push_done == 1
    count = count + 1;
end

prev_push_done = push_done;

push_count = count;