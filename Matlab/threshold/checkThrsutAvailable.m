% This function is used to check whether a thruster is available now.
% Because of thruster restriction, sometimes we can't switch the command
% too fast.
% A thruster only has two statuses: on (1) or off (0)

function thrust_switch = checkThrsutAvailable(odd_s, new_s, current_time, last_switch_time)
% varaiable thrust_switch indicates whether to activate the thruster

restriction_on = 0.03;
restriction_off = 0.03;

if new_s > odd_s % turn on now
    if (current_time-last_switch_time) <= restriction_on
        thrust_switch = 0;
    else
        thrust_switch = 1;
    end
else % turn off now
    if (current_time-last_switch_time) <= restriction_off
        thrust_switch = 0;
    else
        thrust_switch = 1;
    end
end

end