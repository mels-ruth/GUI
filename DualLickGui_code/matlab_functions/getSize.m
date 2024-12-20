%% getSize

% accepts: int lickTime, int selectionTime
% returns: int size

% function: finds difference in time between lick time and selection time
% in milliseconds to determine marker size

function size = getSize(lickTime, selectionTime)
    diff = lickTime - selectionTime;
    if diff <= 500 % lick is 0.5 seconds or less (very fast)
        size = 20;
    elseif diff <= 1000 % lick is between 0.5 and 1 seconds (fast)
        size = 35;
    elseif diff <= 3000 % lick is between 1 and 3 seconds(fine)
        size = 50;
    elseif diff > 3000 % lick is slower than 3 seconds (bad)
        size = 65;
    end
end