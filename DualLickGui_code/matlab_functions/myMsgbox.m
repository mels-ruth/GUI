%% myMsgbox

% accepts: string message, string title, string icon
% returns: None

% function: formats a msgbox to have larger font size and appropriate
% position for this font size

function myMsgbox (message, title, icon)
    
    h = msgbox(message, title, icon);

    % Change position
    pos = get(h, 'Position');
    newWidth = pos(3) + 50;
    set(h, 'Position', [pos(1), pos(2), newWidth, pos(4)]);

    % Change font size
    textHandles = findall(h, 'Type', 'Text');
    set(textHandles, 'FontSize', 10);

end