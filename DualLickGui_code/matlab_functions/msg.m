%% msg

% accepts: component appComponent, string command
% returns: message

% function: gets current value of app component and formats the proper 
% letter command to send to arduino

function message = msg(appComponent, command)

    % Concatenates command and component value
    value = appComponent.Value;
    message = [command, num2str(value)];
    
end