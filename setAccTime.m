function setAccTime(~,info)
% This is the function that executes when you press a key and have the
% Phantom figure window highlighted.  It adds the acceleration, start,
% and end times to the global variable that tracks such things.

% Declare the global variable that we're updating, plus the global
% variables that control the strength and duration of the acceleraton
% pulses that we are applying.
global keyacctime amag adur

% % Log the time this function was called.
tnow = toc;

% Get the key that the user pressed.
k = info.Key;

% Set the acceleration direction that corresponds to the key the user
% has pressed. The positive x-axis points out at the user, the
% positive y-axis points to the user's right, and the positive z-axis
% points up. You are welcome to change this mapping however you want.
switch k
    case {'l' '4' 'numpad4' 'leftarrow'}
        ahat = [0 -1 0]';
    case {'r' '6' 'numpad6' 'rightarrow'}
        ahat = [0 1 0]';
    case {'u' '8' 'numpad8' 'uparrow'}
        ahat = [0 0 1]';
    case {'d' '2' 'numpad2' 'downarrow'}
        ahat = [0 0 -1]';
    case {'i' '9' 'numpad9' 'slash'}
        ahat = [-1 0 0]';
    case {'o' '1' 'numpad1' 'period'}
        ahat = [1 0 0]';
    case {'z' 's' '5' 'numpad5' '0' 'comma'}
        ahat = nan * [1 1 1]';
    otherwise
        ahat = [0 0 0];
end

if (norm(ahat) > 0)
    % Store the desired acceleration pulse in the global variable,
    % including the 3D acceleration vector, start time, and end time.
    keyacctime(:,end+1) = [amag*ahat; tnow; tnow + adur];
elseif (isnan(norm(ahat)))
    % Special case of bringing haptic device to a halt.  We just pass
    % through the sentinel value of NaN in the vector.
    keyacctime(:,end+1) = [amag*ahat; tnow; tnow + adur];
end