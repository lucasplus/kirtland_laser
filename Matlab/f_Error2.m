% Function to determine the error given beta and x
% calculates the error using both the old metric and the new one
%
% Inputs:
%   param - (dependent) {Sx Sz Stheta}
%   observed - (independent) A Nx[3+length(angles)] matrix of the
%       observation data
%           {[User_c] [User_c_initial] [angles] [I]}
% Outputs:
%   y1hat - according to old metric
%   y2hat - according to new metric
function [y1hat y2hat] = f_Error2(param,observed)

N = size(observed,1); % number of observations
Na = (size(observed,2)-6)/2; % number of angles

angles = observed(1,7:(6+Na)); % the angles

y1 = zeros(1,N); y2 = zeros(1,N);

% for the number of observations
for i=1:N
    User_c = observed(i,1:3); % user command for this observation
    
    % model
    Im = f_Theta2theta(User_c,param,angles)'; 
    
    % observed
    Io = observed(i,(7+Na):end);
    
    % difference between observed and model
    y1(i) = mean(abs(Im'-Io'));
    
    % shape info (0 if same ; 1 > x > 0 depending on how related)
    y2(i) = f_SignalRelation(Im,Io);
end

y1hat = mean(y1); y2hat = mean(y2);