% Function to determine the error given beta and x
%
% For both units and the extrinsic relation between them
%
%   param - (dependent) {Sx Sz Stheta}
%
%   observed - (independent) A Nx[3+length(angles)] matrix of the
%       observation data
%           {[User_c] [User_c_initial] [angles] [I]}
function yhat = f_Error(param)

global observed

N = size(observed,1); % number of observations
Na = (size(observed,2)-6)/2; % number of angles

angles = observed(1,7:(6+Na)); % the angles

y = zeros(1,N);

% for the number of observations
for i=1:N
    User_c = observed(i,1:3); % user command for this observation
    
    % model
    Im = f_Theta2theta(User_c,param,angles)'; 
    
    % observed
    Io = observed(i,(7+Na):end);
    
    % shape info (0 if same ; 1 > x > 0 depending on how related)
    y(i) = f_SignalRelation(Im,Io);
end

yhat = mean(y);
