% Script to perform optimization using LM
clc; clear all; close all
global observed

% Simulate collecting observations _______________________________________

N = 10; % Number of observations
angles = 0:2:80; % Range of scans

% specimen geometry parameters [x_s,z_s,theta_s]
Specimen_p = [.1 .1 .2]; 

% we will assume the operator placed the sample with some random noise
User_c = -Specimen_p + randn(size(Specimen_p)).*[1.5 1.5 .5];

% allocate
I = zeros(N,length(angles)); d = zeros(N,length(angles)); User_c_set = zeros(N,3);

% the each observation
for i=1:N
    % While the mean intensity is less then .1 or the count is less then 10
    % keep resimulating with different permutations. The thought is: the
    % algorithm works best with observations which have some meat to them
    % so to speak.
    Imean = 0; count = 0;
    while Imean < .1 || count < 10
        % up count
        count = count+1; 
        % new permutation
        User_c_set(i,:) = User_c + randn(size(User_c)).*[1.5 1.5 .5];
        % simulate
        [I(i,:) d(i,:)] = f_Theta2theta(User_c_set(i,:),Specimen_p,angles);
        % mean of the intensities
        Imean = mean(I(i,:));
    end
end

% show me
plot(angles,I,'.-'); grid on
xlabel('\theta (deg)'); ylabel('Intensity (A.U.)');
matlab2tikz('obs.tikz','height','\figureheight','width','\figurewidth')

% Optimize for the specimen parameters ____________________________________

% pack the observed
observed = [User_c_set repmat(User_c,N,1) repmat(angles,N,1) I];

options = struct('GoalsExactAchieve',0,'GradConstr',false,'Display','iter');

tic
Specimen_p_opt = fminlbfgs(@f_Error,-User_c,options);
toc


% Tell me _________________________________________________________________

% print to screen
fprintf('\nOptimization results (0 is desired):\n')
fprintf('Error function before: %g \n',f_Error(-User_c))
fprintf('Error function after: %g \n',f_Error(Specimen_p_opt))

