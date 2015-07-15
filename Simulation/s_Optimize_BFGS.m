%% Script to perform optimization using BFGS
clc; clear all; close all
global observed

%% Simulate collecting observations _______________________________________

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

% show me the observations
plot(angles,I,'.-'); grid on
xlabel('\theta (deg)'); ylabel('Intensity (A.U.)');
matlab2tikz('obs.tikz','height','\figureheight','width','\figurewidth')

%% Optimize for the specimen parameters ____________________________________

% pack the observed
observed = [User_c_set repmat(User_c,N,1) repmat(angles,N,1) I];

options = struct('GoalsExactAchieve',0,'GradConstr',false,...
    'Display','iter','MaxFunEvals',1000);

tic
Specimen_p_opt = fminlbfgs(@f_Error,-User_c,options);
toc


%% Tell me _________________________________________________________________

% print to screen
fprintf('\nOptimization results (0 is desired):\n')
fprintf('Error function before: %g \n',f_Error(-User_c))
fprintf('Error function after: %g \n',f_Error(Specimen_p_opt))

%% Evaluate the space with a grid pattern _________________________________

% pack the observed (should have it)
% observed = [User_c_set repmat(User_c,N,1) repmat(angles,N,1) I];

% range to explore
range = 2.5; step = .1;
range_Sx = Specimen_p(1)-range : step: Specimen_p(1)+range;
range_Sz = Specimen_p(2)-range : step : Specimen_p(2)+range;
range_Stheta = Specimen_p(3)-range : step : Specimen_p(3)+range;

% number of entries in each range
nSx = length(range_Sx);
nSz = length(range_Sz);
nStheta = length(range_Stheta);

if matlabpool('size') == 0
    matlabpool 2
end

% evaluate for Sx and Stheta
tic;
eVal_SxStheta1 = zeros(nSx,nStheta);
eVal_SxStheta2 = zeros(nSx,nStheta);
parfor ii=1:nSx
    for jj=1:nStheta
        state = [range_Sx(ii) Specimen_p(2) range_Stheta(jj)];
        [eVal_SxStheta1(jj,ii) eVal_SxStheta2(jj,ii)] = ...
            f_Error2(state,observed);
    end
end
eVal_SxStheta1 = exp(-eVal_SxStheta1);
eVal_SxStheta2 = exp(-eVal_SxStheta2);
toc

% evaluate for Sz and Stheta
tic;
eVal_SzStheta1 = zeros(nSz,nStheta);
eVal_SzStheta2 = zeros(nSz,nStheta);
parfor ii=1:nSz
    for jj=1:nStheta
        state = [Specimen_p(1) range_Sz(ii) range_Stheta(jj)];
        [eVal_SzStheta1(jj,ii) eVal_SzStheta2(jj,ii)] = ...
            f_Error2(state,observed);
    end
end
eVal_SzStheta1 = exp(-eVal_SzStheta1);
eVal_SzStheta2 = exp(-eVal_SzStheta2);
toc

%% Show Me ________________________________________________________________

[init_error1 init_error2] = f_Error2(-User_c,observed);
init_error1 = exp(-init_error1);
init_error2 = exp(-init_error2);

% figure;
% pos = get(gcf,'Position'); set(gcf,'Position',[pos(1:2) 2*pos(3:4)])
% surf(range_Sx,range_Stheta,eVal_SxStheta1)
% xlabel('X_s (cm)','FontSize',15,'FontName','Times'); 
% ylabel('\theta_s (deg)','FontSize',15,'FontName','Times');
% hold on; plot3(Specimen_p(1),Specimen_p(3),1,'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'r','LineWidth',1.5); hold off; axis auto
% hold on; plot3(Specimen_p_opt(1),Specimen_p_opt(3),1,'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'm','LineWidth',1.5); hold off; axis auto
% hold on; plot3(-User_c(1),-User_c(3),init_error1,'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'g','LineWidth',1.5); hold off; 
% %print -depsc -noui SxStheta_Error.eps 
% matlab2tikz('e_SxStheta1.tikz','height','\figureheight','width','\figurewidth')

figure;
pos = get(gcf,'Position'); set(gcf,'Position',[pos(1:2) 2*pos(3:4)])
surf(range_Sx,range_Stheta,eVal_SxStheta2)
xlabel('X_s (cm)','FontSize',15,'FontName','Times'); 
ylabel('\theta_s (deg)','FontSize',15,'FontName','Times');
hold on; plot3(Specimen_p(1),Specimen_p(3),1,'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'r','LineWidth',1.5); hold off; axis auto
hold on; plot3(Specimen_p_opt(1),Specimen_p_opt(3),1,'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'm','LineWidth',1.5); hold off; axis auto
hold on; plot3(-User_c(1),-User_c(3),init_error2,'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'g','LineWidth',1.5); hold off; 
% print -depsc -noui SxStheta_Error.eps 
matlab2tikz('e_SxStheta2.tikz','height','\figureheight','width','\figurewidth')

% figure;
% pos = get(gcf,'Position'); set(gcf,'Position',[pos(1:2) 2*pos(3:4)])
% surf(range_Sz,range_Stheta,eVal_SzStheta1)
% xlabel('Z_s (cm)','FontSize',15,'FontName','Times'); 
% ylabel('\theta_s (deg)','FontSize',15,'FontName','Times');
% hold on; plot3(Specimen_p(2),Specimen_p(3),1,'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'r','LineWidth',1.5); hold off; axis auto
% hold on; plot3(Specimen_p_opt(2),Specimen_p_opt(3),1,'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'm','LineWidth',1.5); hold off; axis auto
% hold on; plot3(-User_c(2),-User_c(3),init_error1,'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'g','LineWidth',1.5); hold off; 
% %print -depsc -noui SzStheta_Error.eps 
% matlab2tikz('e_SzStheta1.tikz','height','\figureheight','width','\figurewidth')

figure;
pos = get(gcf,'Position'); set(gcf,'Position',[pos(1:2) 2*pos(3:4)])
surf(range_Sz,range_Stheta,eVal_SzStheta2)
xlabel('Z_s (cm)','FontSize',15,'FontName','Times'); 
ylabel('\theta_s (deg)','FontSize',15,'FontName','Times');
hold on; plot3(Specimen_p(2),Specimen_p(3),1,'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'r','LineWidth',1.5); hold off; axis auto
hold on; plot3(Specimen_p_opt(2),Specimen_p_opt(3),1,'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'm','LineWidth',1.5); hold off; axis auto
hold on; plot3(-User_c(2),-User_c(3),init_error2,'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'g','LineWidth',1.5); hold off; 
% print -depsc -noui SzStheta_Error.eps 
matlab2tikz('e_SzStheta2.tikz','height','\figureheight','width','\figurewidth')

%% with contours

% figure;
% pos = get(gcf,'Position'); set(gcf,'Position',[pos(1:2) 2*pos(3:4)])
% contourf(range_Sx,range_Stheta,eVal_SxStheta1)
% xlabel('X_s (cm)','FontSize',15,'FontName','Times'); 
% ylabel('\theta_s (deg)','FontSize',15,'FontName','Times');
% hold on; plot(Specimen_p(1),Specimen_p(3),'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'w','LineWidth',.5); hold off; axis auto
% hold on; plot(Specimen_p_opt(1),Specimen_p_opt(3),'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'm','LineWidth',.5); hold off; axis auto
% hold on; plot(-User_c(1),-User_c(3),'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'g','LineWidth',.5); axis tight; hold off; 
% %print -depsc -noui SxStheta_Error.eps 
% matlab2tikz('e_cSxStheta1.tikz','height','\figureheight','width','\figurewidth')

figure;
pos = get(gcf,'Position'); set(gcf,'Position',[pos(1:2) 2*pos(3:4)])
contourf(range_Sx,range_Stheta,eVal_SxStheta2)
xlabel('X_s (cm)','FontSize',15,'FontName','Times'); 
ylabel('\theta_s (deg)','FontSize',15,'FontName','Times');
hold on; plot(Specimen_p(1),Specimen_p(3),'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'w','LineWidth',.5); hold off; axis auto
hold on; plot(Specimen_p_opt(1),Specimen_p_opt(3),'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'g','LineWidth',.5); hold off; axis auto
hold on; plot(-User_c(1),-User_c(3),'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'r','LineWidth',.5); axis tight; hold off; 
%print -depsc -noui SxStheta_Error.eps 
matlab2tikz('e_cSxStheta2.tikz','height','\figureheight','width','\figurewidth')

% figure;
% pos = get(gcf,'Position'); set(gcf,'Position',[pos(1:2) 2*pos(3:4)])
% contourf(range_Sz,range_Stheta,eVal_SzStheta1)
% xlabel('Z_s (cm)','FontSize',15,'FontName','Times'); 
% ylabel('\theta_s (deg)','FontSize',15,'FontName','Times');
% hold on; plot(Specimen_p(2),Specimen_p(3),'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'w','LineWidth',.5); hold off; axis auto
% hold on; plot(Specimen_p_opt(2),Specimen_p_opt(3),'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'g','LineWidth',.5); hold off; axis auto
% hold on; plot(-User_c(2),-User_c(3),'o',...
%     'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
%     'r','LineWidth',.5); axis tight; hold off; 
% %print -depsc -noui SzStheta_Error.eps 
% matlab2tikz('e_cSzStheta1.tikz','height','\figureheight','width','\figurewidth')

figure;
pos = get(gcf,'Position'); set(gcf,'Position',[pos(1:2) 2*pos(3:4)])
contourf(range_Sz,range_Stheta,eVal_SzStheta2)
xlabel('Z_s (cm)','FontSize',15,'FontName','Times'); 
ylabel('\theta_s (deg)','FontSize',15,'FontName','Times');
hold on; plot(Specimen_p(2),Specimen_p(3),'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'w','LineWidth',.5); hold off; axis auto
hold on; plot(Specimen_p_opt(2),Specimen_p_opt(3),'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'g','LineWidth',.5); hold off; axis auto
hold on; plot(-User_c(2),-User_c(3),'o',...
    'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor',...
    'r','LineWidth',.5); axis tight; hold off; 
%print -depsc -noui SzStheta_Error.eps 
matlab2tikz('e_cSzStheta2.tikz','height','\figureheight','width','\figurewidth')