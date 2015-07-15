% function to render a view of the test set up
% INPUTS:
%   T_Cell - a 2xN cell array containing the transformation matrix in the 
%   top row and a label on the second row. N is the number of matrices to 
%   be plotted
%   L_Cell - a 2xN cell array containing the transformation matrix in the 
%   top row and a label on the second row. N is the number of matrices to 
%   be plotted
function f_Render(T_Cell,L_Cell,plot_flag)

% Setup figure ___________________________________________________________

% If the plot flag is on then the user wants to set up the axis from
% scratch, if it isn't one then the axis are already set up
if plot_flag == 1
    plot3(0,0,0); % needs this to work
    grid on; view([180 0]); camroll(90); % make a grid and change view
end

% Unpack _________________________________________________________________
T0S = T_Cell{1,4}; T0D = T_Cell{1,5};

p_reflect = L_Cell{1,1}; d_reflect = L_Cell{1,2}; dn = L_Cell{1,3}; 

% Plot the tranformation frames __________________________________________
scale=40; 

% Number of frames to plot
N = size(T_Cell,2);

% Initialize
Label = num2cell(1:3); 
Location = [linspace(.2,1,N-1) .3];

for i=1:N
    % Define the Label 
    Label{1} = sprintf('X_%s',T_Cell{2,i});
    Label{2} = sprintf('Y_%s',T_Cell{2,i});
    Label{3} = sprintf('Z_%s',T_Cell{2,i});
    
    lf_SingleFrame(T_Cell{1,i},scale(1),Label,Location(i));
end

% Plot the specimen and detector _________________________________________
lf_PlotRectangle(T0S,40); lf_PlotRectangle(T0D,10,30)

% Plot the laser and normal vectors ______________________________________

% Laser source
hold on; line([p_reflect(1) 0],[p_reflect(2) 0],[p_reflect(3) -125],...
    'Color',[1 0 0]) 

% Add the reflection vector to the plot
hold on; quiver3(p_reflect(1),p_reflect(2),p_reflect(3),...
    d_reflect(1),d_reflect(2),d_reflect(3),130,'r')

% Add the normal to the plot
hold on; quiver3(p_reflect(1),p_reflect(2),p_reflect(3),...
    dn(1),dn(2),dn(3),20,'g')

% Make sure the axis are large enough and square
set(gca,'XLim',[-50 160],'ZLim',[-160 50])


% function to plot a coordinate frame on the current figure
function lf_SingleFrame(T,Len,Label,Label_Location)
% INPUT: 
%   T - Transformation matrix to be visualized
%   Len - Length of the axis (defaults to 1)
%   Label - 1x3 cell array of labels (defaults to x,y,&z)      
%   Label_Location - Where to place the label along the axis (0-1)

% % If there are less than 4 arguments, assign the Label_Location
% if nargin < 4
%   Label_Location = 1;
% end
% 
% % If there are less than 3 arguments, assign the Labels
% if nargin < 3
%   Label = num2cell(1:3);
%   Label{1} = 'x';
%   Label{2} = 'y';
%   Label{3} = 'z';
% end
%   
% % If less than 2, assign the length
% if nargin < 2
%   Len = 1;  
% end

% Points to be plotted
o = [0   0   0  ]';
x = [Len 0   0  ]'; 
y = [0   Len 0  ]'; 
z = [0   0   Len]'; 

% Transform them
o = T*[o; 1];
x = T*[x; 1];
y = T*[y; 1];
z = T*[z; 1];

% Plot the lines
line([o(1) x(1)],[o(2) x(2)],[o(3) x(3)])
line([o(1) y(1)],[o(2) y(2)],[o(3) y(3)])
line([o(1) z(1)],[o(2) z(2)],[o(3) z(3)])

% Find the Label Location in global coordinates
where = Label_Location * Len;
L_x = [where 0     0    ]'; 
L_y = [0     where 0    ]'; 
L_z = [0     0     where]'; 

% Now in Frame coordinates
L_x = T*[L_x; 1];
L_y = T*[L_y; 1];
L_z = T*[L_z; 1];

% Label them
text(L_x(1),L_x(2),L_x(3),Label{1},'FontName','Times','FontAngle','italic','FontSize',12);
text(L_y(1),L_y(2),L_y(3),Label{2},'FontName','Times','FontAngle','italic','FontSize',12);
text(L_z(1),L_z(2),L_z(3),Label{3},'FontName','Times','FontAngle','italic','FontSize',12);

% function to add a rectangle to the plot
function lf_PlotRectangle(T,L,W)
% INPUTS:
%   fig_handle - handle to the figure 
%   T - transformation matrix to the frame where the specimen will be
%   L (optional) - the length of the specimen
%   W (optional) - the width of the specimen

% if the user does not specify a length go with default
if nargin < 2
    L = 10;
end

% if the user wants to specify both length and width let him
if nargin < 3
    W = .2*L; % width of the object (function of length)
end

% The corners of the object
c1 = T*[L/2;0;0;1];
c2 = T*[L/2;0;W;1];
c3 = T*[-L/2;0;W;1];
c4 = T*[-L/2;0;0;1];

% Coordinates of the corners
X = [c1(1) c2(1) c3(1) c4(1)];
Y = [c1(2) c2(2) c3(2) c4(2)];
Z = [c1(3) c2(3) c3(3) c4(3)];

% Add to plot
hold on; p_handle = patch(X,Y,Z,'b');
alpha(p_handle,.2);
hold off;