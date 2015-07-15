% function to simulate a theta2theta scan
% Inputs:
%   User_c - [x,z,theta] => command parameters
%   Specimen_p - [x_s,z_s,theta_s] => specimen geometry parameters
%   angles - a Nx1 array angular positions the specimen will go through
%       during the simulated theta2theta scan
% Outputs:
%   I - intensity as seen by the detector
%   d - distance from the beam to the detector (cm)
function [I d] = f_Theta2theta(User_c,Specimen_p,angles)

% Setup ___________________________________________________________________

% user commands and specimen parameters
x_c = User_c(1); z_c = User_c(2); theta_c = User_c(3);
x_s = Specimen_p(1); z_s = Specimen_p(2); theta_s = Specimen_p(3);

Na = length(angles); % number of angles in the test

% Translations ____________________________________________________________

% T0 = eye(4); % The base frame {0}


% Transformation from the base frame {0} to frame {1} 

% sin and cos of angles for the specimen
% theta_c is added because it is the offset specified by the user
s_angles = sind(angles+theta_c); c_angles = cosd(angles+theta_c); 

% form the T01 cell array
% start with template, make it into a cube, add the angles, turn into cell
T01 = zeros(4,4,Na);
T01(2,2,:) = ones(1,Na); T01(4,4,:) = ones(1,Na);
T01(1,1,:) = c_angles;   T01(3,3,:) = c_angles;
T01(1,3,:) = -s_angles;  T01(3,1,:) = s_angles;
    

% Transformation to frame {3} 

% form the T03 cell array
% start with template, make it into a cube, turn into cell
T03 = zeros(4,4,Na);
T03(1,1,:) = ones(1,Na); T03(1,4,:) = x_c*ones(1,Na);
T03(3,3,:) = ones(1,Na); T03(3,4,:) = z_c*ones(1,Na);
T03(4,4,:) = ones(1,Na); T03(2,2,:) = ones(1,Na);
T03 = lf_MDTimes(T01,T03);


% Transformation to the specimen surface {S}

% angle for the specimen (it stays constant)
c_theta = cosd(theta_s); s_theta = sind(theta_s);
   
T0S = zeros(4,4,Na);
T0S(1,1,:) = c_theta*ones(1,Na); T0S(1,3,:) = -s_theta*ones(1,Na);
T0S(1,4,:) = x_s*ones(1,Na); T0S(2,2,:) = ones(1,Na); 
T0S(3,1,:) = s_theta*ones(1,Na); T0S(3,3,:) = c_theta*ones(1,Na); 
T0S(3,4,:) = z_s*ones(1,Na); T0S(4,4,:) = ones(1,Na);

T0S = lf_MDTimes(T03,T0S);


% Transformation from the base {0} to the detector {D} 

% sin and cos of angles for the detector
angles_d = 2*angles; % the detector goes to twice the specimen angle
s_angles_d=sind(angles_d); c_angles_d=cosd(angles_d); % sin and cos of angles

% form the T0D cell array
T0D = zeros(4,4,Na);
T0D(2,2,:) = ones(1,Na);     T0D(4,4,:) = ones(1,Na);
T0D(1,1,:) = -c_angles_d;    T0D(3,3,:) = -c_angles_d;
T0D(1,3,:) = s_angles_d;     T0D(3,1,:) = -s_angles_d;
T0D(1,4,:) = 125*s_angles_d; T0D(3,4,:) = -125*c_angles_d;

% Trace the path of the laser ____________________________________________
[p_reflect d_reflect dn] = lf_Trace(T0S);

% Determine the intensity and the distance to the detector _______________
[I d] = lf_Sensor(p_reflect,d_reflect,T0D);


% function to multiply two multidimensional matrices togethor
function C = lf_MDTimes(A,B)

Ns = size(A,3); % Number of sheets

C = zeros(size(A));
for i=1:Ns;
    C(:,:,i) = A(:,:,i)*B(:,:,i);
end

% function to simulate the reading from the detector given the geometry 
function [I d] = lf_Sensor(p_reflect,d_reflect,T_D)
% Inputs:
%   p_reflect - 3x1xNa point of reflection
%   d_reflect - 3x1xNa direction of reflection
%   T_D - 4x4xNa transformation matrix to detector

Na = size(T_D,3); % Number or angles

I = zeros(Na,1); d = zeros(Na,1);
for i=1:Na

% get where detector is located (P_D) from T_D
P_D = T_D(1:3,4,i);

% angle between beam and sensor
% theta = acosd((P_D'*d_reflect(:,:,i))/...
%     (sqrt(P_D'*P_D)*sqrt(d_reflect(:,:,i)'*d_reflect(:,:,i))));

% if the angle between the detector and the direction of reflection is
% above a certain threshold report an intensity of 0
% if abs(theta) > 60*(pi/180)
%     d(i) = NaN; I(i) = 0;
%     continue
% end

% points along the reflected line
P0 = p_reflect(:,:,i); P1 = p_reflect(:,:,i) + 125*d_reflect(:,:,i);

% shortest distance between the sensor and the beam
d(i) = lf_d(P_D,P0,P1);

% intensity of the light
I(i) = exp(-2*d(i)^2/1.2^2);  % (2/(pi*1.2^2)) * exp(-2*d^2/1.2^2); 

end

% distance from a line to a point in 3D
function d = lf_d(P,P0,P1)

vL = P1-P0; w = P-P0;
c_vL_w=[vL(2)*w(3)-vL(3)*w(2); vL(3)*w(1)-vL(1)*w(3); vL(1)*w(2)-vL(2)*w(1)];% cross(vL,w);
d = sqrt(c_vL_w'*c_vL_w) / sqrt(vL'*vL);

% function to trace the laser ray
function [p_reflect d_reflect dn] = lf_Trace(T0S)
% Input:
%   TOS - 4x4xNa stack of transform from base frame to the surface of the 
%       specimen
% Output:
%   p_reflect - point of reflection
%   d_reflect - direction of reflection

Na = size(T0S,3);  % Number of angles

di = [0;0;-1]; % direction of the laser

dn = zeros(3,1,Na); d_reflect = zeros(3,1,Na); p_reflect = zeros(3,1,Na);
for i=1:Na
    % normalized direction of the normal to the surface of the specimen
    % (-z of the TOS frame) - (the origin of the T0S frame)
    dn_save = (T0S(:,:,i) * [0;0;-1;1]) - T0S(1:4,4,i);
    dn(:,:,i) = dn_save(1:3); 
    dn(:,:,i) = dn(:,:,i)/sqrt(dn(:,:,i)'*dn(:,:,i));
    
    % reflection direction
    d_reflect(:,:,i) = 2*(dn(:,:,i)'*di)*dn(:,:,i)-di;
    
    % point of reflection
    % lf_Intersectv() only works in the 2D plane, so we will ignore the y
    % component of our input. Think of our xz problem as a xy problem for a
    % second
    p_reflect(:,:,i) = lf_Intersectv(dn(:,:,i),T0S(1:3,4,i),di,[0;0;-100]);
end

% function to find the intersection of a vector with a plane
function x = lf_Intersectv(n,v0,u,p0)
% INPUTS:
%   n - normal of the plane
%   v0 - point on the plane
%   u - direction of the vector
%   p0 - point on the vector

s = ((n'*(v0-p0))/(n'*u));
x = p0 + s*u;