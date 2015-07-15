% function to simulate the entire system
% Inputs:
%   User_c - [x,z,theta] => command parameters
%   Specimen_p - [x_s,z_s,theta_s] => specimen geometry parameters
%   Detector_c - angular position of the sensor
% Outputs:
%   T_Cell - a 2x5 cell array containing the transformation matrix in the 
%   top row and a label on the second row. 
%   L_Cell - a 1x5 cell array containing the information related to the laser
%       p_reflect - point of reflection
%       d_reflect - direction of reflection
%       dn - direction of the normal
%       I - intensity as seen by the detector
%       d - distance from the beam to the detector (cm)
function [T_Cell L_Cell] = f_Simulate(User_c,Specimen_p,Detector_c)

T0 = eye(4); % The base frame

% Unwrap _________________________________________________________________
x = User_c(1); z = User_c(2); theta = User_c(3);
x_s = Specimen_p(1); z_s = Specimen_p(2); theta_s = Specimen_p(3);

% Transformation from the base frame (0) to frame 1 ______________________
c_theta = cosd(theta); s_theta = sind(theta);

T01 = [c_theta  0  -s_theta  0;
       0        1  0         0;
       s_theta  0  c_theta   0;
       0        0  0         1];
    
% Transformation from frame 1 to frame 3 _________________________________
T03 = [1 0 0 x; 0 1 0 0; 0 0 1 z; 0 0 0 1];
T03 = T01*T03;

% Transformation from frame 3 to the specimen surface (S) ________________
c_theta = cosd(theta_s); s_theta = sind(theta_s);

T0S = [c_theta  0  -s_theta  x_s;
       0        1  0         0;
       s_theta  0  c_theta   z_s;
       0        0  0         1];
   
T0S = T03*T0S;

% Transformation from the base (0) to the detector (D) ___________________
c_theta = cosd(Detector_c); s_theta = sind(Detector_c);

T0D = [c_theta  0  -s_theta  125*s_theta;
       0        1  0         0;
       s_theta  0  c_theta   -125*c_theta;
       0        0  0         1];

% rotate so that z is pointing along the beam
T0D = T0D * [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1];

% Trace the path of the laser ____________________________________________
[p_reflect d_reflect dn] = lf_Trace(T0S);

% Determine the intensity and the distance to the detector _______________
[I d] = lf_Sensor(p_reflect,d_reflect,T0D(1:3,4));

% Return _________________________________________________________________

% Cell containing all of the transformation matrices
T_Cell = cell(2,5);
T_Cell(1,:) = {T0 T01 T03 T0S T0D}; 
T_Cell(2,:) = {'0' '1' '3' 'S' 'D'};  

% Cell containing the laser information
L_Cell = cell(1,5);
L_Cell(1,:) = {p_reflect d_reflect dn I d}; 


% function to simulate the reading from the detector given the geometry 
function [I d] = lf_Sensor(p_reflect,d_reflect,P_D)
% Inputs:
%   p_reflect - point of reflection
%   d_reflect - direction of reflection
%   P_D - point where the detector is loacted

% angle between beam and sensor
theta = acosd(dot(P_D,d_reflect)/(norm(P_D)*norm(d_reflect)));

if abs(theta) > 60*(pi/180)
    d = NaN; I = 0;
    return
end

% points along the reflected line
P0 = p_reflect; P1 = p_reflect + 125*d_reflect;

% shortest distance between the sensor and the beam
d = lf_d(P_D,P0,P1);

% intensity of the light
I = exp(-2*d^2/1.2^2);  % (2/(pi*1.2^2)) * exp(-2*d^2/1.2^2); 

% distance from a line to a point in 3D
function d = lf_d(P,P0,P1)

vL = P1-P0; w = P-P0;

d = norm(cross(vL,w)) / norm(vL);

% function to trace the laser ray
function [p_reflect d_reflect dn] = lf_Trace(T0S)
% Input:
%   TOS - transform from base frame to the surface of the specimen
% Output:
%   p_reflect - point of reflection
%   d_reflect - direction of reflection

% Initial vectors _______________________________________________________
di = [0;0;-1]; % direction of the laser

% direction of the normal to the surface of the specimen
% (-z of the TOS frame) - (the origin of the T0S frame)
% CHANGED: dn = (T0S * [0;0;-1;1]) - T0S(1:4,4); 
dn = -T0S(:,3);
dn = dn(1:3); dn = dn/norm(dn);

% Reflection direction _________________________________________________
 % R = 2*(dn*dn')-eye(3); % Householder transformation matrix
 % d_reflect = R*di; % direction of the reflection
d_reflect = 2*dot(dn,di)*dn-di;
 
% Point of reflection __________________________________________________
% lf_Intersectv() only works in the 2D plane, so we will ignore the y
% component of our input. Think of our xz problem as a xy problem for a
% second
p_reflect = lf_Intersectv(dn,T0S(1:3,4),di,[0;0;-10]);

% function to find the intersection of a vector with a plane
function x = lf_Intersectv(n,v0,u,p0)
% INPUTS:
%   n - normal of the plane
%   v0 - point on the plane
%   u - direction of the vector
%   p0 - point on the vector

s = (dot(n,(v0-p0))/dot(n,u));
x = p0 + s*u;