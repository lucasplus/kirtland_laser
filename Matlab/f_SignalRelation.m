% function to find the shape relation between 2 signals
function rAB = f_SignalRelation(A,B)

% reform to make sure array is in row form
sA = size(A); sB = size(B);
if sA(1)~=1; A = A'; end
if sB(1)~=1; B = B'; end

% auto-covariance
xAA = xcov(A,'biased');
xBB = xcov(B,'biased');

% max of the auto-covariances
mxAABB = max([xAA xBB]);
if mxAABB==0; mxAABB=eps; end

% cross-covariance
xAB = xcov(A,B,'biased');

% ratio
rAB = max(xAB)/mxAABB;

% make it so that 0 if same and 1>rAB>0 depending on how related
rAB = 1-rAB;