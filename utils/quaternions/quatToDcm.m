function C = quatToDcm( q )
%  QUATTODCM Convert quaternion to direction cosine matrix.
%  
% PARAMETERS:
% ----------
% q: [N x 4] double
%       list of quaternions, where each column is a quaternion.
%
% RETURNS:
% ----------
% C: [3 x 3 x 4] double
%       list of DCMs, each "slice" of 3D matrix is a DCM.
%
% Compatible with complex step, unlike quat2dcm() function from the
% aerospace toolbox.
%
%   Copyright 2000-2007 The MathWorks, Inc.

% Normalize (vectorized implementation compatible with complex step).
qin = (q./(sum(q.*q,1).^0.5)).';

C = zeros(3,3,size(qin,1));

C(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
C(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
C(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
C(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
C(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
C(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
C(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
C(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
C(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;