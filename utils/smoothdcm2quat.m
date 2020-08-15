function q_ba = smoothdcm2quat(C_ba)
%SMOOTHDCM2QUAT Converts a batch of DCMs to quaternions while avoiding
% discontinuities.
% Inputs:
% --------
% C_ba: [3 x 3 x N] double
%       list of DCMs
%
% Outputs:
% --------
% q_ba [N x 4] double
%       list of quaternions, where the FIRST component is the scalar.

% Get the quaternion from the DCM
eta = 0.5*(C_ba(1,1,:) + C_ba(2,2,:) + C_ba(3,3,:) + 1).^(0.5);
epsilon(1,:,:) = (C_ba(2,3,:) - C_ba(3,2,:))./(4*eta);
epsilon(2,:,:) = (C_ba(3,1,:) - C_ba(1,3,:))./(4*eta);
epsilon(3,:,:) = (C_ba(1,2,:) - C_ba(2,1,:))./(4*eta);

eta = squeeze(eta);
epsilon = squeeze(epsilon);
q_ba = [eta(:).';epsilon];

% Loop through entire set, taking the dot product of two successive
% quaternions, always picking the quaternion which is closer to the
% previous one (has larger dot product). 
for lv1 = 2:size(q_ba,2)
    dot1 = q_ba(:,lv1).'*q_ba(:,lv1-1);
    dot2 = (-q_ba(:,lv1)).'*q_ba(:,lv1-1);
    
    if dot2 > dot1
        q_ba(:,lv1) = -q_ba(:,lv1);
    end
end
q_ba = q_ba.';
end