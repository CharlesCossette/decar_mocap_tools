function q_ba = dcmToQuat(C_ba)
%DCMTOQUAT Converts a batch of DCMs to quaternions while avoiding
% discontinuities, unlike the dcm2quat() function in the aerospace toolbox.
% Furthermore, this is compatible with the complex step.
%
% PARAMETERS:
% --------
% C_ba: [3 x 3 x N] double
%       list of DCMs
%
% RETURNS:
% --------
% q_ba [4 x N] double
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

q_ba = q_ba./(sum(q_ba.*q_ba,1).^0.5);
end