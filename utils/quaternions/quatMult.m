function q_31 = quatMult(q_32, q_21)
%QUATMULT A vectorized implementation of quaternion multiplication. The
% formulas are taken from (1.49) of Spacecraft Dynamics and Control:
% An Introduction
% de Ruiter, Anton H.J. ;Damaren, Christopher J.; Forbes, James R.
%
% If q_21 is the quaternion parameterization of C_21, then this function
% returns q_31 where C_31 = C_32*C_21;
%
% We use slightly different formulas depending on which of q_32, q_21 are
% sent as the list of quaternions. This is strictly for vectorization
% reasons, the underlying operations are equivalent.
%
% PARAMETERS:
% -----------
% q_32: [4 x N] double
%       array of unit quaternions, where each column is a different
%       quaternion. 
% q_21: [4 x N] double
%       array of unit quaternions, where each column is a different
%       quaternion. If only one quaternion is suppled for this argument,
%       each quaternion in q_32 will be multiplied by q_21. The reverse
%       also applies. 
%
% RETURNS:
% ---------
% q_31: [4 x N] double
%       array of unit quaternions corresponding to the same rotation as
%       C_31, as described above.

if size(q_32,2) > 1
    etas_32 = q_32(1,:);
    eps_32 = q_32(2:4,:);
    eps_21 = q_21(2:4);
    eta_21 = q_21(1);
    etas_31 = eta_21*etas_32 - eps_21.'*eps_32; % Formulas from above.
    eps_31 = etas_32.*eps_21 + eta_21*eps_32 + crossOp(eps_21)*eps_32;
    q_31 = [etas_31;eps_31];
elseif size(q_21,2) > 1
    etas_32 = q_32(1);
    eps_32 = q_32(2:4);
    eps_21 = q_21(2:4,:);
    eta_21 = q_21(1,:);
    etas_31 = eta_21*etas_32 - eps_32.'*eps_21; % Formulas from above.
    eps_31 = etas_32.*eps_21 + eps_32*eta_21 - crossOp(eps_32)*eps_21;
    q_31 = [etas_31;eps_31];
end
end

function X = crossOp(v)
    % compute the cross operator X of a vector v
     X = [    0, -v(3),  v(2);
           v(3),     0, -v(1);
          -v(2),  v(1),    0];
end