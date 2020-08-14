function q_31 = quatmul(q_32, q_21)
%QUATMUL A vectorized implementation of quaternion multiplication. The
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
% Allows either q_32 to be a batch of quaternions, or q_21, but not both.
if size(q_32,2) > 1
    etas_32 = q_32(1,:);
    eps_32 = q_32(2:4,:);
    eps_21 = q_21(2:4);
    eta_21 = q_21(1);
    etas_31 = eta_21*etas_32 - eps_21.'*eps_32; % Formulas from above.
    eps_31 = etas_32.*eps_21 + eta_21*eps_32 + CrossOperator(eps_21)*eps_32;
    q_31 = [etas_31;eps_31];
elseif size(q_21,2) > 1
    etas_32 = q_32(1);
    eps_32 = q_32(2:4);
    eps_21 = q_21(2:4,:);
    eta_21 = q_21(1,:);
    etas_31 = eta_21*etas_32 - eps_32.'*eps_21; % Formulas from above.
    eps_31 = etas_32.*eps_21 + eps_32*eta_21 - CrossOperator(eps_32)*eps_21;
    q_31 = [etas_31;eps_31];
end
end