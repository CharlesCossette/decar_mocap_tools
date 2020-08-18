%% Test 1 - Quaternion to DCM
% We compare with the aerospace toolbox.
rng(1) 
q = randn(4,1000);
q = q./vecnorm(q);

C_test = quatToDcm(q);
C = quat2dcm(q.');

assert(all(abs(C_test - C) < 1e-14,'all'))

%% Test 2 - DCM to Quaternion
% Use aerospace toolbox to generate the initial DCMs, convert to
% quaternions. Since quaternions are ambiguous, we need to convert back to
% DCMs to do a proper comparison.
rng(1)
q = randn(4,1000);
q = q./vecnorm(q);
C = quat2dcm(q.');

q_test = dcmToQuat(C);
C_test = quat2dcm(q_test.');

assert(all(abs(C_test - C)< 1e-12,'all'))

%% Test 3 - Quat Rate / Omega
rng(1)
om_ba_b = randn(3,1000);
q_ba = randn(4,1000);
q_ba = q_ba./vecnorm(q_ba);

q_ba_dot = zeros(4,1000);
om_test = zeros(3,1000);
for lv1 = 1:size(q_ba,2)
    q_ba_dot(:,lv1) = omegaToQuatRate(q_ba(:,lv1), om_ba_b(:,lv1));
    om_test(:,lv1) = quatRateToOmega(q_ba(:,lv1), q_ba_dot(:,lv1));
end

assert(all(abs(om_test - om_ba_b) < 1e-12,'all'))

%% Test 4 - Quaternion Multiplication
rng(1)
q = randn(4,1000);
q = q./vecnorm(q);
C_21 = quat2dcm(q.');
C_32 = quat2dcm(q(:,1).');

C_31 = zeros(3,3,1000);
for lv1 = 1:size(C_31,3)
    C_31(:,:,lv1) = C_32*C_21(:,:,lv1);
end

q_31 = quatMult(q(:,1),q);
C_31_test = quat2dcm(q_31.');

assert(all(abs(C_31_test - C_31)< 1e-12,'all'))

%% Test 5 - Quaternion Multiplication
rng(1)
q = randn(4,1000);
q = q./vecnorm(q);
C_32 = quat2dcm(q.');
C_21 = quat2dcm(q(:,1).');

C_31 = zeros(3,3,1000);
for lv1 = 1:size(C_31,3)
    C_31(:,:,lv1) = C_32(:,:,lv1)*C_21;
end

q_31 = quatMult(q,q(:,1));
C_31_test = quat2dcm(q_31.');

assert(all(abs(C_31_test - C_31)< 1e-12,'all'))