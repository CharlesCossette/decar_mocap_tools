%% Test 1 - No noise, no corruption.
m_a = [-4;17;-53];
phis = randn(3,20000)*0.5;
C_bas = rotvecToDcm(phis);

m_b = zeros(3,size(C_bas,3));
for lv1 = 1:size(C_bas,3)
    m_b(:,lv1) = C_bas(:,:,lv1)*m_a;
end

data_synced.mag = m_b;
data_synced.C_ba = C_bas;
data_synced.t = 1:size(C_bas,3);
data_synced.staticIndices = false(size(data_synced.t));
results = magCalibrate(data_synced)


%% Test 2 - No noise, bias.
m_a = [-4;17;-53];
bias = [10;20;30];

phis = randn(3,20000)*0.5;
C_bas = rotvecToDcm(phis);
m_b = zeros(3,size(C_bas,3));
for lv1 = 1:size(C_bas,3)
    m_b(:,lv1) = C_bas(:,:,lv1)*m_a - bias;
end

data_synced.mag = m_b;
data_synced.C_ba = C_bas;
data_synced.t = 1:size(C_bas,3);
data_synced.staticIndices = false(size(data_synced.t));
results = magCalibrate(data_synced)

%% Test 3 - No noise, bias, rotation.
m_a = [-4;17;-53];
bias = [10;20;30];
C_ms = rotvecToDcm([0.1;0.2;0.3]);

phis = randn(3,20000)*0.5;
C_bas = rotvecToDcm(phis);
m_b = zeros(3,size(C_bas,3));
for lv1 = 1:size(C_bas,3)
    m_b(:,lv1) = C_ms.'*C_bas(:,:,lv1)*m_a - bias;
end

data_synced.mag = m_b;
data_synced.C_ba = C_bas;
data_synced.t = 1:size(C_bas,3);
data_synced.staticIndices = false(size(data_synced.t));
results = magCalibrate(data_synced)

%% Test 4 - No noise, bias, rotation, scale.
m_a = [-4;17;-53];
bias = [10;20;30];
C_ms = rotvecToDcm([0.1;0.2;0.3]);
scale = [1.2;1.3;1.4];

phis = randn(3,20000)*0.5;
C_bas = rotvecToDcm(phis);
m_b = zeros(3,size(C_bas,3));
for lv1 = 1:size(C_bas,3)
    m_b(:,lv1) = diag(1./scale)*C_ms.'*C_bas(:,:,lv1)*m_a - bias;
end

[A,b,e] = magcal(m_b.');
m_b = ((m_b.' - b)*A).';

data_synced.mag = m_b;
data_synced.C_ba = C_bas;
data_synced.t = 1:size(C_bas,3);
data_synced.staticIndices = false(size(data_synced.t));
results = magCalibrate(data_synced)


%% Test 5 - Noise, bias, rotation.
m_a = [-4;17;-53];
bias = [10;20;30];
C_ms = rotvecToDcm([0.1;0.2;0.3]);
std_dev = 1;

phis = randn(3,20000)*0.5;
C_bas = rotvecToDcm(phis);
m_b = zeros(3,size(C_bas,3));
for lv1 = 1:size(C_bas,3)
    m_b(:,lv1) = C_ms.'*C_bas(:,:,lv1)*m_a - bias + randn(3,1)*std_dev;
end

data_synced.mag = m_b;
data_synced.C_ba = C_bas;
data_synced.t = 1:size(C_bas,3);
data_synced.staticIndices = false(size(data_synced.t));
results = magCalibrate(data_synced)