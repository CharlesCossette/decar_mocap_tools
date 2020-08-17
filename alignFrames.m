function [dataAligned, C_am, C_gm] = alignFrames(dataSynced, rightHandedOnly)
% If rightHanded == true, then transformations are restricted to be
% positive-determinant DCMs.

    if nargin < 2
        rightHandedOnly = false; % by default, assume the possibility of 
                                 % left-handed frames
    end
    
    C_all = getAllPermutationMatrices(rightHandedOnly);

    % We now have a set of all the possible DCMs corresponding to different
    % axis switches, including left-handed frames.
    % Loop through all the DCMs and try every single one. Find the DCMs 
    % with the lowest error. 
    % We evaluate accel and gyro completely seperately, as it is possible
    % for them to be using different frames.
    J_best_accel = inf;
    J_best_gyro = inf;
    C_ma_best = eye(3); % C_am is a dcm from mocap body frame to accel frame.
    C_mg_best = eye(3); % C_gm is a dcm from mocap body frame to gyro frame.
    for lv1 = 1:size(C_all,3)
        C = C_all(:,:,lv1);
        
        % Evaluate the accelerometer error, check if new best.
        e_accel = computeErrorAccel(C,dataSynced);
        J_accel = 0.5*(e_accel.'*e_accel);
        if J_accel < J_best_accel
            C_ma_best = C;
            J_best_accel = J_accel;
        end
        
        % Evaluate the gyroscope error, check if new best.
        e_gyro = computeErrorGyro(C,dataSynced);
        J_gyro = 0.5*(e_gyro.'*e_gyro);
        if J_gyro < J_best_gyro
            C_mg_best = C;
            J_best_gyro = J_gyro;
        end
    end
    
    if any(C_ma_best ~= C_mg_best,'all')
        disp('Different accel/gyro frames detected!')
        disp('Gyro will be transformed to the same frame as accel.')
        
    end
    disp('Best fit accelerometer DCM:')
    C_ma_best
    disp('Best fit gyroscope DCM:')
    C_mg_best
    
    dataAligned = dataSynced;
    dataAligned.accel = C_ma_best*dataSynced.accel;
    dataAligned.gyro = C_mg_best*dataSynced.gyro;
    C_am = C_ma_best;
    C_gm = C_mg_best;
        
end

function output = computeErrorAccel(C_ms,dataSynced)
    gaps = dataSynced.gapIndices;
    error_accel = dataSynced.accel_mocap(:,~gaps) - C_ms*dataSynced.accel(:,~gaps);
    output = error_accel(:);
end
function output = computeErrorGyro(C_ms,dataSynced)
    gaps = dataSynced.gapIndices;
    error_omega = dataSynced.gyro_mocap(:,~gaps) - C_ms*dataSynced.gyro(:,~gaps);
    output = error_omega(:);
end