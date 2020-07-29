function [C, dataAligned] = alignFrames(data, rightHanded)
% A function to deal with left-handed frames and find a suitable initial
% guess for the DCM that relates the MOCAP data and the  corresponding IMU 
% data.
% If rightHanded == true, then it is assumed all frames are right-handed
% and we are only after an initial guess for the DCM.

    if nargin < 2
        rightHanded = false; % by default, assume the possibility of 
                             % left-handed frames
    end
    
    % Create a mesh of all possible rotations of the frame.
    [roll, pitch, yaw] = meshgrid([-pi/2, 0 , pi/2]);
    euler_angles = [roll(:), pitch(:), yaw(:)];
    
    % Create a mesh of all possible sign flips.
    if rightHanded
        signs = [1,1,1];
    else
        [sign1, sign2, sign3] = meshgrid([-1, 1]);
        signs = [sign1(:), sign2(:), sign3(:)];
    end
    
    % Set up function handles for the principal DCMs.
    C1 = @(x) [1, 0, 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
    C2 = @(x) [cos(x) 0 sin(x); 0 1 0; -sin(x) 0 cos(x)];
    C3 = @(x) [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1];

    % Count number of measurements
    numAcc  = length(data.accIMU);   % number of accel meas
    numGyr  = length(data.omegaIMU); % number of gyr meas
    numMeas = numAcc + numGyr;             % total number of accel 
                                           % and gyro meas.)
    
    % Iterate through all possible combinations of DCMs and sign flips
    minJ = Inf; % minimum cost function
    output = zeros(3*numMeas, 1); % error vector
    for lv1=1:1:length(euler_angles)
        % DCM iterate
        Citer = C1(euler_angles(lv1,1))*C2(euler_angles(lv1,2))*C3(euler_angles(lv1,3));

        for lv2=1:1:size(signs,1)
            % flip signs of accelerometers according to the current iterate
            dataIter.accIMU = data.accIMU;
            dataIter.accIMU(1,:) = signs(lv2,1) .* dataIter.accIMU(1,:);
            dataIter.accIMU(2,:) = signs(lv2,2) .* dataIter.accIMU(2,:);
            dataIter.accIMU(3,:) = signs(lv2,3) .* dataIter.accIMU(3,:);
            
            % compute the acceleration components of the error vector
            for lv4=1:1:numAcc
                i = 3*(lv4-1)+1;
                j = 3*lv4;
                output(i:j,:) = Citer * data.accMocap(:,lv4) - dataIter.accIMU(:,lv4);
            end
            
            for lv3=1:1:size(signs,1)
                % flip signs of gyroscops according to the current iterate
                dataIter.omegaIMU = data.omegaIMU;
                dataIter.omegaIMU(1,:) = signs(lv3,1) .* dataIter.omegaIMU(1,:);
                dataIter.omegaIMU(2,:) = signs(lv3,2) .* dataIter.omegaIMU(2,:);
                dataIter.omegaIMU(3,:) = signs(lv3,3) .* dataIter.omegaIMU(3,:);
                
                % compute the omega components of the error vector
                for lv4=1:1:numGyr
                    i = numAcc + 3*(lv4-1)+1;
                    j = numAcc + 3*lv4;
                    output(i:j,:) = Citer * data.omegaMocap(:,lv4) - dataIter.omegaIMU(:,lv4);
                end
                
                % compute the cost function
                J = norm(0.5.*output.'*output);
                
                % check if this results in an improvement
                if J < minJ
                    minJ = J;
                    minIndex1 = lv1; % Euler angles
                    minIndex2 = lv2; % Sign changes of accelerometer data
                    minIndex3 = lv3; % Sign changes of gyroscope data
                end
            end
        end
    end

    % Compute DCM
    C = C1(euler_angles(minIndex1,1))*C2(euler_angles(minIndex1,2))*C3(euler_angles(minIndex1,3));
    
    % For any sign change, update data and log to user
    dataAligned = data;
    if signs(minIndex2,1) == -1
        dataAligned.accIMU(1,:) = -dataAligned.accIMU(1,:);
        disp('Flipped the sign of the x-axis accelerometer data.')
    end
    if signs(minIndex2,2) == -1
        dataAligned.accIMU(2,:) = -dataAligned.accIMU(2,:);
        disp('Flipped the sign of the y-axis accelerometer data.')
    end
    if signs(minIndex2,3) == -1
        dataAligned.accIMU(3,:) = -dataAligned.accIMU(3,:);
        disp('Flipped the sign of the z-axis accelerometer data.')
    end
    if signs(minIndex3,1) == -1
        dataAligned.omegaIMU(1,:) = -dataAligned.omegaIMU(1,:);
        disp('Flipped the sign of the x-axis gyroscope data.')
    end
    if signs(minIndex3,2) == -1
        dataAligned.omegaIMU(2,:) = -dataAligned.omegaIMU(2,:);
        disp('Flipped the sign of the y-axis gyroscope data.')
    end
    if signs(minIndex3,3) == -1
        dataAligned.omegaIMU(3,:) = -dataAligned.omegaIMU(3,:);
        disp('Flipped the sign of the z-axis gyroscope data.')
    end
end