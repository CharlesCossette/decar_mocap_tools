function [dataAligned, C_am, C_gm] = alignFrames(dataSynced, rightHanded)
% If rightHanded == true, then transformations are restricted to be
% positive-determinant DCMs.

    if nargin < 2
        rightHanded = false; % by default, assume the possibility of 
                             % left-handed frames
    end
    
    % Create a mesh of all possible rotations of the frame.
    [roll, pitch, yaw] = meshgrid([-pi/2, 0 , pi/2]);
    [sign1, sign2, sign3] = meshgrid([-1, 1]);
    angs = [roll(:),pitch(:), yaw(:)];
    signs = [sign1(:),sign2(:),sign3(:)];
    
    % Set up function handles for the principal DCMs.
    C1 = @(x) [1, 0, 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
    C2 = @(x) [cos(x) 0 sin(x); 0 1 0; -sin(x) 0 cos(x)];
    C3 = @(x) [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1];
    
    % Generate all possible right-handed DCMs
    C_list = zeros(3,3,size(angs,1)+1);
    
    for lv1 = 1:size(angs,1)
        C_list(:,:,lv1) = C1(angs(lv1,1))*C2(angs(lv1,2))*C3(angs(lv1,3));
    end
    
    % Strip duplicates
    C_list = round(C_list); % round to eliminate numerical errors
    C_list_rows = mat3D2rows(C_list);
    C_list_rows = unique(C_list_rows,'rows');
    C_list = rows2mat3D(C_list_rows);
    
    % For each of the right handed DCMs, create all possible left-handed
    % DCMs
    if rightHanded
        C_all = C_list;
    else
        C_all = zeros(3,3,size(C_list,3)*size(signs,1));
        counter = 1;
        for lv1 = 1:size(C_list,3)
            for lv2 = 1:size(signs,1)
                C_all(:,:,counter) = C_list(:,:,lv1).*signs(lv2,:);
                counter = counter + 1;
            end
        end
        % Strip duplicates again
        C_all = round(C_all); % round to eliminate numerical errors
        C_all_rows = mat3D2rows(C_all);
        C_all_rows = unique(C_all_rows,'rows');
        C_all = rows2mat3D(C_all_rows);
    end
    

    % We now have a set of all the possible DCMs corresponding to different
    % axis switches, including left-handed frames.
    % Loop through all the DCMs and try every single one. Find the DCMs 
    % with the lowest error. 
    % We evaluate accel and gyro completely seperately, as it is possible
    % for them to be using different frames.
    J_best_accel = inf;
    J_best_gyro = inf;
    C_am_best = eye(3); % C_am is a dcm from mocap body frame to accel frame.
    C_gm_best = eye(3); % C_gm is a dcm from mocap body frame to gyro frame.
    for lv1 = 1:size(C_all,3)
        C = C_all(:,:,lv1);
        
        % Evaluate the accelerometer error, check if new best.
        e_accel = computeErrorAccel(C,dataSynced);
        J_accel = 0.5*(e_accel.'*e_accel);
        if J_accel < J_best_accel
            C_am_best = C;
            J_best_accel = J_accel;
        end
        
        % Evaluate the gyroscope error, check if new best.
        e_gyro = computeErrorGyro(C,dataSynced);
        J_gyro = 0.5*(e_gyro.'*e_gyro);
        if J_gyro < J_best_gyro
            C_gm_best = C;
            J_best_gyro = J_gyro;
        end
    end
    
    if any(C_am_best ~= C_gm_best,'all')
        disp('Different accel/gyro frames detected!')
        disp('Gyro will be transformed to the same frame as accel.')
        
    end
    disp('Best fit accelerometer DCM:')
    C_am_best
    disp('Best fit gyroscope DCM:')
    C_gm_best
    
    dataAligned = dataSynced;
    dataAligned.accIMU = C_am_best.'*dataSynced.accIMU;
    dataAligned.omegaIMU = C_gm_best.'*dataSynced.omegaIMU;
    C_am = C_am_best;
    C_gm = C_gm_best;
        
end

function C_rows = mat3D2rows(Cs)
    % Takes a [3 x 3 x N] matrix of DCMs and returns a [N x 9] matrix where
    % each row is the columns of the DCM stacked in a [1 x 9]
    n = size(Cs,1);
    m = size(Cs,2);
    C_rows = zeros(size(Cs,3),n*m);
    for lv1 = 1:size(Cs,3)
        C = Cs(:,:,lv1);
        C_rows(lv1,:) = C(:).'; 
    end
end

function Cs = rows2mat3D(C_rows)
    % Takes a [N x 9] matrix where each row is the columns of the DCM 
    % stacked in a [1 x 9] and returns the DCMs in a [3 x 3 x N] matrix.
    n = 3;
    m = 3;
    Cs = zeros(n,m,size(C_rows,1));
    for lv1 = 1:size(C_rows,1)
        Cs(:,:,lv1) = reshape(C_rows(lv1,:),n,m);
    end
end

function output = computeErrorAccel(C_sm,dataSynced)
    error_accel = C_sm*dataSynced.accMocap - dataSynced.accIMU;
    output = [error_accel(:)];
end
function output = computeErrorGyro(C_sm,dataSynced)
    error_omega = C_sm*dataSynced.omegaMocap - dataSynced.omegaIMU;
    output = [error_omega(:)];
end