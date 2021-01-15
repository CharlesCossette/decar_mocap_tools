function data_corrected = mocapSetBodyFrameZaxisUp(data_mocap, name)
% mocapSetBodyFrameZaxisUp ensures that the z-axis of the body frame of the 
% specified rigid body, as specified by the Mocap system, is pointing upwards.

    data_corrected = data_mocap;
    
    r_up_a = [0;0;1];
    r_up_b = zeros(3,50);
    for lv2 = 1:size(r_up_b,2)
        r_up_b(:,lv2) = data_corrected.(name).C_ba(:,:,lv2)*r_up_a;
    end
    r_up_b = mean(r_up_b,2,'omitnan');
    if norm(r_up_a - r_up_b) > 0.2
        % Then the Z-axis is not up! Assuming y axis is up.
        disp('WARNING: We have detected that the Z axis of the body frame of');
        disp([newline, '>>>>>>>> ', name, ' <<<<<<<<', newline]);
        disp('did not start pointing upwards. Do you want me to change the');
        disp('axes of the body frame so that the Z axis points up at the start?')
        disp('if the body is simply not oriented with Z pointing up.');
        disp(newline)
        is_vertical = input('Response (y/n):','s');
        switch is_vertical
            case {'y','yes'}

                disp(['This correction will be made automatically, assuming',...
                    ' that the Y-axis was in fact the up/vertical one.']);
                q_bprimeb = [-0.5;0.5;0.5;0.5];
                data_corrected.(name).q_ba...
                            = quatMult(q_bprimeb,data_corrected.(name).q_ba);
                data_corrected.(name).C_ba...
                            = quatToDcm(data_corrected.(name).q_ba);
                data_corrected.(name).q_ba...
                            = dcmToQuat(data_corrected.(name).C_ba);

                % Repeat the test again, make sure the problem is fixed.
                for lv2 = 1:size(r_up_b,2)
                    r_up_b(:,lv2) = data_corrected.(name).C_ba(:,:,lv2)*r_up_a;
                end
                r_up_b = mean(r_up_b,2,'omitnan');
                if norm(r_up_a - r_up_b) > 0.2
                    error('DECAR_MOCAP_TOOLS: Program error!') 
                end
            otherwise
                disp('The body frame is left unmodified.')
        end

    end
end