function S_corrected = mocapSetBodyFrameZaxisUp(S, name)
% mocapSetBodyFrameZaxisUp ensures that the z-axis of the body frame of the 
% specified rigid body, as specified by the Mocap system, is pointing upwards.

    S_corrected = S;
    
    r_up_a = [0;0;1];
    r_up_b = zeros(3,50);
    for lv2 = 1:size(r_up_b,2)
        r_up_b(:,lv2) = S.(name).C_ba(:,:,lv2)*r_up_a;
    end
    r_up_b = mean(r_up_b,2,'omitnan');
    if norm(r_up_a - r_up_b) > 0.2
        % Then the Z-axis is not up! Assuming y axis is up.
        disp('WARNING: We have detected that the Z axis of the body frame of');
        disp([newline, '>>>>>>>> ', name, ' <<<<<<<<', newline]);
        disp('did not start pointing upwards. We cannot tell if this is');
        disp('because the body frame was not defined with Z pointing up, or ')
        disp('if the body is simply not oriented with Z pointing up.');
        disp(newline)
        is_vertical = input(['Was ', name,' vertical (i.e. intended Z-up) ',...
                            newline, ' at the start of the take? (y/n)'],'s');
        switch is_vertical
            case {'y','yes'}

                disp(['This correction will be made automatically, assuming',...
                    ' that the Y-axis was in fact the up/vertical one.']);
                q_bprimeb = [-0.5;0.5;0.5;0.5];
                S.(name).q_ba = quatMult(q_bprimeb,S.(name).q_ba);
                S.(name).C_ba = quatToDcm(S.(name).q_ba);
                S.(name).q_ba = dcmToQuat(S.(name).C_ba);

                % Repeat the test again, make sure the problem is fixed.
                for lv2 = 1:size(r_up_b,2)
                    r_up_b(:,lv2) = S.(name).C_ba(:,:,lv2)*r_up_a;
                end
                r_up_b = mean(r_up_b,2,'omitnan');
                if norm(r_up_a - r_up_b) > 0.2
                    error('Program error!') 
                end
            otherwise
                disp('The body frame is left unmodified.')
        end

    end
end