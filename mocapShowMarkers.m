function mocapShowMarkers(data_mocap)
%MOCAPSHOWMARKERS Shows a visual scene of all the rigid bodies along with
%the location of unlabeled markers. Can be used to identity locations of
%sensors on the rigid bodies.
%
% PARAMETERS:
% -----------
% data_mocap: struct
%       struct as generated by mocapCsvToScruct()
 
    if isa(data_mocap,'char') || isa(data_mocap,'string')
        data_mocap = mocap_csv2struct(data_mocap);
    elseif ~isa(data_mocap,'struct')
        error('First argument must either be a struct or a filename.')
    end
    fieldnames = fields(data_mocap);
    
    % Plot visual scene to aid selection.
    figure(1)
    clf;
    hold on
    for lv1 = 1:numel(fieldnames)
        switch data_mocap.(fieldnames{lv1}).type
            case 'Rigid Body'
                rgb = data_mocap.(fieldnames{lv1});
                r = rgb.r_zw_a(:,120);
                C = rgb.C_ba(:,:,5);
                triad = AnimatedTriad();
                triad.plot(r,C);
                text(r(1), r(2), r(3) + 1.1, fieldnames{lv1},...
                    'FontSize',18, 'FontWeight', 'bold')
            case 'Marker'
                marker = data_mocap.(fieldnames{lv1});
                r = marker.r_zw_a(:,120);
                C = eye(3);
                sph = AnimatedSphere();
                sph.radius = 0.01;
                sph.plot(r,C)
                if contains(fieldnames{lv1}, 'Unlabeled')
                    text(r(1)+0.01, r(2)+0.01, r(3) + 0.01, fieldnames{lv1}, 'FontWeight', 'bold')
                else
                    text(r(1)+0.01, r(2)+0.01, r(3) + 0.01, fieldnames{lv1}, 'FontSize', 8)
                end
        end

       
    end
    axis equal
    axis vis3d
    grid on
    view(3)
end