function mocapPlay(data_mocap)
%MOCAPPLAY Plays an animation of a specific mocap data files.
%
% Example uses:
%    
%    mocapPlay(data_mocap)
%    mocapPlay('mocap_data_file.csv')
%
% Requires decar_animate.
    if isa(data_mocap,'char') || isa(data_mocap,'string')
        data_mocap = mocap_csv2struct(data_mocap);
    elseif ~isa(data_mocap,'struct')
        error('First argument must either be a struct or a filename.')
    end
    
    
    % Extract only rigid bodies
    fieldnames = fields(data_mocap);
    rigid_bodies = {};
    markers = {};
    for lv1 = 1:numel(fieldnames)
        switch data_mocap.(fieldnames{lv1}).type
            case 'Rigid Body'
                rigid_bodies = [rigid_bodies; {data_mocap.(fieldnames{lv1})}];
            case 'Marker'
                markers = [markers; {data_mocap.(fieldnames{lv1})}];
        end
       
    end

    ani = Animation();
    for lv1 = 1:numel(rigid_bodies)
        triad = AnimatedTriad();
        ani.addElement(triad);
    end
    for lv1 = 1:numel(markers)
        sph = AnimatedSphere();
        sph.radius = 0.05;
        ani.addElement(sph);
    end
    
    figure(1)
    clf
    ani.build()
    %axis([-4 4 -2.5 2.5 0 3])
    %axis([-10 10 -10 10 -10 10]*0.5)
    xlabel('$x$ (m)','interpreter','latex','fontsize',15)
    ylabel('$y$ (m)','interpreter','latex','fontsize',15)
    zlabel('$z$ (m)','interpreter','latex','fontsize',15)
    rgb = rigid_bodies{1};
    for lv1 = 1:10:numel(rgb.t)
        r = zeros(3,numel(rigid_bodies) + numel(markers));
        C = repmat(eye(3),1,1,numel(rigid_bodies) + numel(markers));
        counter = 1;
        for lv2 = 1:numel(rigid_bodies)
            rgb = rigid_bodies{lv2};
            r(:,counter) = rgb.r_zw_a(:,lv1);
            C(:,:,counter) = rgb.C_ba(:,:,lv1);
            counter = counter + 1;
        end
        for lv2 = 1:numel(markers)
            marker = markers{lv2};
            r(:,counter) = marker.r_zw_a(:,lv1);
            C(:,:,counter) = eye(3);
            counter = counter + 1;
        end
        ani.update(r,C);
        drawnow;
    end
end