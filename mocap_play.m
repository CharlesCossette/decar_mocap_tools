function mocap_play(dataMocap)

    % Extract only rigid bodies
    fieldnames = fields(dataMocap);
    rigidBodies = {};
    for lv1 = 1:numel(fieldnames)
        if strcmp(dataMocap.(fieldnames{lv1}).type, 'Rigid Body')
            rigidBodies = [rigidBodies; {dataMocap.(fieldnames{lv1})}];
        end
    end

    ani = Animation();
    for lv1 = 1:numel(rigidBodies)
        rgb = rigidBodies{lv1};
        triad = AnimatedTriad();
        ani.addElement(triad);
    end
    figure(1)
    clf
    ani.build()
    %axis([-4 4 -2.5 2.5 0 3])
    %axis([-10 10 -10 10 -10 10]*0.5)
    xlabel('$x$ (m)','interpreter','latex','fontsize',15)
    ylabel('$y$ (m)','interpreter','latex','fontsize',15)
    zlabel('$z$ (m)','interpreter','latex','fontsize',15)
    rgb = rigidBodies{1};
    for lv1 = 1:10:numel(rgb.t)
        r = zeros(3,numel(rigidBodies));
        C = repmat(eye(3),1,1,numel(rigidBodies));
        for lv2 = 1:numel(rigidBodies)
            rgb = rigidBodies{lv2};
            r(:,lv2) = rgb.r_zw_a(:,lv1);
            C(:,:,lv2) = rgb.C_ba(:,:,lv1);
        end
        ani.update(r,C);
        drawnow;
    end
end