function varargout = mocap_getPointInBodyFrame(data_mocap, rigid_body_names, marker_names)
%MOCAP_GETPOINTINBODYFRAME Returns the location of a specified marker(s)
% relative to the "pivot point" (i.e. point z) on the body as specified in
% the Optitrack Motion capture system, resolved in the mocap body frame.
% This function can also be used to return multiple points at once by
% specifying rigidBodyNames and markersNames as cell arrays
%
%
% Examples:
%
% r_pz_b = ...
% mocap_getPointInBodyFrame('mocap_data.csv','RigidBody002','Unlabeled1341')
%
% r_pz_b = ...
% mocap_getPointInBodyFrame(dataMocap,'RigidBody002','Unlabeled1341')
%
% [r_p1z_b, r_p2z_b] = ...
% mocap_getPointInBodyFrame(dataMocap,{'RigidBody001', 'RigidBody002'},...
%                                     {'Unlabeled1341','Unlabeled1340'})
%
% Inputs:
% --------
% dataMocap: [struct] or [string]
%       Mocap raw data as returned by the function mocap_csv2struct(), or
%       alternatively the filename containing the mocap data.
% rigidBodyNames: [string] or [N x 1 cell array]
%       Single string specifying the name of the rigid body, or cell array
%       of rigid body names in which to resolve the marker locations
%       specified in the following argument.
% markerNames: [string] or [N x 1 cell array]
%       Single string specifiying the marker name that you want the
%       location of, relative to the pivot point of the rigid body
%       specified in the previous argument. If specified as a string, the
%       i^th markerName will be relative to the i^th rigid body name of the
%       previous argument.
%
% Outputs:
% --------
% varargout: Number of out argments = length(markerNames)
%       Positions of markerNames relative to their respective rigid body
%       "pivot points" (point z) resolved in their respective rigid body
%       frames.

    % Extract from csv if filename was supplied.
    if isa(data_mocap,'char') || isa(data_mocap,'string')
        data_mocap = mocap_csv2struct(data_mocap);
    elseif ~isa(data_mocap,'struct')
        error('First argument must either be a struct or a filename.')
    end

    if nargin == 1
        varargout = graphicalPointSelection(data_mocap);
    else
        if isa(rigid_body_names,'char') ||isa(rigid_body_names,'string')
            rigid_body_names = {rigid_body_names};
        end

            if isa(marker_names,'char') ||isa(marker_names,'string')
                marker_names = {marker_names};
            end
            nOutputs  = numel(marker_names);
            varargout = cell(1,nOutputs);

            assert(length(rigid_body_names) == length(marker_names))

            % Extract the relative position from the desired marker to point z for
            % each rigid body specified and a corresponding label.
            for lv1=1:1:length(rigid_body_names)
                rigidBodyName = rigid_body_names{lv1};
                markerName = marker_names{lv1};
                varargout{lv1} = getAverageRelPos(data_mocap, rigidBodyName, markerName);
            end
    end
end

function output = graphicalPointSelection(data_mocap)
% Continuously ask user for rigidbody,marker pairs
% To be honest, this is not very useful.    
    mocapShowMarkers(data_mocap);

    counter = 1;
    while true
        disp('----------------------------------------------------------------')
        reply = input('Choose point. Specify as <rigid_body_name>,<marker_name>:\n','s');
        if strcmp(reply,'done') || strcmp(reply,'quit') || strcmp(reply,'')
            break
        end
        comma_loc = strfind(reply, ',');
        rigid_body_name = reply(1:comma_loc-1);
        marker_name = reply(comma_loc+1:end);
        r_pz_b = getAverageRelPos(data_mocap, rigid_body_name, marker_name);
        output{counter} = r_pz_b;
        counter = counter + 1;
    end
end

function r_pz_b = getAverageRelPos(data_mocap, rigid_body_name, marker_name)
% Gets the relative position between a specified marker and the pivot point
% of a given rigid body.

    r_pz_b = zeros(3,size(data_mocap.(rigid_body_name).t,1));
    n_timepoints = max(size(data_mocap.(rigid_body_name).t,1),500);
    for lv2 = 1:n_timepoints
        C_ba = data_mocap.(rigid_body_name).C_ba(:,:,lv2);
        r_zw_a = data_mocap.(rigid_body_name).r_zw_a(:,lv2);
        r_pw_a = data_mocap.(marker_name).r_zw_a(:,lv2);
        r_pz_b(:,lv2) = C_ba*(r_pw_a - r_zw_a);
    end
    r_pz_b =  mean(r_pz_b,2,'omitnan');  
    if norm(r_pz_b) > 1
        warning(['DECAR_MOCAP_TOOLS: Relative position more than 1m away, are ',...
                 'sure this is correct?'])
    end
end