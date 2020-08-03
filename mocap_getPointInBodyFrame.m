function varargout = mocap_getPointInBodyFrame(dataMocap, rigidBodyNames, markerNames)
%MOCAP_GETPOINTINBODYFRAME Returns the location of a specified marker(s)
% relative to the "pivot point" (i.e. point z) on the body as specified in
% the Optitrack Motion capture system, resolved in the mocap body frame.
% This function can also be used to return multiple points at once by
% specifying rigidBodyNames and markersNames as cell arrays
%
%
% Examples:
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
% rigidBodyNames: [string] or [cell array]
%       Single string specifying the name of the rigid body, or cell array
%       of rigid body names in which to resolve the marker locations
%       specified in the following argument.
% markerNames: [string] or [cell array]
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

if isa(dataMocap,'char') || isa(dataMocap,'string')
    dataMocap = mocap_csv2struct(dataMocap);
elseif ~isa(dataMocap,'struct')
    error('First argument must either be a struct or a filename.')
end

if isa(rigidBodyNames,'char') ||isa(rigidBodyNames,'string')
    rigidBodyNames = {rigidBodyNames};
end

if isa(markerNames,'char') ||isa(markerNames,'string')
    markerNames = {markerNames};
end
nOutputs  = numel(markerNames);
varargout = cell(1,nOutputs);

assert(length(rigidBodyNames) == length(markerNames))

% Extract the distance from the reference point to the IMU for each
% rigid body speciried and a corresponding label
for lv1=1:1:length(rigidBodyNames)
    rigidBodyName = rigidBodyNames{lv1};
    markerName = markerNames{lv1};
    r_pz_b = zeros(3,size(dataMocap.(rigidBodyName).t,1));
    for lv2 = 1:size(dataMocap.(rigidBodyName).t,1)
        C_ba = dataMocap.(rigidBodyName).C_ba(:,:,lv2);
        r_zw_a = dataMocap.(rigidBodyName).r_zw_a(:,lv2);
        r_pw_a = dataMocap.(markerName).r_zw_a(:,lv2);
        r_pz_b(:,lv2) = C_ba*(r_pw_a - r_zw_a);
    end
    varargout{lv1} = mean(r_pz_b,2,'omitnan');  
end

end