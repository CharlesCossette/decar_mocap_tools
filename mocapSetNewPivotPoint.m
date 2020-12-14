function data_mocap = mocapSetNewPivotPoint(data_mocap, rigid_body_name, pos_shift)
%MOCAPSETNEWPIVOTPOINT Modifies the position data inside a data_mocap
%struct to be the position of a new "origin", "pivot point", or, as our
%group calls it, "point z". i.e. a reference point on the body.
%
% PARAMETERS:
% -----------
% data_mocap: struct
%       Mocap data as extracted by mocapCsvToStruct()
% rigid_body_name: string
%       target rigid body to apply to modification, as named by the mocap.
% pos_shift: [3 x 1] double
%       Position shift IN THE BODY FRAME of <rigid_body_name>. I.e. this
%       quantity should be r_pz_b, where p will be the new reference point.
% 
% RETURNS:
% --------
% data_mocap: struct
%       Supplied mocap data but with the modification.

for lv1=1:1:length(data_mocap.(rigid_body_name).t)
    data_mocap.(rigid_body_name).r_zw_a(:,lv1) ...
        = data_mocap.(rigid_body_name).r_zw_a(:,lv1)...
            + data_mocap.(rigid_body_name).C_ba(:,:,lv1)'*pos_shift;
end

end