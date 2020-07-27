function r_pz_b = mocap_getPointInBodyFrame(dataMocap, rigidBodyName, markerName)
% Example:
% r_pz_b =
% mocap_getPointInBodyFrame(dataMocap,'RigidBody002','Unlabeled1341')
%
r_pz_b = zeros(3,size(dataMocap.(rigidBodyName).t,1));
for lv1 = 1:size(dataMocap.(rigidBodyName).t,1)
    C_ba = dataMocap.(rigidBodyName).C_ba(:,:,lv1);
    r_zw_a = dataMocap.(rigidBodyName).r_zw_a(:,lv1);
    r_pw_a = dataMocap.(markerName).r_zw_a(:,lv1);
    r_pz_b(:,lv1) = C_ba*(r_pw_a - r_zw_a);
end
r_pz_b = mean(r_pz_b,2,'omitnan');

end