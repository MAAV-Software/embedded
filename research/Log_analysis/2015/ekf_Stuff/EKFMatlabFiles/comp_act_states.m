%comp_act_states.m

%Mimics Step 5 on Page 36, Beard Notes. Note that the estimated states are
%computed using the system noise, and not the sensor noise.

for i=1:1:n-1
    
    fxu=eval_fxu(act_states(:,i),DJI_Fz_RAW(i)/m,DJI_Roll_RAW(i),DJI_Pitch_RAW(i),DJI_YawRate_RAW(i),atan2(Imu_Rot(i,6),Imu_Rot(i,9)), asin(-Imu_Rot(i,3)), atan2(Imu_Rot(i,2), Imu_Rot(i,1)));
    act_states(:,i+1)=act_states(:,i)+T_predict*fxu+system_noise(:,i);
    %Note the addition of the system noise.
    
end
