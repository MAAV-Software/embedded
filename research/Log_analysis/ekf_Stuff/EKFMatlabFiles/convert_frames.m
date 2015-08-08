%convert_frames.m

%act_yaw=act_states(9,:); %Using the actual yaw to resolve the
%actual positions (px, py, pz) and the corresponding velocities from the 
%yaw frame to the inertial frame.
act_yaw=atan2(Imu_Rot(:,2), Imu_Rot(:,1));

act_yaw_frame_pos=act_states(1:3,:); %Actual yaw-frame positions.
act_yaw_frame_vel=act_states(4:6,:); %Actual yaw-frame velocities.

%est_yaw=est_states(9,:); %Using the estimated yaw to resolve the
%estimated positions (px, py, pz) and the corresponding velocities from the 
%yaw frame to the inertial frame.
est_yaw = act_yaw;
est_yaw_frame_pos=est_states(1:3,:); %Estimated yaw-frame positions.
est_yaw_frame_vel=est_states(4:6,:); %Estimated yaw-frame velocities.

for i=1:1:n
    
    %When resolving the positions and velocities, note that the transposes
    %of the yaw-rotation matrices are used. 
    
    act_C_yaw=[cos(act_yaw(i)),sin(act_yaw(i)),0;
               -1*sin(act_yaw(i)),cos(act_yaw(i)),0;
               0,0,1]; %Yaw rotation matrix, a 1-axis rotation (see the
                       %Beard notes).
    act_inertial_pos(:,i)=act_C_yaw'*act_yaw_frame_pos(:,i);
    %Resolving the actual positions.
    act_inertial_vel(:,i)=act_C_yaw'*act_yaw_frame_vel(:,i);
    %Resolving the actual velocities.
           
    est_C_yaw=[cos(est_yaw(i)),sin(est_yaw(i)),0;
               -1*sin(est_yaw(i)),cos(est_yaw(i)),0;
               0,0,1]; %Yaw rotation matrix, a 1-axis rotation (see the
                       %Beard notes).
    est_inertial_pos(:,i)=est_C_yaw'*est_yaw_frame_pos(:,i);
    %Resolving the estimated positions.
    est_inertial_vel(:,i)=est_C_yaw'*est_yaw_frame_vel(:,i);
    %Resolving the estimated velocities.

end       
            