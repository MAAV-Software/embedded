%comp_est_states.m

%Page 36, Beard Notes. Note that the estimated states are computed using
%the sensor noise, and not the system noise.

count=0; %A counter that controls whether a prediction step or 
%correction step occurs.
correction_step_count=0; %A counter that keeps track of the number of 
%correction steps that have occurred. 
P_init=zeros(n_states,n_states); %Assuming an initial P matrix of zeroes.
P=P_init;

for i=1:1:n-1
    
    if count==N %Correction (Sensor) Update
        
        correction_step_count=correction_step_count+1;
        
        %Computing the values of the estimated states but not placing them
        %into the estimated-states vector unless the booleans in the 
        %correction_bool and camera_correction_bool vectors are equal to
        %one:
        L=P*C'*inv(R+C*P*C'); %Step 11.
        P=(eye(n_states)-L*C)*P; %Step 12.
        
%        est_states_nom=est_states(:,i)+L*(act_states(:,i)+sensor_noise(:,i)-C*est_states(:,i)); %Step 13.
        
        sensor_states(:,i) = [Px4_Xdot(i), Px4_Ydot(i), Lidar_Dist(i)]';
        est_states_nom=est_states(:,i)+L*(sensor_states(:,i)-C*est_states(:,i));
        %Using all sensors other than the camera to update (correct) all
        %states other than the px and py positions and the yaw angle:
        if correction_bool(correction_step_count)==1
%             est_states(3:8,i+1)=est_states_nom(3:8);
             est_states(3:6,i+1)=est_states_nom(3:6);
        end
        
        %Using the camera to update (correct) the px and py positions and
        %the yaw angle:
        if camera_correction_bool(correction_step_count)==1
            est_states(1:2,i+1)=est_states_nom(1:2);
%            est_states(9,i+1)=est_states_nom(9);
        end
        
        %If no sensor updates (corrections) occur, perform a prediction 
        %step instead:
        if (correction_bool(correction_step_count)==0 && camera_correction_bool(correction_step_count)==0)
           
%           fxu=eval_fxu(est_states(:,i),az(i),p(i),q(i),r(i)); %Step 5.
            fxu=eval_fxu(est_states(:,i),DJI_Fz_RAW(i),DJI_Roll_RAW(i),DJI_Pitch_RAW(i),DJI_YawRate_RAW(i),atan2(Imu_Rot(i,6),Imu_Rot(i,9)), asin(-Imu_Rot(i,3)), atan2(Imu_Rot(i,2), Imu_Rot(i,1)));
            est_states(:,i+1)=est_states(:,i)+T_predict*fxu; %Step 5.
            %Note the absence of the system noise.
%            A=eval_A(est_states(:,i),az(i),p(i),q(i),r(i)); %Step 6.
            A = zeros(6,6);
            A(1,4) = 1; A(2,5) = 1; A(3,6) = 1;
            P=P+T_predict*(A*P+P*A'+G*Q*G'); %Step 7.
        
        end
        
        count=0;
        continue;
    
    end
    
    %Otherwise, Prediction (No Sensor) Update
    
%    fxu=eval_fxu(est_states(:,i),az(i),p(i),q(i),r(i)); %Step 5.
    fxu=eval_fxu(est_states(:,i),DJI_Fz_RAW(i),DJI_Roll_RAW(i),DJI_Pitch_RAW(i),DJI_YawRate_RAW(i),atan2(Imu_Rot(i,6),Imu_Rot(i,9)), asin(-Imu_Rot(i,3)), atan2(Imu_Rot(i,2), Imu_Rot(i,1)));
           
    est_states(:,i+1)=est_states(:,i)+T_predict*fxu; %Step 5.
    %Note the absence of the system noise.
%    A=eval_A(est_states(:,i),az(i),p(i),q(i),r(i)); %Step 6.
    A = zeros(6,6);
    A(1,4) = 1; A(2,5) = 1; A(3,6) = 1;
    P=P+T_predict*(A*P+P*A'+G*Q*G'); %Step 7.
    
    count=count+1;
    
end
