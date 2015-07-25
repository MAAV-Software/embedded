%comp_act_states.m

%Mimics Step 5 on Page 36, Beard Notes. Note that the estimated states are
%computed using the system noise, and not the sensor noise.

for i=1:1:n-1
    
    fxu=eval_fxu(act_states(:,i),az(i),p(i),q(i),r(i));
    act_states(:,i+1)=act_states(:,i)+T_predict*fxu+system_noise(:,i);
    %Note the addition of the system noise.
    
end
