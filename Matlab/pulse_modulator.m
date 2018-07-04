%%%%%% problems waiting for solutions (incomplete code) %%%%%%%
% - check availability of thrusters
%% symbol 
s=tf('s');
Var=[1.632119 0.002449]; % variance of noise for yaw and gz
Dev=sqrt(Var)/180*pi;

%% Initialization
%%%%%%%%%%%%%% time %%%%%%%%%%%%%%
sensor_period=0.1;
drawing_period=0.01; 
end_time=60;

testing_period=0:drawing_period:end_time;
data_samplingNumber=size(testing_period,2);
sensor_samplingNumber=end_time/sensor_period;
scale_sensor2data=sensor_period/drawing_period;

%%%%%%%%%%%%%%% plant %%%%%%%%%%%%%%%
% assuming that center of mass is at geometric center 
m=16; %mass kg
w=0.239; %width meter
l=0.464; %length meter
I=1/12*m*(w^2+l^2); %moment of inertia (kg*meter^2)
Sys=tf(1,[I 0]); %system transfer function
angu_v_initial=0/180*pi; %rad/sec
angle_initial=30/180*pi; %rad. Must between -180~180

%%%%%%%%%%%%% thruster %%%%%%%%%%%%%%
Sys_thrust=tf(1,1); %tf(4800,[1 140 4800]); %thruster perturbation
thrust_M_design=0.4*(l/2)*(3/4); % desirable moment produced by thruster
thrust_M=0.5*(l/2)*(3/4); % actual moment produced by thruster
on_duration_min=0.03; %0.03;
off_duration_min=0.03; %0.03;
Control_value_min=thrust_M_design*on_duration_min/(on_duration_min+off_duration_min);

%%%%%%%%%%%%%%% sensor %%%%%%%%%%%%%%
K_angle=0.3; %feedback gain for angle position   
K_angu_v=0.6; %feedback gain for angular velocity

%%%%%%%% controller %%%%%%%%
uplimit=thrust_M_design;
%angle_threshold=0.5/180*pi;
deadband=uplimit*0.25; %angle_threshold*K_angle;

%%%%%%%%%%%%%% disturbance value %%%%%%%%%%%%%%
M_passive_dist_value=0.01764;

%%%%%%%%%%%%%% expectation and user input %%%%%%%%%%%%%%
Expectation=0; % the desired value for the control loop input.  
Desired_angle_input = 0/180*pi; %the orientation we want to achieve. Must between -180~180
Range_adj = 175/180*pi; % range for adjustment to make the angles around 180 and -180 continuous

%%%%%%%% Adjustment: to make the angles around 180 and -180 continuous %%%%%%%%
if Desired_angle_input < -Range_adj 
    Desired_angle = Desired_angle_input+2*pi;
else
    Desired_angle = Desired_angle_input;
end

%% control loop
Control_value_record=zeros(1,data_samplingNumber); %to record the control value during testing perioed.
%Control_value_thrust_record=zeros(1,data_samplingNumber); %to record the control value into pulse modulator during testing perioed.
Actuator_record=zeros(1,data_samplingNumber); %to record the input without pulse modulation function during testing perioed.
disturbance_impulse=zeros(1,data_samplingNumber); % active disturbance influencing the conditions at certain time point

angu_v_response=ones(1,data_samplingNumber)*angu_v_initial; %the initial angular velocity plot
angle_response=zeros(1,data_samplingNumber);  %the initial angle plot (for thrust)
for i=1:data_samplingNumber
    angle_response(1,i)=angu_v_initial*(i-1)*drawing_period+angle_initial;
end
angu_v_response_thrust=ones(1,data_samplingNumber)*angu_v_initial; 
angle_response_thrust=zeros(1,data_samplingNumber); 
for i=1:data_samplingNumber
    angle_response_thrust(1,i)=angu_v_initial*(i-1)*drawing_period+angle_initial;
end

%%%%%%%%%%%%%%% active disturbance %%%%%%%%%%%%%%%%%
% disturbance_impulse(1,5/drawing_period+1)=20;
% disturbance_impulse(1,10/drawing_period+1)=-50;
% 
% angle_response=lsim(Sys/s,disturbance_impulse,testing_period,'zoh')'+angle_response;
% angu_v_response=lsim(Sys,disturbance_impulse,testing_period,'zoh')'+angu_v_response;
% angle_response_thrust=lsim(Sys/s,disturbance_impulse,testing_period,'zoh')'+angle_response_thrust;
% angu_v_response_thrust=lsim(Sys,disturbance_impulse,testing_period,'zoh')'+angu_v_response_thrust;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Constrain the angle representation between -180 and 180 
angle_response(angle_response>pi) = rem(angle_response(angle_response>pi),2*pi)-2*pi;
angle_response(angle_response<-pi) = rem(angle_response(angle_response<-pi),2*pi)+2*pi;
angle_response_thrust(angle_response_thrust>pi) = rem(angle_response_thrust(angle_response_thrust>pi),2*pi)-2*pi;
angle_response_thrust(angle_response_thrust<-pi) = rem(angle_response_thrust(angle_response_thrust<-pi),2*pi)+2*pi;
% data for sensor to read
angle_response_adj = angle_response;
angle_response_thrust_adj = angle_response_thrust;
if Desired_angle_input < -Range_adj || Desired_angle_input > Range_adj % adjust -180~0 to 180~360 
    angle_response_adj(angle_response<0) = angle_response(angle_response<0)+2*pi;
    angle_response_thrust_adj(angle_response_thrust<0) = angle_response_thrust(angle_response_thrust<0)+2*pi;
end

%initialized conditions for the pulse modulation algorithm
duration_on=on_duration_min;
duration_cycle=duration_on+off_duration_min;
time_last_on=-duration_cycle;
time_last_off=-duration_cycle+duration_on;

for sensor_point=1:sensor_samplingNumber
    
    %%%%%%%%%%%%%% sensor data %%%%%%%%%%%%%%%
    angle_sensorGet=angle_response_adj(1,(sensor_point-1)*scale_sensor2data+1)-Desired_angle;
    angle_sensorGet_thrust=angle_response_thrust_adj(1,(sensor_point-1)*scale_sensor2data+1)-Desired_angle+Dev(1,1)*randn(1);     
    angu_v_sensorGet=angu_v_response(1,(sensor_point-1)*scale_sensor2data+1);
    angu_v_sensorGet_thrust=angu_v_response_thrust(1,(sensor_point-1)*scale_sensor2data+1)+Dev(1,2)*randn(1);
    
    %%%%%%%%%%%%%% disturbance determination %%%%%%%%%%%%%%
    M_passive_dist=sign(-angu_v_sensorGet)*M_passive_dist_value;
    M_passive_dist_thrust=sign(-angu_v_response_thrust(1,(sensor_point-1)*scale_sensor2data+1))*M_passive_dist_value; % friction is determined by the actual valocity 
    
    %%%%%%%%%%%%%% error determination %%%%%%%%%%%%%%
    Error=Expectation-angle_sensorGet*K_angle-angu_v_sensorGet*K_angu_v;
    Error_thrust=Expectation-angle_sensorGet_thrust*K_angle-angu_v_sensorGet_thrust*K_angu_v;
    
    %%%%%%%%%%%%%%%%% control value determination %%%%%%%%%%%%%%%%%
    Control_value=Error; % constant force input
    
    % thruster force input
    if abs(Error_thrust)>=uplimit  
        Control_value_thrust=sign(Error_thrust)*thrust_M_design;
    else
        if abs(Error_thrust)>=deadband 
            Control_value_thrust=Error_thrust;
        else
            Control_value_thrust=0;
        end
    end
    
    %%%%%%%%%%%%%%%%% pulse modulator %%%%%%%%%%%%%%%%%
    if time_last_on>time_last_off
        if abs(Control_value_thrust)>=Control_value_min
            duration_on=off_duration_min*abs(Control_value_thrust)/(thrust_M_design-abs(Control_value_thrust));
        else
            duration_on=on_duration_min;
        end
        if (sensor_point-1)*sensor_period-time_last_on<duration_on
            thrust_switch=sign(Control_value_thrust);
        else
            thrust_switch=0;
            time_last_off=(sensor_point-1)*sensor_period;
        end
    else
        duration_cycle=thrust_M_design*duration_on/abs(Control_value_thrust);
        if duration_cycle<duration_on+off_duration_min
            duration_cycle=duration_on+off_duration_min;
        end
       if (sensor_point-1)*sensor_period-time_last_off<duration_cycle-duration_on;
           thrust_switch=0;           
       else
           thrust_switch=sign(Control_value_thrust);
           time_last_on=(sensor_point-1)*sensor_period;
       end
    end

    %%%%%%%%%%%%%% input to the plant %%%%%%%%%%%%%%%
    input=zeros(1,data_samplingNumber); 
    input(1,(sensor_point-1)*scale_sensor2data+1:(sensor_point-1)*scale_sensor2data+scale_sensor2data)=Control_value; %set the actual input, which is equal to control value plus friction moment
  
    switching=zeros(1,data_samplingNumber);
    switching(1,(sensor_point-1)*scale_sensor2data+1:(sensor_point-1)*scale_sensor2data+scale_sensor2data)=thrust_switch*thrust_M;
    Actuator=lsim(Sys_thrust,switching,testing_period,'zoh')';
    input_thrust=Actuator+(Actuator~=0)*M_passive_dist_thrust; 

    %%%%%%%%%%%%%% response of the plant %%%%%%%%%%%%%%%
    angle_response=lsim(Sys/s,input,testing_period,'zoh')'+angle_response;
    angu_v_response=lsim(Sys,input,testing_period,'zoh')'+angu_v_response;
    angle_response_thrust=lsim(Sys/s,input_thrust,testing_period,'zoh')'+angle_response_thrust;
    angu_v_response_thrust=lsim(Sys,input_thrust,testing_period,'zoh')'+angu_v_response_thrust;
    
    % Constrain the angle representation between -180 and 180
    angle_response(angle_response>pi) = rem(angle_response(angle_response>pi),2*pi)-2*pi;
    angle_response(angle_response<-pi) = rem(angle_response(angle_response<-pi),2*pi)+2*pi;
    angle_response_thrust(angle_response_thrust>pi) = rem(angle_response_thrust(angle_response_thrust>pi),2*pi)-2*pi;
    angle_response_thrust(angle_response_thrust<-pi) = rem(angle_response_thrust(angle_response_thrust<-pi),2*pi)+2*pi;
    % data for sensor to read
    angle_response_adj = angle_response;
    angle_response_thrust_adj = angle_response_thrust;
    if Desired_angle_input < -Range_adj || Desired_angle_input > Range_adj % adjust -180~0 to 180~360
        angle_response_adj(angle_response<0) = angle_response(angle_response<0)+2*pi;
        angle_response_thrust_adj(angle_response_thrust<0) = angle_response_thrust(angle_response_thrust<0)+2*pi;
    end
    
    %%%%%%%%%%%% (out of control loop) record of the input to the plant %%%%%%%%%%%%%%
    Control_value_record(1,(sensor_point-1)*scale_sensor2data+1:(sensor_point-1)*scale_sensor2data+scale_sensor2data)=Control_value;
    %Control_value_thrust_record(1,(sensor_point-1)*scale_sensor2data+1:(sensor_point-1)*scale_sensor2data+scale_sensor2data)=Control_value_thrust;
    Actuator_record=Actuator_record+Actuator;
end

figure,
subplot(3,1,1)
if Desired_angle_input < -Range_adj || Desired_angle_input > Range_adj % -180 is shifted to 180
    plot(testing_period,angle_response_thrust/pi*180,'b',testing_period,angle_response_thrust_adj/pi*180,'m',testing_period,disturbance_impulse,'g',testing_period,angle_response/pi*180,'r'), grid on, title('angle (degree)'), %,testing_period,disturbance_impulse,'g',testing_period,angle_response/pi*180,'r'
else
    plot(testing_period,angle_response_thrust/pi*180,'b',testing_period,disturbance_impulse,'g',testing_period,angle_response/pi*180,'r'), grid on, title('angle (degree)'),
end
% dim = [0.4 0.8 0.1 0.1];
% C=num2str(K_angle);
% note='K-angle =';
% str=strcat(note,C);
% annotation('textbox',dim,'String',str,'FitBoxToText','on');
% dim = [0.5 0.8 0.1 0.1];
% C=num2str(K_angu_v);
% note='K-angular velocity =';
% str=strcat(note,C);
% annotation('textbox',dim,'String',str,'FitBoxToText','on');
% dim = [0.4 0.75 0.1 0.1];
% C=num2str(angle_initial/pi*180);
% note='Initial angle error (degree) =';
% str=strcat(note,C);
% annotation('textbox',dim,'String',str,'FitBoxToText','on');
% dim = [0.4 0.7 0.1 0.1];
% C=num2str(angu_v_initial/pi*180);
% note='Initial angular velocity (degree) =';
% str=strcat(note,C);
% annotation('textbox',dim,'String',str,'FitBoxToText','on');

subplot(3,1,2)
plot(testing_period,angu_v_response_thrust/pi*180,'b',testing_period,angu_v_response/pi*180,'r'), grid on, title('avgular velocity (degree/s)'); %testing_period,angu_v_response/pi*180,'r',
subplot(3,1,3)
plot(testing_period,Actuator_record,'b'), grid on, title('control value (torque, N-m)'); %testing_period,Control_value_thrust_record,'m',testing_period,Control_value_record,'r',
