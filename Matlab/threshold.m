%%%%%% problems waiting for solutions (incomplete code) %%%%%%%
% - check availability of thrusters
%% symbal
s=tf('s');

%% Initialization
%%%%%%%%%%%%%% time %%%%%%%%%%%%%%
sensor_period=0.05; % must be integer multiples of drawing period and larger than thruster restriction
drawing_period=0.01;
end_time=50;

testing_period=0:drawing_period:end_time;
data_samplingNumber=size(testing_period,2);
sensor_samplingNumber=end_time/sensor_period;
scale_sensor2data=sensor_period/drawing_period;

%%%%%%%%%%%%%%% plant %%%%%%%%%%%%%%%
% assuming that center of mass is at geometric center %
m=16; %mass kg
w=0.239; %width meter
l=0.464; %length meter
I=1/12*m*(w^2+l^2); %moment of inertia (kg*meter^2) %%%%%%%%program test data=0.006
Sys=tf(1,[I 0]); %system transfer function
angu_v_initial=0/180*pi; %rad/sec
angle_initial=30/180*pi; %rad. positive for counterclockwise, while negative for clockwise
if angle_initial>pi
    angle_initial=rem(angle_initial,2*pi)-2*pi;
else
    if angle_initial<-pi
        angle_initial=rem(angle_initial,2*pi)+2*pi;
    end
end

%%%%%%%%%%%%% thruster %%%%%%%%%%%%%%
Sys_thrust_M=tf(1,1); %tf(4800,[1 140 4800]); %thruster perturbation
firing_period=sensor_period*1;
Impulse_bit=0.5*firing_period; %%%%%%%program test data=40*(10^-6)
thrust_M=Impulse_bit/firing_period*(l/2)*(3/4); % moment produced by thruster %%%%%%%program test data(arm)=0.14

augu_v_changeEachFiring=Impulse_bit*(l/2)*(3/4)/I;
scale_firing2data=firing_period/drawing_period;

%%%%%%%%%%%%%% disturbance value %%%%%%%%%%%%%%
M_passive_dist_value=0.01764;

%%%%%%%%%%%%%% threshold %%%%%%%%%%%%%%
angle_threshold=0.5/180*pi;
angu_v_threshold=1.2*augu_v_changeEachFiring; % threshold during converging.
angu_v_TracingRange=0.8*augu_v_changeEachFiring; % threshold for fitting optimal angular velocity

%%%%%%%%%%%% user input %%%%%%%%%%%%
Desired_angle=10/180*pi;

%% control loop
Actuator_record=zeros(1,data_samplingNumber); %to record the input without pulse modulation function during testing perioed.
angu_v_allowed_record=zeros(1,data_samplingNumber);
disturbance_impulse=zeros(1,data_samplingNumber);

angu_v_response_thrust=ones(1,data_samplingNumber)*angu_v_initial; %the initial angular velocity plot (for thrust)
angle_response_thrust=zeros(1,data_samplingNumber); %the initial angle plot (for thrust)
for i=1:data_samplingNumber
    angle_response_thrust(1,i)=angu_v_initial*(i-1)*drawing_period+angle_initial;
end

%%%%%%%%%%%%%%% active disturbance %%%%%%%%%%%%%%%%%
%%%%% each impulse lasts drawing_period seconds
% disturbance_impulse(1,5/drawing_period+1)=10;
% disturbance_impulse(1,10/drawing_period+1)=-8;
% 
% angle_response_thrust=lsim(Sys/s,disturbance_impulse,testing_period,'zoh')'+angle_response_thrust;
% angu_v_response_thrust=lsim(Sys,disturbance_impulse,testing_period,'zoh')'+angu_v_response_thrust;

for sensor_point=1:sensor_samplingNumber
    %%%%%%%%%%%%%% sensor data %%%%%%%%%%%%%%%
%     %%%%%%noise
%     angle_sensorGet_thrust=awgn(angle_response_thrust(1,(sensor_point-1)*scale_sensor2data+1),6,'measured');
%     angu_v_sensorGet_thrust=awgn(angu_v_response_thrust(1,(sensor_point-1)*scale_sensor2data+1),6,'measured');
%     %%%%%%
    angle_sensorGet_thrust=angle_response_thrust(1,(sensor_point-1)*scale_sensor2data+1)-Desired_angle;
    angu_v_sensorGet_thrust=angu_v_response_thrust(1,(sensor_point-1)*scale_sensor2data+1);
    
    %%%%%%%%%%%%%% error determination %%%%%%%%%%%%%%
    if abs(angle_sensorGet_thrust)<=angle_threshold
        if abs(angu_v_sensorGet_thrust)<angu_v_threshold
            thrust_switch=0;
        else
            thrust_switch=sign(-angu_v_sensorGet_thrust);
        end
    else
%         if sign(angle_sensorGet_thrust)==sign(angu_v_sensorGet_thrust)
%             thrust_switch=sign(-angu_v_sensorGet_thrust);
%         else
            angu_v_allowed=((2*abs(angle_sensorGet_thrust)/(augu_v_changeEachFiring/sensor_period))^0.5)*(augu_v_changeEachFiring/sensor_period);
            if abs(abs(angu_v_sensorGet_thrust)-angu_v_allowed)<angu_v_TracingRange
                thrust_switch=0;
            else
                if abs(angu_v_sensorGet_thrust)<angu_v_allowed
                    thrust_switch=sign(-angle_sensorGet_thrust);
                else
                    thrust_switch=sign(-angu_v_sensorGet_thrust);
                end
            end
%         end
    end
    %%%%%%%%%%%%%% disturbance %%%%%%%%%%%%%%
    M_passive_dist_thrust=sign(-angu_v_sensorGet_thrust)*M_passive_dist_value; 
    
    %%%%%%%%%%%%%% input to the plant %%%%%%%%%%%%%%%
    switching=zeros(1,data_samplingNumber);
    switching(1,(sensor_point-1)*scale_sensor2data+1:(sensor_point-1)*scale_sensor2data+scale_firing2data)=thrust_switch*thrust_M;
    Actuator=lsim(Sys_thrust_M,switching,testing_period,'zoh')';
    input=Actuator+(Actuator~=0)*M_passive_dist_thrust;  

    %%%%%%%%%%%%%% response of the plant %%%%%%%%%%%%%%%
    angle_response_thrust=lsim(Sys/s,input,testing_period,'zoh')'+angle_response_thrust;
    angu_v_response_thrust=lsim(Sys,input,testing_period,'zoh')'+angu_v_response_thrust;
    
    %%%%%%%%%%%% (out of control loop) record of the input to the plant %%%%%%%%%%%%%%
    Actuator_record=Actuator_record+Actuator;
end
for i=1:data_samplingNumber
    if abs(angle_response_thrust(1,i))<=angle_threshold
        angu_v_allowed_record(1,i)=sign(angu_v_response_thrust(1,i))*angu_v_threshold;
    else
        angu_v_allowed_record(1,i)=sign(angu_v_response_thrust(1,i))*((2*abs(angle_response_thrust(1,i))/(augu_v_changeEachFiring/sensor_period))^0.5)*(augu_v_changeEachFiring/sensor_period);
    end
end

figure,
subplot(3,1,1),
plot(testing_period,angle_response_thrust/pi*180,'b',testing_period,disturbance_impulse,'g'), grid on, title('angle (degree)'), %,testing_period,disturbance_impulse,'g'
% dim = [0.4 0.8 0.1 0.1];
% C=num2str(angle_initial/pi*180);
% note='Initial angle error (degree) =';
% str=strcat(note,C);
% annotation('textbox',dim,'String',str,'FitBoxToText','on');
% dim = [0.4 0.75 0.1 0.1];
% C=num2str(angu_v_initial/pi*180);
% note='Initial angular velocity (degree) =';
% str=strcat(note,C);
% annotation('textbox',dim,'String',str,'FitBoxToText','on');

subplot(3,1,2),
plot(testing_period,angu_v_response_thrust/pi*180,'b'), grid on, title('avgular velocity (degree/s)'); %,testing_period,angu_v_allowed_record/pi*180,'r'
subplot(3,1,3),
plot(testing_period,Actuator_record,'b'), grid on, title('control value (torque, N-m)');
 
% %%%%%%%%%%%%%%%%% angle v.s. angular velocity allowed %%%%%%%%%%%%%%%%
% angle_samping=0:0.1:20;
% N=size(angle_samping,2);
% angu_v_allowed_trend=zeros(1,N);
% for i=1:N
%     if angle_samping(1,i)<=angle_threshold
%         angu_v_allowed_trend(1,i)=angu_v_threshold;
%     else
%         angu_v_allowed_trend(1,i)=coefficient_angu_v_threshold*((2*angle_samping(1,i)/180*pi/(augu_v_changeEachFiring/sensor_period))^0.5)*(augu_v_changeEachFiring/sensor_period);
%     end
% end
% figure,
% plot(angle_samping, angu_v_allowed_trend/pi*180), grid on, title('angle'), xlabel('angle'), ylabel('angular velocity allowed');
