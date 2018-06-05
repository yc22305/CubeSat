%% data load
data = xlsread('data_collection');

%% data extract
yaw = data(2:6681,1:1);
pitch = data(2:6681,2:2);
roll = data(2:6681,3:3);
ax = data(2:6681,4:4);
ay = data(2:6681,5:5);
az = data(2:6681,6:6);
gx = data(2:6681,7:7);
gy = data(2:6681,8:8);
gz = data(2:6681,9:9);
mx = data(2:6681,10:10);
my = data(2:6681,11:11);
mz = data(2:6681,12:12);

% draw the raw data
% figure, 
% plot(yaw, 'r'), title('yaw (degree/s)');
% figure,
% plot(pitch, 'g');
% figure,
% plot(roll, 'b');
% figure, 
% plot(ax, 'r');
% figure,
% plot(ay, 'g');
% figure,
% plot(az, 'b');
% figure, 
% plot(gx, 'r');
% figure,
% plot(gy, 'g');
% figure,
% plot(gz, 'b'), title('angular velocity (degree/s)');
% figure, 
% plot(mx, 'r');
% figure,
% plot(my, 'g');
% figure,
% plot(mz, 'b');

% data after processing (remove extreme data, shift to mean of 0, etc.)
yaw(859) = 1.6;
figure,
plot(yaw-mean(yaw), 'r'), title('yaw (degree/s)');
figure,
plot(gz-mean(gz), 'b'), title('angular velocity (degree/s)');

% calculate the covariance (variance)
Cov_yaw = cov(yaw);
Cov_pitch = cov(pitch);
Cov_roll = cov(roll);
Cov_ax = cov(ax);
Cov_ay = cov(ay);
Cov_az = cov(az);
Cov_gx = cov(gx);
Cov_gy = cov(gy);
Cov_gz = cov(gz);
Cov_mx = cov(mx);
Cov_my = cov(my);
Cov_mz = cov(mz);

Cov_array = [Cov_yaw Cov_pitch Cov_roll Cov_ax Cov_ay Cov_az Cov_gx Cov_gy Cov_gz Cov_mx Cov_my Cov_mz];

fileID = fopen('variance_MPU9250.txt','w');
fprintf(fileID,'%12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s\n', 'yaw', 'pitch', 'roll', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz');
fprintf(fileID,'%12f %12f %12f %12f %12f %12f %12f %12f %12f %12f %12f %12f',Cov_array);
fclose(fileID);