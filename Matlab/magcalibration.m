%%  DATA from excel sheet
name = 'data_1';
Data_mx_1 = xlsread('magcalibration_data.xlsx',name,'A3:A182');
Data_my_1 = xlsread('magcalibration_data.xlsx',name,'B3:B182');
Data_mz_1 = xlsread('magcalibration_data.xlsx',name,'C3:C182');
Data_mx_2 = xlsread('magcalibration_data.xlsx',name,'D3:D182');
Data_my_2 = xlsread('magcalibration_data.xlsx',name,'E3:E182');
Data_mz_2 = xlsread('magcalibration_data.xlsx',name,'F3:F182');
Data_mx_3 = xlsread('magcalibration_data.xlsx',name,'G3:G182');
Data_my_3 = xlsread('magcalibration_data.xlsx',name,'H3:H182');
Data_mz_3 = xlsread('magcalibration_data.xlsx',name,'I3:I182');
%% Raw data 
figure(1),
scatter(Data_mx_1,Data_my_1,40,'filled','MarkerEdgeColor','b','MarkerFaceColor','b');
hold on
scatter(Data_mx_1,Data_mz_1,'filled','MarkerEdgeColor','r','MarkerFaceColor','r');
hold on
scatter(Data_my_1,Data_mz_1,'filled','MarkerEdgeColor','g','MarkerFaceColor','g');
daspect([1 1 1]);

figure(2), 
scatter3(Data_mx_1,Data_my_1,Data_mz_1,'filled','MarkerEdgeColor','b','MarkerFaceColor','b');
daspect([1 1 1]);
%% Hard Iron Calibration 
figure(3)
scatter(Data_mx_2,Data_my_2,'filled','MarkerEdgeColor','b','MarkerFaceColor','b');
hold on
scatter(Data_mx_2,Data_mz_2,'filled','MarkerEdgeColor','r','MarkerFaceColor','r');
hold on
scatter(Data_my_2,Data_mz_2,'filled','MarkerEdgeColor','g','MarkerFaceColor','g');
daspect([1 1 1]);

figure(4)
scatter3(Data_mx_2,Data_my_2,Data_mz_2,'filled','MarkerEdgeColor','b','MarkerFaceColor','b');
daspect([1 1 1]);
%% Hard Iron Calibration + Soft Iron Calibration 
figure(5)
scatter(Data_mx_3,Data_my_3,'filled','MarkerEdgeColor','b','MarkerFaceColor','b');
hold on
scatter(Data_mx_3,Data_mz_3,'filled','MarkerEdgeColor','r','MarkerFaceColor','r');
hold on
scatter(Data_my_3,Data_mz_3,'filled','MarkerEdgeColor','g','MarkerFaceColor','g');
daspect([1 1 1]);

figure(6)
scatter3(Data_mx_3,Data_my_3,Data_mz_3,'filled','MarkerEdgeColor','b','MarkerFaceColor','b');
daspect([1 1 1]);
%% SAVE PLOT
data_num = name;
%file_folder = 'test/';

file_class = '_raw_data_2D';
file_name = strjoin({data_num,file_class});
saveas(figure(1),file_name,'fig'); saveas(figure(1),file_name,'jpg');
file_class = '_raw_data_3D';
file_name = strjoin({data_num,file_class});
saveas(figure(2),file_name,'fig'); saveas(figure(2),file_name,'jpg');

file_class = '_hard_iron_data_2D';
file_name = strjoin({data_num,file_class});
saveas(figure(3),file_name,'fig'); saveas(figure(3),file_name,'jpg');
file_class = '_hard_iron_3D';
file_name = strjoin({data_num,file_class});
saveas(figure(4),file_name,'fig'); saveas(figure(4),file_name,'jpg');

file_class = '_soft_iron_2D';
file_name = strjoin({data_num,file_class});
saveas(figure(5),file_name,'fig'); saveas(figure(5),file_name,'jpg');
file_class = '_soft_iron_3D';
file_name = strjoin({data_num,file_class});
saveas(figure(6),file_name,'fig'); saveas(figure(6),file_name,'jpg');
