close all;
clear;
clc;

% File Name Assignment
% ---------------------------------------------------------%
Flexion_filename = {};
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion1.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion2.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion3.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion4.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion5.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion6.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion7.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion8.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion9.txt';
Flexion_filename{end + 1} = '../StudioData/0709/0709_Flexion10.txt';

Extension_filename = {};
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension1.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension2.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension3.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension4.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension5.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension6.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension7.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension8.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension9.txt';
Extension_filename{end + 1} = '../StudioData/0709/0709_Extension10.txt';

dataCount_Flexion = size(Flexion_filename, 2);
dataCount_Extension = size(Extension_filename, 2);

% Read Table
% ---------------------------------------------------------%
Table_Flexion = cell(1, dataCount_Flexion);
Table_Extension = cell(1, dataCount_Extension);

for i = 1:dataCount_Flexion
    Table_Flexion{i} = readtable(Flexion_filename{i}, 'VariableNamingRule', 'preserve');
end

for i = 1:size(Extension_filename, 2)
    Table_Extension{i} = readtable(Extension_filename{i}, 'VariableNamingRule', 'preserve');
end

% Get Variable in Table
% ---------------------------------------------------------%
time_Flexion = cell(1, dataCount_Flexion);
curPosition_Flexion = cell(1, dataCount_Flexion);
goalVelocity_Flexion = cell(1, dataCount_Flexion);
tensionRecordFlag_PIP_Flexion = cell(1, dataCount_Flexion);
tensionRecordFlag_DIP_Flexion = cell(1, dataCount_Flexion);
angleRecordFlag_PIP_Flexion = cell(1, dataCount_Flexion);
angleRecordFlag_DIP_Flexion = cell(1, dataCount_Flexion);
comp_AngAS5048A_PIP_Flexion = cell(1, dataCount_Flexion);
comp_AngAS5048A_DIP_Flexion = cell(1, dataCount_Flexion);
curF_Velocity_PIP_Flexion = cell(1, dataCount_Flexion);
curF_Velocity_DIP_Flexion = cell(1, dataCount_Flexion);
filtered_cur_loadcell_Flexion = cell(1, dataCount_Flexion);
tareRecordFlag_Flexion = cell(1, dataCount_Flexion);

for i = 1:dataCount_Flexion
    time_Flexion{i} = Table_Flexion{i}.Var2;
    curPosition_Flexion{i} = Table_Flexion{i}.curPosition;
    goalVelocity_Flexion{i} = Table_Flexion{i}.goalVelocity;
    tensionRecordFlag_PIP_Flexion{i} = Table_Flexion{i}.tensionRecordFlag_PIP;
    tensionRecordFlag_DIP_Flexion{i} = Table_Flexion{i}.tensionRecordFlag_DIP;
    angleRecordFlag_PIP_Flexion{i} = Table_Flexion{i}.angleRecordFlag_PIP;
    angleRecordFlag_DIP_Flexion{i} = Table_Flexion{i}.angleRecordFlag_DIP;
    comp_AngAS5048A_PIP_Flexion{i} = Table_Flexion{i}.comp_AngAS5048A_PIP;
    comp_AngAS5048A_DIP_Flexion{i} = Table_Flexion{i}.comp_AngAS5048A_DIP;
    curF_Velocity_PIP_Flexion{i} = Table_Flexion{i}.curF_Velocity_PIP;
    curF_Velocity_DIP_Flexion{i} = Table_Flexion{i}.curF_Velocity_DIP;
    filtered_cur_loadcell_Flexion{i} = Table_Flexion{i}.filtered_cur_loadcell;
    tareRecordFlag_Flexion{i} = Table_Flexion{i}.tareRecordFlag;

end

time_Extension = cell(1, dataCount_Extension);
curPosition_Extension = cell(1, dataCount_Extension);
goalVelocity_Extension = cell(1, dataCount_Extension);
tensionRecordFlag_PIP_Extension = cell(1, dataCount_Extension);
tensionRecordFlag_DIP_Extension = cell(1, dataCount_Extension);
angleRecordFlag_PIP_Extension = cell(1, dataCount_Extension);
angleRecordFlag_DIP_Extension = cell(1, dataCount_Extension);
comp_AngAS5048A_PIP_Extension = cell(1, dataCount_Extension);
comp_AngAS5048A_DIP_Extension = cell(1, dataCount_Extension);
curF_Velocity_PIP_Extension = cell(1, dataCount_Extension);
curF_Velocity_DIP_Extension = cell(1, dataCount_Extension);
filtered_cur_loadcell_Extension = cell(1, dataCount_Extension);
tareRecordFlag_Extension = cell(1, dataCount_Extension);

for i = 1:dataCount_Extension
    time_Extension{i} = Table_Extension{i}.Var2;
    curPosition_Extension{i} = Table_Extension{i}.curPosition;
    goalVelocity_Extension{i} = Table_Extension{i}.goalVelocity;
    tensionRecordFlag_PIP_Extension{i} = Table_Extension{i}.tensionRecordFlag_PIP;
    tensionRecordFlag_DIP_Extension{i} = Table_Extension{i}.tensionRecordFlag_DIP;
    angleRecordFlag_PIP_Extension{i} = Table_Extension{i}.angleRecordFlag_PIP;
    angleRecordFlag_DIP_Extension{i} = Table_Extension{i}.angleRecordFlag_DIP;
    comp_AngAS5048A_PIP_Extension{i} = Table_Extension{i}.comp_AngAS5048A_PIP;
    comp_AngAS5048A_DIP_Extension{i} = Table_Extension{i}.comp_AngAS5048A_DIP;
    curF_Velocity_PIP_Extension{i} = Table_Extension{i}.curF_Velocity_PIP;
    curF_Velocity_DIP_Extension{i} = Table_Extension{i}.curF_Velocity_DIP;
    filtered_cur_loadcell_Extension{i} = Table_Extension{i}.filtered_cur_loadcell;
    tareRecordFlag_Extension{i} = Table_Extension{i}.tareRecordFlag;

end

% Get the segment we need
% ---------------------------------------------------------%
avgTare_Flexion = cell(1, dataCount_Flexion);
instantTension_PIP_Flexion = cell(1, dataCount_Flexion);
instantTension_DIP_Flexion = cell(1, dataCount_Flexion);
avgAngle_PIP_Flexion = cell(1, dataCount_Flexion);
avgAngle_DIP_Flexion = cell(1, dataCount_Flexion);

for i = 1:dataCount_Flexion

    d_tareFlag = diff(tareRecordFlag_Flexion{i});
    d_tenFlag_PIP = diff(tensionRecordFlag_PIP_Flexion{i});
    d_tenFlag_DIP = diff(tensionRecordFlag_DIP_Flexion{i});
    d_angFlag_PIP = diff(angleRecordFlag_PIP_Flexion{i});
    d_angFlag_DIP = diff(angleRecordFlag_DIP_Flexion{i});

    startIdx_tareFlag = find(d_tareFlag == 1);
    startIdx_tenFlag_PIP = find(d_tenFlag_PIP == 1);
    startIdx_tenFlag_DIP = find(d_tenFlag_DIP == 1);
    startIdx_angFlag_PIP = find(d_angFlag_PIP == 1);
    startIdx_angFlag_DIP = find(d_angFlag_DIP == 1);

    endIdx_tareFlag = find(d_tareFlag == -1) - 1;
    endIdx_angFlag_PIP = find(d_angFlag_PIP == -1) - 1;
    endIdx_angFlag_DIP = find(d_angFlag_DIP == -1) - 1;

    for j = 1:size(startIdx_tareFlag)
        segmentTare = filtered_cur_loadcell_Flexion{i}(startIdx_tareFlag(j):endIdx_tareFlag(j));
        avgTare_Flexion{i}(j) = mean(segmentTare);
    end

    for j = 1:size(startIdx_tenFlag_PIP)
        instantTension_PIP_Flexion{i}(j) = filtered_cur_loadcell_Flexion{i}(startIdx_tenFlag_PIP(j));
    end

    for j = 1:size(startIdx_tenFlag_DIP)
        instantTension_DIP_Flexion{i}(j) = filtered_cur_loadcell_Flexion{i}(startIdx_tenFlag_DIP(j));
    end

    for j = 1:size(startIdx_angFlag_PIP)
        segmentAng = comp_AngAS5048A_PIP_Flexion{i}(startIdx_angFlag_PIP(j):endIdx_angFlag_PIP(j));
        avgAngle_PIP_Flexion{i}(j) = mean(segmentAng);
    end

    for j = 1:size(startIdx_angFlag_DIP)
        segmentAng = comp_AngAS5048A_DIP_Flexion{i}(startIdx_angFlag_DIP(j):endIdx_angFlag_DIP(j));
        avgAngle_DIP_Flexion{i}(j) = mean(segmentAng);
    end

end

avgTare_Extension = cell(1, dataCount_Extension);
instantTension_PIP_Extension = cell(1, dataCount_Extension);
instantTension_DIP_Extension = cell(1, dataCount_Extension);
avgAngle_PIP_Extension = cell(1, dataCount_Extension);
avgAngle_DIP_Extension = cell(1, dataCount_Extension);

for i = 1:dataCount_Extension

    d_tareFlag = diff(tareRecordFlag_Extension{i});
    d_tenFlag_PIP = diff(tensionRecordFlag_PIP_Extension{i});
    d_tenFlag_DIP = diff(tensionRecordFlag_DIP_Extension{i});
    d_angFlag_PIP = diff(angleRecordFlag_PIP_Extension{i});
    d_angFlag_DIP = diff(angleRecordFlag_DIP_Extension{i});

    startIdx_tareFlag = find(d_tareFlag == 1);
    startIdx_tenFlag_PIP = find(d_tenFlag_PIP == 1);
    startIdx_tenFlag_DIP = find(d_tenFlag_DIP == 1);
    startIdx_angFlag_PIP = find(d_angFlag_PIP == 1);
    startIdx_angFlag_DIP = find(d_angFlag_DIP == 1);

    endIdx_tareFlag = find(d_tareFlag == -1) - 1;
    endIdx_angFlag_PIP = find(d_angFlag_PIP == -1) - 1;
    endIdx_angFlag_DIP = find(d_angFlag_DIP == -1) - 1;

    for j = 1:size(startIdx_tareFlag)
        segmentTare = filtered_cur_loadcell_Extension{i}(startIdx_tareFlag(j):endIdx_tareFlag(j));
        avgTare_Extension{i}(j) = mean(segmentTare);
    end

    for j = 1:size(startIdx_tenFlag_PIP)
        instantTension_PIP_Extension{i}(j) = filtered_cur_loadcell_Extension{i}(startIdx_tenFlag_PIP(j));
    end

    for j = 1:size(startIdx_tenFlag_DIP)
        instantTension_DIP_Extension{i}(j) = filtered_cur_loadcell_Extension{i}(startIdx_tenFlag_DIP(j));
    end

    for j = 1:size(startIdx_angFlag_PIP)
        segmentAng = comp_AngAS5048A_PIP_Extension{i}(startIdx_angFlag_PIP(j):endIdx_angFlag_PIP(j));
        avgAngle_PIP_Extension{i}(j) = mean(segmentAng);
    end

    for j = 1:size(startIdx_angFlag_DIP)
        segmentAng = comp_AngAS5048A_DIP_Extension{i}(startIdx_angFlag_DIP(j):endIdx_angFlag_DIP(j));
        avgAngle_DIP_Extension{i}(j) = mean(segmentAng);
    end

end

% Tare Procressing
% ---------------------------------------------------------%
for i = 1:10
    instantTension_PIP_Flexion{i} = instantTension_PIP_Flexion{i} - avgTare_Flexion{i};
    instantTension_DIP_Flexion{i} = instantTension_DIP_Flexion{i} - avgTare_Flexion{i};
    instantTension_PIP_Extension{i} = instantTension_PIP_Extension{i} - avgTare_Extension{i};
    instantTension_DIP_Extension{i} = instantTension_DIP_Extension{i} - avgTare_Extension{i};
end

% ---------------------------------------------------------%
% ---------------------------------------------------------%
figure(1);
hold on;

yyaxis left;
plot(time_Flexion{1}, curPosition_Flexion{1}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, filtered_cur_loadcell_Flexion{1}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, comp_AngAS5048A_PIP_Flexion{1}, 'Color', '#77AC30', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, comp_AngAS5048A_DIP_Flexion{1}, 'Color', '#EDB120', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
ylabel('Loadcell\_Value or Angle (deg)');

yyaxis right;
plot(time_Flexion{1}, curF_Velocity_PIP_Flexion{1}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, curF_Velocity_DIP_Flexion{1}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, goalVelocity_Flexion{1}, 'Color', '#4DBEEE', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, tareRecordFlag_Flexion{1}, 'Color', '#D95319', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, tensionRecordFlag_PIP_Flexion{1}, 'Color', '#0072BD', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, tensionRecordFlag_DIP_Flexion{1}, 'Color', '#FF00FF', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, angleRecordFlag_PIP_Flexion{1}, 'Color', 'k', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Flexion{1}, angleRecordFlag_DIP_Flexion{1}, 'Color', 'g', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
ylabel('Velocity (rpm)');

leg1 = legend({'curPosition', 'filtered\_cur\_loadcell', 'comp\_AngAS5048A\_PIP', 'comp\_AngAS5048A\_DIP', ...
                   'curF\_Velocity\_PIP', 'curF\_Velocity\_DIP', 'goalVelocity', 'tareRecordFlag', ...
                   'tensionRecordFlag\_PIP', 'tensionRecordFlag\_DIP', 'angRecordFlag\_PIP', 'angRecordFlag\_DIP'}, 'Location', 'best');
leg1.FontSize = 18;
hold off;
xlabel('time (ms)');
title('Tension and Torque Spring Experiment - Flexion');
set(gca, 'FontSize', 25);
grid on;

% ---------------------------------------------------------%
% ---------------------------------------------------------%
figure(2);
hold on;

yyaxis left;
plot(time_Extension{1}, curPosition_Extension{1}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, filtered_cur_loadcell_Extension{1}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, comp_AngAS5048A_PIP_Extension{1}, 'Color', '#77AC30', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, comp_AngAS5048A_DIP_Extension{1}, 'Color', '#EDB120', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
ylabel('Loadcell\_Value or Angle (deg)');

yyaxis right;
plot(time_Extension{1}, curF_Velocity_PIP_Extension{1}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, curF_Velocity_DIP_Extension{1}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, goalVelocity_Extension{1}, 'Color', '#4DBEEE', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, tareRecordFlag_Extension{1}, 'Color', '#D95319', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, tensionRecordFlag_PIP_Extension{1}, 'Color', '#0072BD', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, tensionRecordFlag_DIP_Extension{1}, 'Color', '#FF00FF', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, angleRecordFlag_PIP_Extension{1}, 'Color', 'k', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(time_Extension{1}, angleRecordFlag_DIP_Extension{1}, 'Color', 'g', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
ylabel('Velocity (rpm)');

leg2 = legend({'curPosition', 'filtered\_cur\_loadcell', 'comp\_AngAS5048A\_PIP', 'comp\_AngAS5048A\_DIP', ...
                   'curF\_Velocity\_PIP', 'curF\_Velocity\_DIP', 'goalVelocity', 'tareRecordFlag', ...
                   'tensionRecordFlag\_PIP', 'tensionRecordFlag\_DIP', 'angRecordFlag\_PIP', 'angRecordFlag\_DIP'}, 'Location', 'best');
leg2.FontSize = 18;
hold off;
xlabel('time (ms)');
title('Tension and Torque Spring Experiment - Extension');
set(gca, 'FontSize', 25);
grid on;

% ---------------------------------------------------------%
% ---------------------------------------------------------%
f = figure(3);
f.Position = [0 550 600 400];
hold on;

plot(avgAngle_PIP_Flexion{1}, instantTension_PIP_Flexion{1}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{2}, instantTension_PIP_Flexion{2}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{3}, instantTension_PIP_Flexion{3}, 'Color', '#77AC30', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{4}, instantTension_PIP_Flexion{4}, 'Color', '#EDB120', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{5}, instantTension_PIP_Flexion{5}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{6}, instantTension_PIP_Flexion{6}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{7}, instantTension_PIP_Flexion{7}, 'Color', '#4DBEEE', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{8}, instantTension_PIP_Flexion{8}, 'Color', '#D95319', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{9}, instantTension_PIP_Flexion{9}, 'Color', '#0072BD', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{10}, instantTension_PIP_Flexion{10}, 'Color', '#FF00FF', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
hold off;
xlabel('Angle (deg)');
ylabel('Loadcell Value');
title('PIP Joint Flexion');
set(gca, 'FontSize', 25);
grid on;

% ---------------------------------------------------------%
% ---------------------------------------------------------%
f = figure(4);
f.Position = [0 50 600 400];
hold on;
plot(avgAngle_PIP_Extension{1}, instantTension_PIP_Extension{1}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{2}, instantTension_PIP_Extension{2}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{3}, instantTension_PIP_Extension{3}, 'Color', '#77AC30', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{4}, instantTension_PIP_Extension{4}, 'Color', '#EDB120', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{5}, instantTension_PIP_Extension{5}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{6}, instantTension_PIP_Extension{6}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{7}, instantTension_PIP_Extension{7}, 'Color', '#4DBEEE', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{8}, instantTension_PIP_Extension{8}, 'Color', '#D95319', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{9}, instantTension_PIP_Extension{9}, 'Color', '#0072BD', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{10}, instantTension_PIP_Extension{10}, 'Color', '#FF00FF', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
hold off;
xlabel('Angle (deg)');
ylabel('Loadcell Value');
title('PIP Joint Extension');
set(gca, 'FontSize', 25);
grid on;

% ---------------------------------------------------------%
% ---------------------------------------------------------%
f = figure(5);
f.Position = [1250 550 600 400];
hold on;

plot(avgAngle_DIP_Flexion{1}, instantTension_DIP_Flexion{1}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{2}, instantTension_DIP_Flexion{2}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{3}, instantTension_DIP_Flexion{3}, 'Color', '#77AC30', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{4}, instantTension_DIP_Flexion{4}, 'Color', '#EDB120', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{5}, instantTension_DIP_Flexion{5}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{6}, instantTension_DIP_Flexion{6}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{7}, instantTension_DIP_Flexion{7}, 'Color', '#4DBEEE', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{8}, instantTension_DIP_Flexion{8}, 'Color', '#D95319', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{9}, instantTension_DIP_Flexion{9}, 'Color', '#0072BD', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{10}, instantTension_DIP_Flexion{10}, 'Color', '#FF00FF', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
hold off;
xlabel('Angle (deg)');
ylabel('Loadcell Value');
title('DIP Joint Flexion');
set(gca, 'FontSize', 25);
grid on;

% ---------------------------------------------------------%
% ---------------------------------------------------------%
f = figure(6);
f.Position = [1250 50 600 400];
hold on;
plot(avgAngle_DIP_Extension{1}, instantTension_DIP_Extension{1}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{2}, instantTension_DIP_Extension{2}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{3}, instantTension_DIP_Extension{3}, 'Color', '#77AC30', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{4}, instantTension_DIP_Extension{4}, 'Color', '#EDB120', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{5}, instantTension_DIP_Extension{5}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{6}, instantTension_DIP_Extension{6}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{7}, instantTension_DIP_Extension{7}, 'Color', '#4DBEEE', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{8}, instantTension_DIP_Extension{8}, 'Color', '#D95319', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{9}, instantTension_DIP_Extension{9}, 'Color', '#0072BD', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{10}, instantTension_DIP_Extension{10}, 'Color', '#FF00FF', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
hold off;
xlabel('Angle (deg)');
ylabel('Loadcell Value');
title('DIP Joint Extension');
set(gca, 'FontSize', 25);
grid on;

% ---------------------------------------------------------%
% ---------------------------------------------------------%

f = figure(7);
f.Position = [1250 50 600 400];

tareList = [];

for i = 1:10
    tareList = cat(1, tareList, avgTare_Flexion{i});
end

for i = 1:10
    tareList = cat(1, tareList, avgTare_Extension{i});
end

hold on;
plot(tareList, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
hold off;
xlabel('Number of times');
ylabel('Loadcell Value');
title('Average Tare value');
set(gca, 'FontSize', 25);
grid on;

% ---------------------------------------------------------%
% ---------------------------------------------------------%
% ----------------------  Fitting -------------------------%
% ---------------------------------------------------------%
% ---------------------------------------------------------%

% Get from the Calibration Experiment
% Loadcell Value to Force (N)
loadcell_gain = 100;

r_winder = 0.0095; % m
r_pulley = 0.0045; % m

% Joint Torque = ((Loadcell-tare)*gain/2)* r_pulley

% Loadcell Value to Tension, then to joint torque
instantTorque_PIP_Flexion = cell(1, dataCount_Flexion);
instantTorque_DIP_Flexion = cell(1, dataCount_Flexion);
instantTorque_PIP_Extension = cell(1, dataCount_Extension);
instantTorque_DIP_Extension = cell(1, dataCount_Extension);

for i = 1:dataCount_Flexion
    instantTorque_PIP_Flexion{i} = instantTension_PIP_Flexion{i} * loadcell_gain * r_pulley / 2;
    instantTorque_DIP_Flexion{i} = instantTension_PIP_Flexion{i} * loadcell_gain * r_pulley / 2;
end

for i = 1:dataCount_Extension
    instantTorque_PIP_Extension{i} = instantTension_PIP_Extension{i} * loadcell_gain * r_pulley / 2;
    instantTorque_DIP_Extension{i} = instantTension_DIP_Extension{i} * loadcell_gain * r_pulley / 2;
end

% Concatenate arrays
instantTorque_PIP_Flexion_All = [];
instantTorque_DIP_Flexion_All = [];
avgAngle_PIP_Flexion_All = [];
avgAngle_DIP_Flexion_All = [];

for i = 1:dataCount_Flexion
    instantTorque_PIP_Flexion_All = cat(1, instantTorque_PIP_Flexion_All, instantTorque_PIP_Flexion{i});
    instantTorque_DIP_Flexion_All = cat(1, instantTorque_DIP_Flexion_All, instantTorque_DIP_Flexion{i});
    avgAngle_PIP_Flexion_All = cat(1, avgAngle_PIP_Flexion_All, avgAngle_PIP_Flexion{i});
    avgAngle_DIP_Flexion_All = cat(1, avgAngle_DIP_Flexion_All, avgAngle_DIP_Flexion{i});
end

instantTorque_PIP_Extension_All = [];
instantTorque_DIP_Extension_All = [];
avgAngle_PIP_Extension_All = [];
avgAngle_DIP_Extension_All = [];

for i = 1:dataCount_Extension
    instantTorque_PIP_Extension_All = cat(1, instantTorque_PIP_Extension_All, instantTorque_PIP_Extension{i});
    instantTorque_DIP_Extension_All = cat(1, instantTorque_DIP_Extension_All, instantTorque_DIP_Extension{i});
    avgAngle_PIP_Extension_All = cat(1, avgAngle_PIP_Extension_All, avgAngle_PIP_Extension{i});
    avgAngle_DIP_Extension_All = cat(1, avgAngle_DIP_Extension_All, avgAngle_DIP_Extension{i});
end

coeffsPIP_Flexion = polyfit(avgAngle_PIP_Flexion_All, instantTorque_PIP_Flexion_All, 3);
coeffsDIP_Flexion = polyfit(avgAngle_DIP_Flexion_All, instantTorque_DIP_Flexion_All, 3);
coeffsPIP_Extension = polyfit(avgAngle_PIP_Extension_All, instantTorque_PIP_Extension_All, 3);
coeffsDIP_Extension = polyfit(avgAngle_DIP_Extension_All, instantTorque_DIP_Extension_All, 3);

PIP_x = linspace(0, 100, 100);
DIP_x = linspace(0, 60, 60);
fitPIP_Flexion = polyval(coeffsPIP_Flexion, PIP_x);
fitDIP_Flexion = polyval(coeffsDIP_Flexion, DIP_x);
fitPIP_Extension = polyval(coeffsPIP_Extension, PIP_x);
fitDIP_Extension = polyval(coeffsDIP_Extension, DIP_x);

% ---------------------------------------------------------%
% ---------------------------------------------------------%
figure(8);
hold on;

plot(avgAngle_PIP_Flexion{1}, instantTorque_PIP_Flexion{1}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{2}, instantTorque_PIP_Flexion{2}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{3}, instantTorque_PIP_Flexion{3}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{4}, instantTorque_PIP_Flexion{4}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{5}, instantTorque_PIP_Flexion{5}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{6}, instantTorque_PIP_Flexion{6}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{7}, instantTorque_PIP_Flexion{7}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{8}, instantTorque_PIP_Flexion{8}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{9}, instantTorque_PIP_Flexion{9}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Flexion{10}, instantTorque_PIP_Flexion{10}, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(PIP_x, fitPIP_Flexion, 'Color', '#77AC30', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');

plot(avgAngle_PIP_Extension{1}, instantTorque_PIP_Extension{1}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{2}, instantTorque_PIP_Extension{2}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{3}, instantTorque_PIP_Extension{3}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{4}, instantTorque_PIP_Extension{4}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{5}, instantTorque_PIP_Extension{5}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{6}, instantTorque_PIP_Extension{6}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{7}, instantTorque_PIP_Extension{7}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{8}, instantTorque_PIP_Extension{8}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{9}, instantTorque_PIP_Extension{9}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_PIP_Extension{10}, instantTorque_PIP_Extension{10}, 'Color', 'b', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(PIP_x, fitPIP_Extension, 'Color', '#EDB120', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');

leg8 = legend({'PIP_Flexion', 'PIP_Flexion (Fit)', 'PIP_Extension', 'PIP_Extension (Fit)'}, 'Location', 'best');
leg8.FontSize = 20;
hold off;
xlabel('AS5048A Angle (Degree)');
ylabel('Torque (N-m)');
title('PIP Joint');
set(gca, 'FontSize', 25);
grid on;

% ---------------------------------------------------------%
% ---------------------------------------------------------%
figure(9);
hold on;

plot(avgAngle_DIP_Flexion{1}, instantTorque_DIP_Flexion{1}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{2}, instantTorque_DIP_Flexion{2}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{3}, instantTorque_DIP_Flexion{3}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{4}, instantTorque_DIP_Flexion{4}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{5}, instantTorque_DIP_Flexion{5}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{6}, instantTorque_DIP_Flexion{6}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{7}, instantTorque_DIP_Flexion{7}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{8}, instantTorque_DIP_Flexion{8}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{9}, instantTorque_DIP_Flexion{9}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Flexion{10}, instantTorque_DIP_Flexion{10}, 'Color', '#7E2F8E', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(DIP_x, fitDIP_Flexion, 'Color', '#4DBEEE', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');

plot(avgAngle_DIP_Extension{1}, instantTorque_DIP_Extension{1}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{2}, instantTorque_DIP_Extension{2}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{3}, instantTorque_DIP_Extension{3}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{4}, instantTorque_DIP_Extension{4}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{5}, instantTorque_DIP_Extension{5}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{6}, instantTorque_DIP_Extension{6}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{7}, instantTorque_DIP_Extension{7}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{8}, instantTorque_DIP_Extension{8}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{9}, instantTorque_DIP_Extension{9}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(avgAngle_DIP_Extension{10}, instantTorque_DIP_Extension{10}, 'Color', '#A2142F', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');
plot(DIP_x, fitDIP_Extension, 'Color', '#FF00FF', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '*');

leg9 = legend({'DIP_Flexion', 'DIP_Flexion (Fit)', 'DIP_Extension', 'DIP_Extension (Fit)'}, 'Location', 'best');
leg9.FontSize = 20;
hold off;
xlabel('AS5048A Angle (Degree)');
ylabel('Torque (N-m)');
title('DIP Joint');
set(gca, 'FontSize', 25);
grid on;
