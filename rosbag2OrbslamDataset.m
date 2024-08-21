%
clc;
clear;
bag = rosbag('G:\20240722_WP_CAM\2024-07-22-11-01-38.bag');
%%
bSel = select(bag,'Topic','/imu0');
msgStructs = readMessages(bSel,'DataFormat','struct');

%%
AngularVelocity = zeros(size(msgStructs,1),3);
Acc = zeros(size(msgStructs,1),3);
Orientation = zeros(size(msgStructs,1),4);
Acc = zeros(size(msgStructs,1),3);
Orientation = zeros(size(msgStructs,1),4);

IMUSec = zeros(size(msgStructs,1),1);
IMUNSec = zeros(size(msgStructs,1),1);
%%
for ii = 1 : size(msgStructs,1)
    AngularVelocity(ii,:) = [msgStructs{ii}.AngularVelocity.X,...
        msgStructs{ii}.AngularVelocity.Y,msgStructs{ii}.AngularVelocity.Z];

    Acc(ii,:) = [msgStructs{ii}.LinearAcceleration.X,...
        msgStructs{ii}.LinearAcceleration.Y,msgStructs{ii}.LinearAcceleration.Z];

    Orientation(ii,:) = [msgStructs{ii}.Orientation.X,...
        msgStructs{ii}.Orientation.Y,msgStructs{ii}.Orientation.Z,msgStructs{ii}.Orientation.W];
    IMUSec(ii) = msgStructs{ii,1}.Header.Stamp.Sec;
    IMUNSec(ii) = msgStructs{ii,1}.Header.Stamp.Nsec;
end
%%
bSelImage = select(bag,'Topic','/cam0/image_raw');
msgStructsImage = readMessages(bSelImage,'DataFormat','struct');
%%
ImgSec = zeros(size(msgStructsImage,1),1);
ImgNSec = zeros(size(msgStructsImage,1),1);

% 定义图像输出目录
outputDir = 'E:/Projects/OpenVIO/rosbag2orblsamDataset/datasets/dataset1/mav0/cam0/data/'; % 修改为自己想要保存帧的目录
if ~exist(outputDir,'dir')
    mkdir(outputDir);
end

% 打开或新建一个txt文件并指定路径及名称
fileID = fopen('E:/Projects/OpenVIO/rosbag2orblsamDataset/datasets/dataset1/timestamps.txt', 'w');

N = 10000;

for ii = N : size(msgStructsImage,1)
    ImgSec(ii) = msgStructsImage{ii,1}.Header.Stamp.Sec;
    ImgNSec(ii) = msgStructsImage{ii,1}.Header.Stamp.Nsec;
    img = reshape(msgStructsImage{ii,1}.Data,msgStructsImage{ii,1}.Width,msgStructsImage{ii,1}.Height);
    % 构造输出文件名
    outputFile = fullfile(outputDir, sprintf('%s.png',[num2str(ImgSec(ii) * 10^9 + ImgNSec(ii))]));
    imwrite(img', outputFile);% 保存帧到指定位置

    fprintf(fileID,[num2str(ImgSec(ii) * 10^9 + ImgNSec(ii)) '\n']); % 换行符号
    
    
    
    fprintf('已处理第 %d/%d 帧\n', ii,size(msgStructsImage,1));
end
fclose(fileID);
%%
% 定义IMU输出目录
outputDir = 'E:/Projects/OpenVIO/rosbag2orblsamDataset/datasets/dataset1/mav0/imu0'; % 修改为自己想要保存帧的目录
if ~exist(outputDir,'dir')
    mkdir(outputDir);
end

filename = [outputDir '\data.csv'];
title={'#timestamp [ns]','w_RS_S_x [rad s^-1]','w_RS_S_y [rad s^-1]','w_RS_S_z [rad s^-1]','a_RS_S_x [m s^-2]','a_RS_S_y [m s^-2]','a_RS_S_z [m s^-2]'};

IMUTimeStamp = IMUSec*10^9 + IMUNSec;

result_table=table(IMUTimeStamp(N:end,:), ...
    AngularVelocity(N:end,1),AngularVelocity(N:end,2),AngularVelocity(N:end,3), ...
    Acc(N:end,1),Acc(N:end,2),Acc(N:end,3),'VariableNames',title);
% 保存IMU数据
writetable(result_table, filename);