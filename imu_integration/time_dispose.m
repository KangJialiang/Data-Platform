

clear
close all
data = [];
n_name = 10;
Re = 6378137;
i = 0;
head_path = 'dataset\2011_09_26_drive_0028\2011_09_26_drive_0028_extract\2011_09_26\2011_09_26_drive_0028_extract\';
% head_path = 'dataset\2011_09_26_drive_0032\2011_09_26_drive_0032_extract\2011_09_26\2011_09_26_drive_0032_extract\';
path = [head_path, 'oxts\data\'];
name = 48*ones(1,10);
fullpath = [path, char(name), '.txt'];
fid = fopen(fullpath);
while fid > -1
    data_temp = textscan(fid,'%f');
    data = [data,data_temp{1}];
    fclose(fid);
    
    i = i+1;
    temp = i;
    for j=1:n_name
        name(j) = 48 + floor(temp/10^(n_name-j));
        temp = mod(temp,10^(n_name-j));
    end
    fullpath = [path, char(name), '.txt'];
    fid = fopen(fullpath);
    
%     while fid <0
%         i = i+1;
%         temp = i;
%         for j=1:n_name
%             name(j) = 48 + floor(temp/10^(n_name-j));
%             temp = mod(temp,10^(n_name-j));
%         end
%         fullpath = [path, char(name), '.txt'];
%         fid = fopen(fullpath);
%     end
end
figure
acc = data(12:14,:)';
plot(data(12:14,:)')
title('acc')
figure
gyro = data(18:20,:)';
plot(data(18:20,:)')
title('gyro')
figure
plot(data(2,:)', data(1,:)')
title('经纬度')
figure
plot(data(27,:)')
title('卫星个数')
figure
plot(data(28,:)')
title('求解状态')
data = data';



fullpath =[head_path, 'oxts\timestamps.txt'];
fid = fopen(fullpath);
imu_timestamps0 = textscan(fid,'%s %s');
fclose(fid);
imu_timestamps1 = imu_timestamps0{1,2};   %不要年月日
imu_timestamps = zeros(length(imu_timestamps1), 1);
for i = 1:length(imu_timestamps1)
    imu_timestamps2 = imu_timestamps1{i};
     sec = str2num(imu_timestamps2(7:end));
     minite = str2num(imu_timestamps2(4:5));
     hour = str2num(imu_timestamps2(1:2));
     imu_timestamps(i) = hour*3600 + minite*60 + sec;
end
figure
plot(diff(imu_timestamps))
title('imu diff time')
imu_timestamps_l = imu_timestamps - imu_timestamps(1);
id_len = 1:length(imu_timestamps_l);
imu_timestamps_l2 = imu_timestamps_l;
imu_timestamps_diff = diff(imu_timestamps);

id = [];
while(~isempty(imu_timestamps_l2))
    id_min = id_len(imu_timestamps_l == min(imu_timestamps_l2));
    imu_timestamps_l2(abs(imu_timestamps_l2 - min(imu_timestamps_l2)) < 1e-6 ) = [];
    id = [id; id_min(end)];
end
imu_timestamps_x = imu_timestamps(id);
data_x = data(id,:);
figure
plot(diff(imu_timestamps_x))
title('imu diff time final')


fullpath =[head_path, 'image_00\timestamps.txt'];
fid = fopen(fullpath);
image_timestamps0 = textscan(fid,'%s %s');
fclose(fid);
image_timestamps1 = image_timestamps0{1,2};   %不要年月日
image_timestamps = zeros(length(image_timestamps1),1);
for i = 1:length(image_timestamps1)
    image_timestamps2 = image_timestamps1{i};
     sec = str2num(image_timestamps2(7:end));
     minite = str2num(image_timestamps2(4:5));
     hour = str2num(imu_timestamps2(1:2));
     image_timestamps(i) = hour*3600 + minite*60 + sec;
end
figure
plot(diff(image_timestamps))
title('image diff time')
image_timestamps_l = image_timestamps - image_timestamps(1);

figure
plot(image_timestamps,ones(size(image_timestamps)),'.', imu_timestamps_x,ones(size(imu_timestamps_x)),'.')


%% imu积分
pos0 = [0; 0; 0];
vel0 = [data_x(1,8), data_x(1,7), data_x(1,11)]';
att0 = [-data_x(1,5), data_x(1,4), data_x(1,6)-pi/2]';
bias_a = [0; 0; 0.01];
bias_g = [0; 0; 0];
pos_all = zeros(length(imu_timestamps_x), 3);
vel_all = zeros(length(imu_timestamps_x), 3);
att_all = zeros(length(imu_timestamps_x), 3);
pos_all(1,:) = pos0';
vel_all(1,:) = vel0';
att_all(1,:) = att0';
q_rfu = a2qua([0; 0; pi/2]);
[q_rfu, att_rfu, C_rfu] = attsyn(q_rfu);
for i = 2:length(imu_timestamps_x)
    
    delta_t = imu_timestamps_x(i) - imu_timestamps_x(i-1);
    [pos_new,vel_new,att_new] = imu_int2(pos0,vel0,att0, C_rfu*acc(i,:)', C_rfu*gyro(i,:)', bias_a,bias_g,delta_t);
    pos_all(i,:) = pos_new';
    vel_all(i,:) = vel_new';
    att_all(i,:) = att_new';
    pos0 = pos_new;
    vel0 = vel_new;
    att0 = att_new; 
end

pos_data = [data_x(:,2)/180*pi*Re*cos(data_x(1,1)/180*pi), data_x(:,1)/180*pi*Re, data_x(:,3)];
pos_data = pos_data - pos_data(1,:);
nn = length(imu_timestamps_x);
figure
plot3(pos_data(1:nn,1), pos_data(1:nn,2), pos_data(1:nn,3))
xlabel('x')
ylabel('y')
hold on
plot3(pos_all(1:nn,1), pos_all(1:nn,2), pos_all(1:nn,3))
hold off
title('位置')

figure
subplot(3,1,1)
plot(pos_all(:,1) - pos_data(:,1))
subplot(3,1,2)
plot(pos_all(:,2) - pos_data(:,2))
subplot(3,1,3)
plot(pos_all(:,3) - pos_data(:,3))

figure
plot(diff(sqrt( (pos_all(:,1) - pos_data(:,1)).^2 + (pos_all(:,2) - pos_data(:,2)).^2 + (pos_all(:,3) - pos_data(:,3)).^2 )))

figure
plot(diff(sqrt( (pos_all(:,1) - pos_data(:,1)).^2 + (pos_all(:,2) - pos_data(:,2)).^2 )))