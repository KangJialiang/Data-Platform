function [pos_new,vel_new,att_new] = imu_int2(pos,vel,att,acc,gyro,bias_a,bias_g,delta_t)
%%
%   全局坐标系：东北天，经纬高
%   车身坐标系：右前上
%   pos,vel,att 全局坐标
%   acc,gyro 车身坐标
%   bias_a bias_g 加速度计、陀螺仪零偏
% 

%%
g0 = [0; 0; -9.8];

qnb0 = a2qua(att);
% [qnb0, att0, Cnb0] = attsyn(qnb0);
acc_n = qmulv(qnb0, acc - bias_a);
acc_n = acc_n + g0;
% acc_n = Cnb0*(acc - bias_a) + g0;

vel_new = vel + acc_n*delta_t;

pos_new = pos + (vel + vel_new)/2 * delta_t;

% gyro_n = Cnb0*(gyro - bias_g);
% att_delta = gyro_n*delta_t;
gyro_b = (gyro - bias_g);
qnb_new = qupdt2(qnb0, gyro_b*delta_t, zeros(3,1));
att_new = q2att(qnb_new);

end



