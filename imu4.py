import math
import os
import re

import matplotlib.pyplot as plt

import numpy as np

plt.rcParams.update({'font.size': 9})
import matplotlib.patches as patches
from pyproj import Transformer

fig=plt.figure()
ax=fig.add_subplot(111)

i=0

def a2qua(att):#返回四行一列
    att2=np.zeros(np.shape(att))
    s=np.zeros(np.shape(att))
    qnb=np.zeros(np.shape(att))
    att2=att/2
    s=np.sin(att2)
    c=np.cos(att2)
    sp=s[0,0]
    sr=s[1,0]
    sy=s[2,0]
    cp=c[0,0]
    cr=c[1,0]
    cy=c[2,0]    
    qnb=np.array([[cp*cr*cy-sp*sr*sy],[sp*cr*cy-cp*sr*sy],[cp*sr*cy+sp*cr*sy],[cp*cr*sy+sp*sr*cy]])
    return qnb


def attsyn(attForm):#四行一列
    
    m=attForm.shape[0]
    n=attForm.shape[1]
    qnb=attForm;#四行一列
    Cnb=q2mat(qnb)#cnb:三行三列
    att=m2att(Cnb)#att:三行两列
    attsyn11=np.array([qnb, att, Cnb])
    return attsyn11

def m2att(Cnb):
    att=np.array([[np.arcsin(Cnb[2,1])],[math.atan2(-Cnb[2,0],Cnb[2,2])],[math.atan2(-Cnb[0,1],Cnb[1,1])]])
    return att

def q2mat(qnb):
    q11=qnb[0,0]*qnb[0,0]
    q12=qnb[0,0]*qnb[1,0]
    q13=qnb[0,0]*qnb[2,0]
    q14=qnb[0,0]*qnb[3,0] 
    q22=qnb[1,0]*qnb[1,0]
    q23=qnb[1,0]*qnb[2,0]
    q24=qnb[1,0]*qnb[3,0]   
    q33=qnb[2,0]*qnb[2,0]
    q34=qnb[2,0]*qnb[3,0]
    q44=qnb[3,0]*qnb[3,0]
    Cnb=np.array([[q11+q22-q33-q44, 2*(q23-q14),2*(q24+q13)],[2*(q23+q14),q11-q22+q33-q44,2*(q34-q12)],[2*(q24-q13),2*(q34+q12),q11-q22-q33+q44]])
    return Cnb

def imu_int2(pos,vel,att,delta_t):
    g0=np.array([[0],[0],[-9.8]])
    qnb0=a2qua(att)#四行一列
    acc_n=qmulv(qnb0)#三行一列
    acc_n=acc_n+g0
    vel_new=vel+acc_n*delta_t
    pos_new=pos+(vel+vel_new)/2*delta_t
    gyro=np.array([[-0.0072],[-0.0014],[0.0035]])
    gyro_b=gyro
    qnb_new=qupdt2(qnb0,gyro_b*delta_t,np.zeros((3,1)))
    att_new=q2att(qnb_new)
    return [pos_new,vel_new,att_new]

def qmulv(q):#q(4,1)#vi是什么值是不是第一列的值
    qo1=-q[0,0]*0.8159-q[2,0]*0.7264-q[3,0]*0.7238
    qo2=q[0,0]*0.8159+q[2,0]*0.7238-q[3,0]*0.7264
    qo3=q[0,0]*0.7264+q[3,0]*-0.0072-q[1,0]*0.7238
    qo4=q[0,0]*0.7238+q[1,0]*0.7264-q[2,0]*0.8159
    vo=np.zeros((3,1))
    vo[0,0]=-qo1*q[1,0]+qo2*q[0,0]-qo3*q[3,0]+qo4*q[2,0]
    vo[1,0]=-qo1*q[2,0]+qo3*q[0,0]-qo4*q[1,0]+qo2*q[3,0]
    vo[2,0]=-qo1*q[3,0]+qo4*q[0,0]-qo2*q[2,0]+qo3*q[1,0]
    return vo

def qupdt2(qnb0,rv_ib,rv_in):
    n2=rv_ib[0,0]*rv_ib[0,0]+rv_ib[1,0]*rv_ib[1,0]+rv_ib[2,0]*rv_ib[2,0];
    if n2<1.0e-08:
        rv_ib0=1-n2*(1/8-n2/384)
        s=1/2-n2*(1/48-n2/3840)
    else:
        n=np.sqrt(n2)
        n_2=n/2
        rv_ib0=np.cos(n_2)
        s=np.sin(n_2)/n
    rv_ib[0,0]=s*rv_ib[0,0]
    rv_ib[1,0]=s*rv_ib[1,0]
    rv_ib[2,0]=s*rv_ib[2,0]
    qb1=qnb0[0,0]*rv_ib0-qnb0[1,0]*rv_ib[0,0]-qnb0[2,0]*rv_ib[1,0]-qnb0[3,0]*rv_ib[2,0]
    qb2=qnb0[0,0]*rv_ib[0,0]+qnb0[1,0]*rv_ib0+qnb0[2,0]*rv_ib[2,0]-qnb0[3,0]*rv_ib[1,0]
    qb3=qnb0[0,0]*rv_ib[1,0]+qnb0[2,0]*rv_ib0+qnb0[3,0]*rv_ib[0,0]-qnb0[1,0]*rv_ib[2,0]
    qb4=qnb0[0,0]*rv_ib[2,0]+qnb0[3,0]*rv_ib0+qnb0[1,0]*rv_ib[1,0]-qnb0[2,0]*rv_ib[0,0]
    n2=rv_in[0,0]*rv_in[0,0]+rv_in[1,0]*rv_in[1,0]+rv_in[2,0]*rv_in[2,0]
    if n2<1.0e-08:
        rv_in0=1-n2*(1/8-n2/384)
        s=-1/2+n2*(1/48-n2/3840)
    else:
        n=np.sqrt(n2)
        n_2=n/2
        rv_in0=np.cos(n_2)
        s=-np.sin(n_2)/n
    rv_in[0,0]=s*rv_in[0,0]
    rv_in[1,0]=s*rv_in[1,0]
    rv_in[2,0]=s*rv_in[2,0] 
    qnb1=np.zeros((4,1)) 
    qnb1[0,0]=rv_in0*qb1-rv_in[0,0]*qb2-rv_in[1,0]*qb3-rv_in[2,0]*qb4;
    qnb1[0,0]=rv_in0*qb2+rv_in[0,0]*qb1+rv_in[1,0]*qb4-rv_in[2,0]*qb3;
    qnb1[2,0]=rv_in0*qb3+rv_in[1,0]*qb1+rv_in[2,0]*qb2-rv_in[0,0]*qb4;
    qnb1[3,0]=rv_in0*qb4+rv_in[2,0]*qb1+rv_in[0,0]*qb3-rv_in[1,0]*qb2;
    n2=qnb1[0,0]*qnb1[0,0]+qnb1[1,0]*qnb1[1,0]+qnb1[2,0]*qnb1[2,0]+qnb1[3,0]*qnb1[3,0]
    if ((n2>1.000001)|(n2<0.999999)):
        nq=1/np.sqrt(n2)
        qnb1[0,0]=qnb1[0,0]*nq
        qnb1[1,0]=qnb1[1,0]*nq
        qnb1[2,0]=qnb1[2,0]*nq
        qnb1[3,0]=qnb1[3,0]*nq
    return qnb1

def q2att(qnb):
    q11=qnb[0,0]*qnb[0,0]
    q12=qnb[0,0]*qnb[1,0]
    q13=qnb[0,0]*qnb[2,0]
    q14=qnb[0,0]*qnb[3,0] 
    q22=qnb[1,0]*qnb[1,0]
    q23=qnb[1,0]*qnb[2,0]
    q24=qnb[1,0]*qnb[3,0]    
    q33=qnb[2,0]*qnb[2,0]
    q34=qnb[2,0]*qnb[3,0] 
    q44=qnb[3,0]*qnb[3,0]
    C12=2*(q23-q14)
    C22=q11-q22+q33-q44
    C31=2*(q24-q13)
    C32=2*(q34+q12)
    C33=q11-q22-q33+q44
    att=np.array([[math.asin(C32)],[math.atan2(-C31,C33)],[math.atan2(-C12,C22)]])
    return att


import math
import os
import re

import numpy as np

i=0

import math
import os
import re

import numpy as np

i=0

#数据整理
i=0
data_x=np.zeros((846,30))
oxts_path='/home/zhushuai/桌面/2011_09_26/2011_09_26_drive_0002_extract/oxts/data/'
path_list=os.listdir(oxts_path)
path_list.sort()
#数据1数据
while i<846:
    oxts_fullpath=oxts_path+path_list[i]
    data_x[i,:]=np.loadtxt(oxts_fullpath)
    i=i+1

#数据1时间
fullpath='/home/zhushuai/桌面/2011_09_26/2011_09_26_drive_0002_extract/oxts/timestamps.txt'
file=open(fullpath,'r')
list_arr=file.readlines()
l=len(list_arr)
i=0
imu_timestamps_x=np.zeros(l)
for i in range(l):
	imu_timestamps_x[i]=int(list_arr[i][11:13])*3600+int(list_arr[i][14:16])*60+int(list_arr[i][17:19])+int(list_arr[i][20:29])/1000000000
#数据2时间
fullpath='/home/zhushuai/桌面/2011_09_26/2011_09_26_drive_0002_extract/image_00/timestamps.txt'
file=open(fullpath,'r')
list_arr=file.readlines()
l=len(list_arr)
image_timestamps_x=np.zeros(l)
for i in range(l):
	image_timestamps_x[i]=int(list_arr[i][11:13])*3600+int(list_arr[i][14:16])*60+int(list_arr[i][17:19])+int(list_arr[i][20:29])/1000000000

#时间逼近
d=np.copy(imu_timestamps_x)
a1=np.zeros(len(image_timestamps_x))
i=0
j=0
lb=len(image_timestamps_x)
ld=len(d)
while(i<lb):
    c=np.absolute(d-image_timestamps_x[i])
    low=min(c)
    g=int(np.argwhere(c==low))
    a1[i]=d[g]
    d=d[g+1:ld]
    i=i+1

#获得时间逼近后的数据
la1=len(a1)
date=np.zeros((la1,30))
i=0
while(i<83):
    g=int(np.argwhere(imu_timestamps_x==a1[i]))
    date[i,:]=data_x[g,:]
    i=i+1

data_x=np.copy(date)
#获得时间逼近后的时间
imu_timestamps_x=np.copy(a1)

#计算积分值
pos0=np.array([[0,0,0]]).T
#print(np.shape(pos0))
vel0=np.array([[data_x[0,7],data_x[0,6],data_x[0,10]]]).T
att0=np.array([[-data_x[0,4],data_x[0,3],data_x[0,5]-math.pi/2]]).T
bias_a=np.array([[0],[0],[0.01]])#三行一列
bias_g=np.array([[0],[0],[0]])#三行一列
pos_all=np.zeros((len(imu_timestamps_x),3))
vel_all=np.zeros((len(imu_timestamps_x),3))
att_all=np.zeros((len(imu_timestamps_x),3))
pos_all[0,:]=pos0.T
vel_all[0,:]=vel0.T
att_all[0,:]=att0.T
att=np.array([[0],[0],[math.pi/2]])
q_rfu=a2qua(att)
[q_rfu,att_rfu,C_rfu]=attsyn(q_rfu)
i=1
while i<len(imu_timestamps_x):#注意pos_new是多少列，然后输出即可
    delta_t=imu_timestamps_x[i]-imu_timestamps_x[i-1]
    [pos_new,vel_new,att_new]=imu_int2(pos0,vel0,att0,delta_t)
    pos_all[i,:]=pos_new.T
    vel_all[i,:]=vel_new.T
    att_all[i,:]=att_new.T
    pos0=pos_new
    vel0=vel_new
    att0=att_new
    i=i+1


imu_timestamps_x=imu_timestamps_x-imu_timestamps_x[0]

#print(imu_timestamps_x[1]-imu_timestamps_x[0])
#ax.plot(imu_timestamps_x,-pos_all[:,0],label="GNSS",linewidth=1)
#plt.show()
plt.plot(-pos_all[:,0],-pos_all[:,1],linewidth=1)

#print(pos_all)
#print(vel_all)
#print(att_all)
#print(np.shape(data_x))
#将经纬度坐标转化为局部坐标
Re=6378137
pos_y=data_x[:,0]/180*math.pi*Re-data_x[0,0]/180*math.pi*Re
pos_x=data_x[:,1]/180*math.pi*Re*np.cos(data_x[0,0]/180*math.pi)-data_x[0,1]/180*math.pi*Re*np.cos(data_x[0,0]/180*math.pi)
plt.plot(-pos_x,-pos_y,linewidth=1)
plt.show()
#将所得到的三维空间下的imu测量数据取x，y下的二维数据
pos_all=pos_all[:,0:2]
vel_all=vel_all[:,0:2]
att_all=att_all[:,0:2]
print(pos_all[1,1]-pos_all[2,1])
i=0
distance_x=np.zeros(82)
distance_y=np.zeros(82)
print(pos_x[2])
while i<82:
    distance_x[i]=pos_all[i,0]-pos_all[i+1,0]+pos_x[i+1]-pos_x[i]
    distance_y[i]=pos_all[i,1]-pos_all[i+1,1]+pos_y[i+1]-pos_y[i]
    i=i+1


imu_timestamps_x=imu_timestamps_x[0:82]
plt.plot(imu_timestamps_x,-distance_x,linewidth=1)
plt.plot(imu_timestamps_x,distance_y,linewidth=1)
plt.show()

#print("x方向",distance_x)
#print("y方向",distance_y)




