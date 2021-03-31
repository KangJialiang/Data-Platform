import math
import os
import re

import matplotlib.pyplot as plt

import numpy as np

plt.rcParams.update({'font.size': 9})
import matplotlib.patches as patches
from pyproj import Transformer

fig=plt.figure()
ax=fig.add_subplot(111, projection='3d')

i=0

def a2qua(att):#返回四行一列ok

    att2=att/2
    s=np.sin(att2)
    c=np.cos(att2)
    sp=s[0]
    sr=s[1]
    sy=s[2]
    cp=c[0]
    cr=c[1]
    cy=c[2]    
    qnb=np.array([cp*cr*cy-sp*sr*sy,sp*cr*cy-cp*sr*sy,cp*sr*cy+sp*cr*sy,cp*cr*sy+sp*sr*cy])
    return qnb

def gt_unpack_imu(imu_path):
    file=open(imu_path,'r')
    list_arr=file.readlines()[1 :]
    times = []
    accs = []
    gyros =[]
    print(len(list_arr))
    for line in list_arr:
        info=np.array(line.strip('\n').split(' ')).astype(np.float)
        times.append(info[0])
        accs.append(info[2: 5])
        gyros.append(info[5: 8])
    print(len(times))
    return np.array(times),np.array(accs),np.array(gyros)

def gt_unpack_xyz(xyz_path):
    file=open(xyz_path,'r')
    list_arr=file.readlines()[1 :]
    times=[]
    velos=[]
    xyzs=[]
    qatts=[]
    print(len(list_arr))
    for line in list_arr:
        info=np.array(line.strip('\n').split(',')).astype(np.float)
        times.append(info[0])
        qatts.append(info[1:5])
        velos.append(info[5:8])
        xyzs.append(info[8:11])
    print(len(times))
    return np.array(times),np.array(qatts),np.array(velos),np.array(xyzs)

def attsyn(attForm):#四行一列
    
    
    qnb=attForm;#四行一列
    Cnb=q2mat(qnb)#cnb:三行三列
    att=m2att(Cnb)#att:三行一列
    #print(attForm.shape,Cnb.shape,att.shape)
    return qnb, att, Cnb.squeeze()

def m2att(Cnb):#ok
    att=np.array([np.arcsin(Cnb[2,1]),math.atan2(-Cnb[2,0],Cnb[2,2]),math.atan2(-Cnb[0,1],Cnb[1,1])])
    #print(np.arcsin(Cnb[2,1,0]),math.atan2(-Cnb[2,0],Cnb[2,2]),math.atan2(-Cnb[0,1],Cnb[1,1]),Cnb.shape)
    return att

def q2mat(qnb):#ok
    q11=qnb[0]*qnb[0]
    q12=qnb[0]*qnb[1]
    q13=qnb[0]*qnb[2]
    q14=qnb[0]*qnb[3] 
    q22=qnb[1]*qnb[1]
    q23=qnb[1]*qnb[2]
    q24=qnb[1]*qnb[3]   
    q33=qnb[2]*qnb[2]
    q34=qnb[2]*qnb[3]
    q44=qnb[3]*qnb[3]
    Cnb=np.array([[q11+q22-q33-q44, 2*(q23-q14),2*(q24+q13)],[2*(q23+q14),q11-q22+q33-q44,2*(q34-q12)],[2*(q24-q13),2*(q34+q12),q11-q22-q33+q44]])
    return Cnb

def imu_int2(pos,vel,att,acc,gyro,bias_a,bias_g,delta_t):
    #print(delta_t)
    g0=np.array([0,0,-9.7955])
    qnb0=a2qua(att)#四行一列
    acc_n=qmulv(qnb0,acc.squeeze()-bias_a)#三行一列
    #print(acc_n.shape)
    acc_n=acc_n+g0
    vel_new=vel+acc_n*delta_t
    
    pos_new=pos+(vel+vel_new)/2*delta_t
    
    gyro_b=(gyro.squeeze()-bias_g)
    qnb_new=qupdt2(qnb0,gyro_b*delta_t,np.zeros((3,1)))
    att_new=q2att(qnb_new)
    #print(qnb0.shape,acc_n.shape,vel_new.shape,acc.shape)
    return pos_new,vel_new,att_new

def qmulv(q,vi):#ok
    #print(vi.shape)
    qo1 =              - q[1] * vi[0] - q[2] * vi[1] - q[3] * vi[2]
    qo2 = q[0] * vi[0]                + q[2] * vi[2] - q[3] * vi[1]
    qo3 = q[0] * vi[1]                + q[3] * vi[0] - q[1] * vi[2]
    qo4 = q[0] * vi[2]                + q[1] * vi[1] - q[2] * vi[0]
    #print(qo1,qo2,qo3,qo4)
    vo = vi
    vo[0] = -qo1 * q[1] + qo2 * q[0] - qo3 * q[3] + qo4 * q[2]
    vo[1] = -qo1 * q[2] + qo3 * q[0] - qo4 * q[1] + qo2 * q[3]
    vo[2] = -qo1 * q[3] + qo4 * q[0] - qo2 * q[2] + qo3 * q[1]
    #print(qo1,qo2,qo3,qo4)
    return vo

def qupdt2(qnb0,rv_ib,rv_in):#ok
    n2=rv_ib[0]*rv_ib[0]+rv_ib[1]*rv_ib[1]+rv_ib[2]*rv_ib[2]
    if n2<1.0e-08:
        rv_ib0=1-n2*(1/8-n2/384)
        s=1/2-n2*(1/48-n2/3840)
    else:
        n=np.sqrt(n2)
        n_2=n/2
        rv_ib0=np.cos(n_2)
        s=np.sin(n_2)/n
    rv_ib[0]=s*rv_ib[0]
    rv_ib[1]=s*rv_ib[1]
    rv_ib[2]=s*rv_ib[2]
    qb1=qnb0[0]*rv_ib0-qnb0[1]*rv_ib[0]-qnb0[2]*rv_ib[1]-qnb0[3]*rv_ib[2]
    qb2=qnb0[0]*rv_ib[0]+qnb0[1]*rv_ib0+qnb0[2]*rv_ib[2]-qnb0[3]*rv_ib[1]
    qb3=qnb0[0]*rv_ib[1]+qnb0[2]*rv_ib0+qnb0[3]*rv_ib[0]-qnb0[1]*rv_ib[2]
    qb4=qnb0[0]*rv_ib[2]+qnb0[3]*rv_ib0+qnb0[1]*rv_ib[1]-qnb0[2]*rv_ib[0]
    n2=rv_in[0]*rv_in[0]+rv_in[1]*rv_in[1]+rv_in[2]*rv_in[2]
    if n2<1.0e-08:
        rv_in0=1-n2*(1/8-n2/384)
        s=-1/2+n2*(1/48-n2/3840)
    else:
        n=np.sqrt(n2)
        n_2=n/2
        rv_in0=np.cos(n_2)
        s=-np.sin(n_2)/n
    rv_in[0]=s*rv_in[0]
    rv_in[1]=s*rv_in[1]
    rv_in[2]=s*rv_in[2] 
    qnb1=np.zeros((4,1)) 
    qnb1[0]=rv_in0*qb1-rv_in[0]*qb2-rv_in[1]*qb3-rv_in[2]*qb4
    qnb1[0]=rv_in0*qb2+rv_in[0]*qb1+rv_in[1]*qb4-rv_in[2]*qb3
    qnb1[2]=rv_in0*qb3+rv_in[1]*qb1+rv_in[2]*qb2-rv_in[0]*qb4
    qnb1[3]=rv_in0*qb4+rv_in[2]*qb1+rv_in[0]*qb3-rv_in[1]*qb2
    n2=qnb1[0]*qnb1[0]+qnb1[1]*qnb1[1]+qnb1[2]*qnb1[2]+qnb1[3]*qnb1[3]
    if ((n2>1.000001)|(n2<0.999999)):
        nq=1/np.sqrt(n2)
        qnb1[0]=qnb1[0]*nq
        qnb1[1]=qnb1[1]*nq
        qnb1[2]=qnb1[2]*nq
        qnb1[3]=qnb1[3]*nq
    return qnb1

def q2att(qnb):#ok
    q11=qnb[0]*qnb[0]
    q12=qnb[0]*qnb[1]
    q13=qnb[0]*qnb[2]
    q14=qnb[0]*qnb[3] 
    q22=qnb[1]*qnb[1]
    q23=qnb[1]*qnb[2]
    q24=qnb[1]*qnb[3]    
    q33=qnb[2]*qnb[2]
    q34=qnb[2]*qnb[3] 
    q44=qnb[3]*qnb[3]
    C12=2*(q23-q14)
    C22=q11-q22+q33-q44
    C31=2*(q24-q13)
    C32=2*(q34+q12)
    C33=q11-q22-q33+q44
    att=np.array([math.asin(C32),math.atan2(-C31,C33),math.atan2(-C12,C22)])
    return att

imu_path='/home/ubuntu/Data-Collector/imu_gt.txt'
imu_timestamps_x,acc,gyro=gt_unpack_imu(imu_path)
xyz_path='/home/ubuntu/Data-Collector/avp_gt.txt'
time_xyz,qatt_xyz,velo_xyz,xyzs=gt_unpack_xyz(xyz_path)
l=imu_timestamps_x.shape[0]
'''
acc=np.zeros((l,3))
acc_tmp=np.zeros(3)
gyro=np.zeros((l,3))
gyro_tmp=np.zeros(3)
imu_timestamps_x=np.zeros(l)
for i in range(l):
    line=list_arr[i].strip('\n').split(' ')
    imu_timestamps_x[i]=int(line[0])
    acc[i]=np.array(line[2:5]).astype(float)
    acc_tmp[0]=acc[i,0]
    acc_tmp[1]=-acc[i,2]
    acc_tmp[2]=acc[i,1]
    acc[i]=acc_tmp
    gyro[i]=np.array(line[5:8]).astype(float)
    gyro_tmp[0]=gyro[i,0]
    gyro_tmp[1]=-gyro[i,2]
    gyro_tmp[2]=gyro[i,1]
    gyro[i]=gyro_tmp
    #print(acc[i],gyro[i])
    '''
#计算积分值
pos0=xyzs[0]
#print(np.shape(pos0))
vel0=velo_xyz[0]
att0=q2att(qatt_xyz[0])
pos_all=np.zeros((l,3))
vel_all=np.zeros((l,3))
att_all=np.zeros((l,3))
pos_all[0,:]=pos0
vel_all[0,:]=vel0
att_all[0,:]=att0
#att=np.array([0,0,math.pi/2])
q_rfu,att_rfu,C_rfu=attsyn(qatt_xyz[0])
i=1
#bias_g=np.array([-0.0072,-0.0014,0.0035])
bias_g=np.array([0,0,0])
#bias_a=np.array([0.308,0.16,0])
bias_a=np.array([0,0,0])
while i<len(imu_timestamps_x):#注意pos_new是多少列，然后输出即可
    #delta_t=(imu_timestamps_x[i]-imu_timestamps_x[i-1])/1000.0
    delta_t=(imu_timestamps_x[i]-imu_timestamps_x[i-1])
    #print(np.matmul(C_rfu,acc[i,:].reshape(-1,1)).shape,C_rfu.shape)
    pos_new,vel_new,att_new=imu_int2(pos0,vel0,att0,np.matmul(C_rfu,acc[i,:].reshape(-1,1)),np.matmul(C_rfu,gyro[i,:].reshape(-1,1)),bias_a,bias_g,delta_t)
    pos_all[i,:]=pos_new
    vel_all[i,:]=vel_new
    att_all[i,:]=att_new
    print(i,pos_new-xyzs[i],vel_new-velo_xyz[i],att_new-q2att(qatt_xyz[i]))
    pos0=pos_new
    vel0=vel_new
    att0=att_new
    i=i+1
plt.plot(pos_all[:,0],pos_all[:,1],pos_all[:,2],linewidth=1,c='g')
plt.plot(xyzs[:,0],xyzs[:,1],xyzs[:,2],linewidth=1,c='b')
ax.set_xlabel('variable X')
ax.set_ylabel('variable Y')
ax.set_zlabel('variable Z')
plt.savefig('imu_traj.png')
plt.show()




