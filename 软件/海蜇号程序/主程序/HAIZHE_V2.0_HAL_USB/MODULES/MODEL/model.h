#ifndef _MODEL_H
#define _MODEL_H

#include "sys.h"
#include "config.h"
#include "mymath.h"


#define CW 	0
#define CCW 1

//水动力系数
typedef struct{
	//线性流体动力项
	float Xu;	//单位：kg/s
	float Yv;
	float Zw;
	float Kp;	//单位：kg*m^2/s
	float Mq;
	float Nr;
	//附加质量
	float Xdu;	//单位：kg
	float Ydv;
	float Zdw;
	float Kdp;	//单位：kg*m^2
	float Mdq;
	float Ndr;
	//二阶流体动力项
	float Xddu;	//单位：kg/m
	float Yddv;
	float Zddw;
	float Kddp;	//单位：kg*m^2
	float Mddq;
	float Nddr;
}HYDRO_FAC;

//转动惯量
typedef struct{
	float Ix,Iy,Iz;
	float Ixy,Ixz;
	float Iyx,Iyz;
	float Izx,Izy;
}I_M;

enum DOF6{
	x = 0,
	y,
	z,
	roll,
	pitch,
	yaw,
};

enum MOTOR{
	M1 = 0,
	M2 ,
	M3 ,
	M4 ,
	M5 ,
	M6 ,
};

typedef struct{
	u8 Moter_Dir[6];				//电机正反桨
	vec6f motor_input;		//电机输入
	float DOF6_FM[6];		//电机6自由度力/力矩
	HYDRO_FAC hf;	//动力学系数
	float m;	//重量(kg)
	float G;	//重力(N)
	float B;	//浮力(N)
	float R;	//半径(m)
	float l;	//原点到推进器转轴距离(m)
	float Ix,Iy,Iz;//转动惯量
	vec3f Zg;	//重心坐标
//	I_M Im;		
	float dt;	//时间常数
	vec3f att;	//姿态
	vec3f ab;	//机体坐标系加速度
	vec3f vn;	//导航坐标系速度
	vec3f pos;	//导航坐标系经纬度（度） 
	
	bool model_init_complete;	//模型初始化完成
	//动力学系统方程
	arm_matrix_instance_f32 J;	//运动学转换矩阵
	arm_matrix_instance_f32 M, D;	//质量矩阵，阻尼矩阵
	arm_matrix_instance_f32 A, H, T, E, X, U, W;	//状态转移矩阵,输入转移矩阵,电机推力矩阵,恢复力矩和噪声转移矩阵,状态量,输入量,各自由度干扰力/力矩
	mat6f J_data, M_data, D_data, T_data;
	mat12f A_data;
	mat12_6f H_data, E_data;
	vec6f W_data;	
	/*状态量 x,y,z,roll,pitch,yaw,u,v,w,p,q,r
	分别是导航坐标系位移；姿态角；b系线速度；b系轴角速度；
	实际上x,y,z用于补偿SINS位置，姿态角直接用实际系统赋值，后6个用于验证动力学模型，
	*/
	vec12f X_data;
	vec6f U_data; //六个电机的推力
	//中间变量
	arm_matrix_instance_f32 X1, X2, X3, M_inv, MinvD, MinvT;
	vec12f X1_data, X2_data, X3_data;
	mat6f M_inv_data, MinvD_data, MinvT_data;
}MODEL;
extern MODEL model;


void model_init(vec3f att, vec3f vn, vec3f pos, float tk);
void model_update(vec3f att);
void model_updata_core(vec3f att, vec6f force);
float input_pwm_to_force(int type, u16 pwm);

#endif
