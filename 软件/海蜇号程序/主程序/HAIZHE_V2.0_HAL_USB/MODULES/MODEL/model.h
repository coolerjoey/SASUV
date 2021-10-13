#ifndef _MODEL_H
#define _MODEL_H

#include "sys.h"
#include "config.h"
#include "mymath.h"


#define CW 	0
#define CCW 1

//ˮ����ϵ��
typedef struct{
	//�������嶯����
	float Xu;	//��λ��kg/s
	float Yv;
	float Zw;
	float Kp;	//��λ��kg*m^2/s
	float Mq;
	float Nr;
	//��������
	float Xdu;	//��λ��kg
	float Ydv;
	float Zdw;
	float Kdp;	//��λ��kg*m^2
	float Mdq;
	float Ndr;
	//�������嶯����
	float Xddu;	//��λ��kg/m
	float Yddv;
	float Zddw;
	float Kddp;	//��λ��kg*m^2
	float Mddq;
	float Nddr;
}HYDRO_FAC;

//ת������
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
	u8 Moter_Dir[6];				//���������
	vec6f motor_input;		//�������
	float DOF6_FM[6];		//���6���ɶ���/����
	HYDRO_FAC hf;	//����ѧϵ��
	float m;	//����(kg)
	float G;	//����(N)
	float B;	//����(N)
	float R;	//�뾶(m)
	float l;	//ԭ�㵽�ƽ���ת�����(m)
	float Ix,Iy,Iz;//ת������
	vec3f Zg;	//��������
//	I_M Im;		
	float dt;	//ʱ�䳣��
	vec3f att;	//��̬
	vec3f ab;	//��������ϵ���ٶ�
	vec3f vn;	//��������ϵ�ٶ�
	vec3f pos;	//��������ϵ��γ�ȣ��ȣ� 
	
	bool model_init_complete;	//ģ�ͳ�ʼ�����
	//����ѧϵͳ����
	arm_matrix_instance_f32 J;	//�˶�ѧת������
	arm_matrix_instance_f32 M, D;	//���������������
	arm_matrix_instance_f32 A, H, T, E, X, U, W;	//״̬ת�ƾ���,����ת�ƾ���,�����������,�ָ����غ�����ת�ƾ���,״̬��,������,�����ɶȸ�����/����
	mat6f J_data, M_data, D_data, T_data;
	mat12f A_data;
	mat12_6f H_data, E_data;
	vec6f W_data;	
	/*״̬�� x,y,z,roll,pitch,yaw,u,v,w,p,q,r
	�ֱ��ǵ�������ϵλ�ƣ���̬�ǣ�bϵ���ٶȣ�bϵ����ٶȣ�
	ʵ����x,y,z���ڲ���SINSλ�ã���̬��ֱ����ʵ��ϵͳ��ֵ����6��������֤����ѧģ�ͣ�
	*/
	vec12f X_data;
	vec6f U_data; //�������������
	//�м����
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
