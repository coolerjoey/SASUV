#ifndef __KF_H
#define __KF_H

   
#include <sys.h>
#include <stdlib.h>
#include "defines.h"
#include "mymath.h"
#include "mission.h"

#define PSINS_LOW_GRADE_MEMS //�;��ȹߵ�

typedef struct {	
	float Re, f, g0, wie;	//������,��Բ����,����,������ת���ٶ� the Earth's parameters
	float e, e2;	//��Բƫ����
	float mg, ug, deg, min, sec, hur, ppm, ppmpsh; 				// commonly used units
	float dps, dph, dpsh, dphpsh, ugpsh, ugpsHz, mpsh, mpspsh, secpsh;
}GLV;	//ȫ�ֱ���global variable
//�������
typedef struct {
	float sinL,cosL,tanL;	//LΪ����γ��
	float RM;//��������Ȧ(�ϱ���Ȧ)�����ʰ뾶
	float RN;//����î��Ȧ(������Ȧ��ֱ)�����ʰ뾶
	float RMh;//RM+h,hΪ����߶�
	float RNh;
	
	vec3f wn;//nϵ���ٶ�
	vec3f wni;	//nϵ�������ת�����Ľ��ٶ�
	vec3f wne;	//nϵ��SINS�ƶ�ʱ������������������ת���ٶ�
	vec3f gn;	//nϵ���ٶ�
		
}EARTH;

typedef struct {	
	float ts,nts,tk;

	arm_matrix_instance_f32 wib,fb,fn,web,wnb,att,vb,tauGyro,tauAcc,_betaGyro,_betaAcc;
	vec3f wib_data,fb_data,fn_data,web_data,wnb_data,att_data,vb_data,eb,db,tauGyro_data,tauAcc_data,_betaGyro_data,_betaAcc_data;
	vec3f wn,an,vn,dn;	//���ٶȣ����ٶ�,�ٶ�,���λ��(m)��
	vec3d pos;	//��γ��(��)
	arm_matrix_instance_f32 Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp;
	mat3f Maa_data, Mav_data, Map_data, Mva_data, Mvv_data, Mvp_data, Mpv_data, Mpp_data;
	arm_matrix_instance_f32 Cnb, Cbn;	//��ת����
	mat3f Cnb_data, Cbn_data;
}SINS;

typedef struct {	
	float tdts,kftk,tmeas;	//tmeas-���������
	int nq, nr;	//״̬ά��������ά��
	bool measflag;	//������
	bool KFinit_complete;
	arm_matrix_instance_f32 Ft, Pk, Hk, Pk1, Kk;//״̬ת�ƾ��󣬹���Э����۲����Ԥ��Э�������������
	arm_matrix_instance_f32 Xk, Zk, Qt, Rt, rts, Zfd,
							Rmax, Rmin, Rbeta, Rb;				// measurement noise R adaptive ��������Ӧ������/������,��������beta/b
	vec15f Xmax, Pmax, Pmin;
	vec3f measGPSVn,measGPSPos;//gps������Ϣ
	mat15f Ft_data,Pk_data, Pk1_data;
	mat6_15f Hk_data;
	mat15_6f Kk_data;
	vec15f Xk_data, Qt_data, rts_data, Pmax_data, Pmin_data, Zfd_data;
	vec6f Zk_data, Rmax_data, Rmin_data, Rbeta_data, Rb_data;
	vec15f FBTau, FBMax, FBXk, FBTotal;// feedback control ����ʱ�䳣�����������ޡ���ǰ����ֵ���ܷ�����
	mat6f Rt_data;
	
	//�м����
	arm_matrix_instance_f32 HT, PHT, HP, Sk, Sk_inv, Y, Dx, KHP, FtT, Xk1;
	mat15f KHP_data,FtT_data;
	mat15_6f HT_data, PHT_data;
	mat6_15f HP_data;
	mat6f Sk_data,Sk_inv_data;
	vec6f Y_data;
	vec15f Dx_data;
	vec15f Xk1_data;
}KF;

extern SINS sins;
extern KF kf;
extern GLV glv;
extern EARTH eth;

void KF_SetMeas(vec3f gpsvn, vec3f gpspos, float tm);
void KF_init(vec3f vn, vec3f posn, float tk);
void KF_update(vec3f att, vec3f wb, vec3f ab,int nSamples, double ts, int nStep);   
void DCM_update(vec3f att, mat3f vec, arm_matrix_instance_f32 *R);

#endif /*__LOGGER_H*/
