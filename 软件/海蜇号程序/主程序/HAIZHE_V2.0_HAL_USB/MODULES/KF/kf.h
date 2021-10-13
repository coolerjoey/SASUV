#ifndef __KF_H
#define __KF_H

   
#include <sys.h>
#include <stdlib.h>
#include "defines.h"
#include "mymath.h"
#include "mission.h"

#define PSINS_LOW_GRADE_MEMS //低精度惯导

typedef struct {	
	float Re, f, g0, wie;	//长半轴,椭圆扁率,重力,地球自转角速度 the Earth's parameters
	float e, e2;	//椭圆偏心率
	float mg, ug, deg, min, sec, hur, ppm, ppmpsh; 				// commonly used units
	float dps, dph, dpsh, dphpsh, ugpsh, ugpsHz, mpsh, mpspsh, secpsh;
}GLV;	//全局变量global variable
//地球参数
typedef struct {
	float sinL,cosL,tanL;	//L为地理纬度
	float RM;//地球子午圈(南北极圈)主曲率半径
	float RN;//地球卯酉圈(与子午圈垂直)主曲率半径
	float RMh;//RM+h,h为地理高度
	float RNh;
	
	vec3f wn;//n系角速度
	vec3f wni;	//n系随地球自转产生的角速度
	vec3f wne;	//n系因SINS移动时地球表面弯曲引起的旋转角速度
	vec3f gn;	//n系加速度
		
}EARTH;

typedef struct {	
	float ts,nts,tk;

	arm_matrix_instance_f32 wib,fb,fn,web,wnb,att,vb,tauGyro,tauAcc,_betaGyro,_betaAcc;
	vec3f wib_data,fb_data,fn_data,web_data,wnb_data,att_data,vb_data,eb,db,tauGyro_data,tauAcc_data,_betaGyro_data,_betaAcc_data;
	vec3f wn,an,vn,dn;	//角速度，加速度,速度,相对位移(m)，
	vec3d pos;	//经纬度(度)
	arm_matrix_instance_f32 Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp;
	mat3f Maa_data, Mav_data, Map_data, Mva_data, Mvv_data, Mvp_data, Mpv_data, Mpp_data;
	arm_matrix_instance_f32 Cnb, Cbn;	//旋转矩阵
	mat3f Cnb_data, Cbn_data;
}SINS;

typedef struct {	
	float tdts,kftk,tmeas;	//tmeas-测量间隔？
	int nq, nr;	//状态维数、量测维数
	bool measflag;	//量测标记
	bool KFinit_complete;
	arm_matrix_instance_f32 Ft, Pk, Hk, Pk1, Kk;//状态转移矩阵，估计协方差，观测矩阵，预测协方差，卡尔曼增益
	arm_matrix_instance_f32 Xk, Zk, Qt, Rt, rts, Zfd,
							Rmax, Rmin, Rbeta, Rb;				// measurement noise R adaptive 量测自适应方差上/下限制,遗忘参数beta/b
	vec15f Xmax, Pmax, Pmin;
	vec3f measGPSVn,measGPSPos;//gps测量信息
	mat15f Ft_data,Pk_data, Pk1_data;
	mat6_15f Hk_data;
	mat15_6f Kk_data;
	vec15f Xk_data, Qt_data, rts_data, Pmax_data, Pmin_data, Zfd_data;
	vec6f Zk_data, Rmax_data, Rmin_data, Rbeta_data, Rb_data;
	vec15f FBTau, FBMax, FBXk, FBTotal;// feedback control 反馈时间常数、反馈上限、当前反馈值、总反馈量
	mat6f Rt_data;
	
	//中间矩阵
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
