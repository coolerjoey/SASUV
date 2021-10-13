#include "kf.h"
#include "parameter.h"

//注意矩阵乘法，转置，求逆等操作的源矩阵和输出矩阵不能是同一个矩阵！！！


GLV glv;
EARTH eth;
SINS sins;
KF kf;

void GLV_init(){
	glv.Re = 6378137.0;
	glv.f = 1.0/298.257;
	glv.wie = 7.2921151467e-5;
	glv.g0 = GRAVITY_MSS;
	glv.e = sqrt(2*glv.f-glv.f*glv.f); glv.e2 = glv.e*glv.e;
	glv.mg = glv.g0/1000.0f;
	glv.ug = glv.mg/1000.0f;
	glv.deg = PI/180.0f;		//度
	glv.min = glv.deg/60.0f;	//分
	glv.sec = glv.min/60.0f;	//秒
	glv.ppm = 1.0e-6;
	glv.hur = 3600.0;
	glv.dps = glv.deg/1.0f;//degree per second
	glv.dph = glv.deg/glv.hur;
	glv.dpsh = glv.deg/sqrt(glv.hur);
	glv.dphpsh = glv.dph/sqrt(glv.hur);
	glv.ugpsHz = glv.ug/sqrt(1.0);
	glv.ugpsh = glv.ug/sqrt(glv.hur);
	glv.mpsh = 1/sqrt(glv.hur); 
	glv.mpspsh = 1/1/sqrt(glv.hur);
	glv.ppmpsh = glv.ppm/sqrt(glv.hur);
	glv.secpsh = glv.sec/sqrt(glv.hur);
}
void EARTH_init(vec3f pos){
#ifdef PSINS_LOW_GRADE_MEMS
	arm_sin_cos_f32(pos[0]*glv.deg, &eth.sinL, &eth.cosL);	//注意角度转弧度
	eth.tanL = eth.sinL/eth.cosL;
	float sq = 1-glv.e2*eth.sinL*eth.sinL;
	float sq2 ;
	arm_sqrt_f32(sq,&sq2);
	eth.RM = glv.Re*(1-glv.e2)/sq/sq2;
	eth.RMh = eth.RM + pos[2];
	eth.RN = glv.Re/sq2;
	eth.RNh = eth.RN + pos[2];
	float sl2 = eth.sinL*eth.sinL;
	eth.gn[0] = eth.gn[1] = 0;
	eth.gn[2] = ( glv.g0*(1+5.27094e-3f*sl2)-3.086e-6f*pos[2] );
#else
	
#endif

}
//地球参数计算->计算状态转移矩阵
void EARTH_update(vec3f pos,vec3f vn){
#ifdef PSINS_LOW_GRADE_MEMS
	arm_sin_cos_f32(pos[0], &eth.sinL, &eth.cosL);
	eth.tanL = eth.sinL/eth.cosL;
	float sq = 1-glv.e2*eth.sinL*eth.sinL;
	float sq2 ;
	arm_sqrt_f32(sq,&sq2);
	eth.RM = glv.Re*(1-glv.e2)/sq/sq2;
	eth.RMh = eth.RM + pos[2];
	eth.RN = glv.Re/sq2;
	eth.RNh = eth.RN + pos[2];
	eth.wni[0] = 0;eth.wni[1] = glv.wie*eth.cosL;eth.wni[2] = glv.wie*eth.sinL;
	eth.wne[0] = -vn[0]/eth.RMh; eth.wne[1]=-vn[1]/eth.RNh; eth.wne[2] = -vn[1]*eth.tanL/eth.RNh;
	arm_add_f32(eth.wni, eth.wne, eth.wn, 3);
	float sl2 = eth.sinL*eth.sinL;
	eth.gn[0] = eth.gn[1] = 0;
	eth.gn[2] = ( glv.g0*(1+5.27094e-3f*sl2)-3.086e-6f*pos[2] );
#else

#endif
}

void SINS_init(vec3f vn, vec3f posn, float tk){
	EARTH_init(posn);
//	EARTH_update(posn, vn);

	sins.tk=tk; 
	sins.ts=sins.nts= 1.0;
	arm_copy_f32(O31, sins.an, 3);
	arm_copy_f32(O31, sins.dn, 3);
	arm_copy_f32(vn, sins.vn, 3);
//	arm_scale_f32(posn, 1e7f, sins.pos, 3);
//	arm_copy_f32(posn, sins.pos, 3);
	for(int i=0;i<3;++i) sins.pos[i]=posn[i];

//	vec3f eb={0.9,0.6,-0.8}; // 陀螺零偏 deg/s
	arm_copy_f32(O31, sins.eb, 3);
	arm_copy_f32(O31, sins.db, 3);

	set2matf(*I33,3,3,*sins.Maa_data,&sins.Maa);
	set2matf(*O33,3,3,*sins.Mav_data,&sins.Mav);
	set2matf(*O33,3,3,*sins.Map_data,&sins.Map);
	set2matf(*O33,3,3,*sins.Mva_data,&sins.Mva);
	set2matf(*O33,3,3,*sins.Mvv_data,&sins.Mvv);
	set2matf(*O33,3,3,*sins.Mvp_data,&sins.Mvp);
	set2matf(*O33,3,3,*sins.Mpp_data,&sins.Mpv);
	set2matf(*O33,3,3,*sins.Mpv_data,&sins.Mpp);

	set2vecf(O31,3,sins.fn_data,&sins.fn);
	set2vecf(O31,3,sins.fb_data,&sins.fb);

	set2matf(*O33,3,3,*sins.Cnb_data,&sins.Cnb);
	set2matf(*O33,3,3,*sins.Cbn_data,&sins.Cbn);
}
void SINS_etm(){
#ifdef PSINS_LOW_GRADE_MEMS
	//低精度惯导只需考虑Mva和Mpv
	askew(sins.fn_data,&sins.Mva);
	sins.Mpv_data[0][1] = 1/111316.666f;	///eth.RMh;
	sins.Mpv_data[1][0] = 1/(111316.666f*eth.cosL);	///(eth.cosL*eth.RNh);
	sins.Mpv_data[2][2] = 1;
#else
	//...
#endif
}

//构造惯导状态转移矩阵F(t)
void KF_SetFt(){
	SINS_etm();
//	Ft = [ Maa    Mav    Map    -Cnb	O33 
//         Mva    Mvv    Mvp     O33    Cnb 
//         O33    Mpv    Mpp     O33    O33
//			         zeros(6,15)  			];
	mat3f temp;
	op_mat(*sins.Cbn_data, *temp, 3, 3);
	setMat3(sins.Maa_data,&kf.Ft,0,0);
	setMat3(sins.Mav_data,&kf.Ft,0,3);
	setMat3(sins.Map_data,&kf.Ft,0,6);
	setMat3(temp,&kf.Ft,0,9);
	setMat3(O33,&kf.Ft,0,12);
	
	setMat3(sins.Mva_data,&kf.Ft,3,0);
	setMat3(sins.Mvv_data,&kf.Ft,3,3);
	setMat3(sins.Mvp_data,&kf.Ft,3,6);
	setMat3(O33,&kf.Ft,3,9);
	setMat3(sins.Cbn_data,&kf.Ft,3,12);
	
	setMat3(O33,&kf.Ft,6,0);
	setMat3(sins.Mpv_data,&kf.Ft,6,3);
	setMat3(sins.Mpp_data,&kf.Ft,6,6);
	setMat3(O33,&kf.Ft,6,9);
	setMat3(O33,&kf.Ft,6,12);
}

void KF_SetHt(){
	//	Ht = [ O63	  I66	O66	]; 
	set2matf(*O6_15, 6, 15, *kf.Hk_data, &kf.Hk);
	setMat3(O33,&kf.Hk,0,0);
	setMat3(I33,&kf.Hk,0,3);
	setMat3(O33,&kf.Hk,0,6);
	setMat3(O33,&kf.Hk,0,9);
	setMat3(O33,&kf.Hk,0,12);
	setMat3(O33,&kf.Hk,3,0);
	setMat3(O33,&kf.Hk,3,3);
	setMat3(I33,&kf.Hk,3,6);
	setMat3(O33,&kf.Hk,3,9);
	setMat3(O33,&kf.Hk,3,12);		
	//HT
	set2matf(*O15_6, 15, 6, *kf.HT_data, &kf.HT);
	arm_mat_trans_f32(&kf.Hk, &kf.HT);
//	print_mat_name(kf.Hk,"Hk");
//	print_mat_name(kf.HT,"HkT");
}

void KF_SetMeas_flag(){
	kf.measflag = false;
	if(vecf_isZero(kf.measGPSPos,3) || vecf_isZero(kf.measGPSVn,3)) return;
//	arm_scale_f32(kf.measGPSPos, glv.deg, kf.measGPSPos, 3);//角度->弧度
	arm_sub_f32(sins.vn, kf.measGPSVn, kf.measGPSVn, 3);
//	arm_sub_f32(sins.pos, kf.measGPSPos, kf.measGPSPos, 3);
	for(int i=0;i<3;++i) kf.measGPSPos[i]=sins.pos[i]-kf.measGPSPos[i];
	arm_copy_f32(kf.measGPSVn, kf.Zk_data, 3);
	arm_copy_f32(kf.measGPSPos, kf.Zk_data+3, 3);
	//测量值清空
	arm_copy_f32(O31, kf.measGPSVn, 3);
	arm_copy_f32(O31, kf.measGPSPos, 3);
	kf.measflag = true;
}
/**
  * @brief  KF设置测量量，由外部调用
  * @note   
  * @param  vnm: 速度.
  * @param  posm: 位置(弧度)
  * @param  wb: 加速度
  * @retval None
  */
void KF_SetMeas(vec3f vnm, vec3f posm, float tm){
	arm_copy_f32(vnm, kf.measGPSVn, 3);
	arm_copy_f32(posm, kf.measGPSPos, 3); 
	kf.tmeas = tm;
}
	


/**
  * @brief  卡尔曼滤波器初始化	
  * @note   
  * @param  vn: GPS速度(m/s)
  * @param  posn: GPS位置(度)
  * @retval None
  */
void KF_init(vec3f vn, vec3f posn, float tk){
// 状态量：phi(3), dvn(3), dpos(3), eb(3), db(3)
	
	kf.tdts = 0.0;kf.nq = 15; kf.nr=6;
	GLV_init();
	SINS_init(vn, posn, tk);
/*测量量*/
	kf.measflag = false;
	arm_copy_f32(O31, kf.measGPSVn, 3);
	arm_copy_f32(O31, kf.measGPSPos, 3);
/*估计协方差初始化*/
	vec15f Pk =	{10.0f*glv.deg, 10.0f*glv.deg, 30.0f*glv.deg, 
				 1.0, 1.0, 1.0, 
				 100.0f/glv.Re, 100.0f/glv.Re, 100.0, 
				 1000.0f*glv.dph, 1000.0f*glv.dph, 1000.0f*glv.dph, 
				 10.0f*glv.mg,10.0f*glv.mg,10.0f*glv.mg};
	set2diag_sq(Pk,15,*kf.Pk_data,&kf.Pk);
//	set2matf(*O15_15, 15, 15, *kf.Pk_data, &kf.Pk);
	//最大最小限制
	vec15f Pmax= {10.0f*glv.deg,10.0f*glv.deg,30.0f*glv.deg, 50.0,50.0,50.0, 1.0e4f/glv.Re,1.0e4f/glv.Re,1.0e4, 
		1000.0f*glv.dph,1000.0f*glv.dph,1000.0f*glv.dph, 100.0f*glv.mg,100.0f*glv.mg,100.0f*glv.mg};
	arm_mult_f32(Pmax, Pmax, kf.Pmax, 15);
	vec15f Pmin = {
			1.0f*glv.min, 1.0f*glv.min, 1.0f*glv.min, 
			0.01, 0.01, 0.1, 
			1.0f/glv.Re, 1.0f/glv.Re, 1.0, 
			1.0f*glv.dph, 1.0f*glv.dph, 1.0f*glv.dph, 
			0.1f*glv.mg, 0.1f*glv.mg, 0.1f*glv.mg};
	arm_mult_f32(Pmin, Pmin, kf.Pmin, 15);
/*预测协方差*/
	set2matf(*O15_15, 15, 15, *kf.Pk1_data, &kf.Pk1);
/*系统噪声*/
	vec15f Qt = {
			1.0f*glv.dpsh, 1.0f*glv.dpsh, 1.0f*glv.dpsh, 
			100.0f*glv.ugpsHz, 100.0f*glv.ugpsHz, 100.0f*glv.ugpsHz, 
			0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0};
	set2vecf_sq(Qt,15,kf.Qt_data,&kf.Qt);
//	set2vecf(O15_1,15,kf.Qt_data,&kf.Qt);
/*观测量*/
	set2vecf(O61,6,kf.Zk_data,&kf.Zk);
/*量测噪声*/
	vec6f Rt = {0.5, 0.5, 0.5, 10.0f/glv.Re, 10.0f/glv.Re, 10.0};
	set2diag_sq(Rt,6,*kf.Rt_data,&kf.Rt);
	//最大最小限制
	set2vecf_scale(Rt, 100, 6, kf.Rmax_data, &kf.Rmax);
	set2vecf_scale(Rt, 0.01, 6, kf.Rmin_data, &kf.Rmin);
		
	vec6f Rb = {0.9,0.9,0.9,  0.9,0.9,0.9};
	set2vecf(Rb,6,kf.Rb_data,&kf.Rb);
/*反馈*/
	vec15f FBTau = {1.0,1.0,10.0,  1.0,1.0,1.0,  1.0,1.0,1.0,  10.0,10.0,10.0,  10.0,10.0,1.0};
	arm_copy_f32(FBTau, kf.FBTau, 15);	
	vec15f FBMax = {INF,INF,INF,  INF,INF,INF,  INF,INF,INF,  5000.0f*glv.dph,5000.0f*glv.dph,5000.0f*glv.dph,  50.0f*glv.mg,50.0f*glv.mg,50.0f*glv.mg}; 
	arm_copy_f32(FBMax, kf.FBMax, 15);	
	arm_copy_f32(O15_1, kf.FBTotal, 15);	
/*状态转移矩阵*/
	set2matf(*O15_15, 15, 15, *kf.Ft_data, &kf.Ft);
	set2matf(*O15_15, 15, 15, *kf.FtT_data, &kf.FtT);
/*状态量*/
	set2vecf(O15_1,15,kf.Xk_data,&kf.Xk);
	set2vecf(O15_1,15,kf.Dx_data,&kf.Dx);
	vec15f Xmax={INF,INF,INF,INF,INF,INF,INF,INF,INF,INF,INF,INF,INF,INF,INF};
	arm_copy_f32(Xmax,kf.Xmax,15);
/*观测阵*/
	KF_SetHt();
/*卡尔曼增益*/
	set2matf(*O15_6, 15, 6, *kf.Kk_data, &kf.Kk);
/*中间矩阵*/
	set2matf(*O15_15, 15, 15, *kf.KHP_data, &kf.KHP);
	set2matf(*O15_6, 15, 6, *kf.PHT_data, &kf.PHT);
	set2matf(*O6_15, 6, 15, *kf.HP_data, &kf.HP);
	set2matf(*O66, 6, 6, *kf.Sk_data, &kf.Sk);
	set2matf(*O66, 6, 6, *kf.Sk_inv_data, &kf.Sk_inv);
	set2vecf(O61,6,kf.Y_data,&kf.Y);
	set2matf(O15_1, 15, 1, kf.Xk1_data, &kf.Xk1);
	set2matf(*O15_15, 15, 1, *kf.FtT_data, &kf.FtT);


	kf.KFinit_complete = true;
}
//余弦矩阵更新
void DCM_update(vec3f att, mat3f vec, arm_matrix_instance_f32 *R){
	float SinR,CosR,SinP,CosP,SinY,CosY;
	arm_sin_cos_f32(att[0], &SinR, &CosR);//phi
	arm_sin_cos_f32(att[1], &SinP, &CosP);//theta
	//把航向角转化成[-180,180],注意不能用原角度
	float yaw = att[2];
	if(yaw>180) yaw -= 360;
	arm_sin_cos_f32(yaw, &SinY, &CosY);//psi
	//北东地坐标系
//	float M[]={	CosY*CosP,	-SinY*CosR+CosY*SinP*SinR,	SinY*SinR+CosY*CosR*SinP,
//				SinY*CosP,	CosY*CosR+SinR*SinP*SinY,	-CosY*SinR+SinP*SinY*CosR,
//				-SinP,		CosP*SinR,					CosP*CosR				};
	//东北天坐标系
	float M[]={	CosY*CosR+SinR*SinP*SinY,	SinY*CosP,		CosY*SinR-SinY*SinP*CosR,
				-SinY*CosR+CosY*SinP*SinR,	CosY*CosP,		-SinY*SinR-CosY*SinP*CosR,
				-CosP*SinR,					SinP,			CosP*CosR		};
	arm_copy_f32(M, *vec, 9);
	arm_mat_init_f32(R ,3, 3, *vec);
}

void Dead_Reckoning(){
	arm_sub_f32(sins.fn.pData, eth.gn, sins.an, 3);	//得到实际加速度
//	sins.an[2] *= -1;	//向下加速是失重，加速度<g
	vec3f dv,vn_latest,v,dp,an;
	//加速度在一定范围内认为是静止状态 -> 去除零漂
	an[0] = fabs(sins.an[0])<fp.sins_db[0]?0:sins.an[0];
	an[1] = fabs(sins.an[1])<fp.sins_db[1]?0:sins.an[1];
	arm_scale_f32(an, sins.nts, dv, 3);//单位时间速度增量
	arm_add_f32(sins.vn, dv, vn_latest, 3);
	arm_add_f32(vn_latest, sins.vn, v, 3);
	arm_scale_f32(v, sins.ts/2, dp, 3);//单位时间位移增量为 (上一时刻速度+当前时刻速度)*dt/2
	arm_add_f32(dp, sins.dn, sins.dn, 3);
	//TODO 为什么除的数值相差这么大？
//	dp[0] = dp[0]/eth.RMh; dp[1] = dp[1]/(eth.cosL*eth.RNh);dp[2] = 0;//(3.1.29~3.1.31) 
	sins.pos[0] += (dp[1]*M_TO_LATLON/1e7f);//纬度
	sins.pos[1] += (dp[0]*M_TO_LATLON/eth.cosL/1e7f);	//经度（距离转度）
	sins.pos[2] += dp[2];
	arm_copy_f32(vn_latest, sins.vn, 3);//速度更新
//	printf("%f %f %f %f \r\n",pos[0]*1e7f,pos[1]*1e7f,sins.pos[0]*1e7f,sins.pos[1]*1e7f);

}
//惯导更新
void SINS_update(vec3f att, vec3f wb, vec3f ab, int nSamples, double ts){
//	vec3f eb = {0.9,0.6,-0.8};
	arm_sub_f32(wb, sins.eb, sins.wn, 3);	//角速度减去陀螺零偏 deg/s ??
	arm_sub_f32(ab, sins.db, sins.fb.pData, 3);
	sins.ts = ts; sins.nts = nSamples*ts; sins.tk += sins.nts;
	float nts2 = sins.nts/2;
#ifdef PSINS_LOW_GRADE_MEMS	
//	EARTH_update(sins.pos, sins.vn);//地球参数更新
	DCM_update(att, sins.Cbn_data, &sins.Cbn);//构建余弦矩阵
	arm_mat_mult_f32(&sins.Cbn,&sins.fb,&sins.fn);	//fn=Cbn*fb
	Dead_Reckoning();//航位推算
#else

#endif
}
/**
  * @brief  部分反馈修正
  * @note   程序中的afa=0表示不反馈,afa=1表示全反馈,0<afa<1表示部分反馈。
  			部分反馈的好处之一是可以使导航解算值更加平滑。
  			例如设置afa=1，可能就会出现锯齿状的形状，afa越小，越平缓，但部分反馈可能比全反馈少许有偏。
  * @param  fbts: 卡尔曼滤波的反馈间隔
  * @retval None
  */

void KF_feedback(float fbts){
	float *pTau=kf.FBTau, *pTotal=kf.FBTotal, *pMax=kf.FBMax, *pXk=kf.FBXk, *p=kf.Xk_data;
	for(int i=0; i<kf.nq; i++, pTau++,pTotal++,pMax++,pXk++,p++){
		if(*pTau<INF/2)
		{
//			double afa = fbts<*pTau ? fbts/(*pTau) : 1.0f;
			double afa = 1.0f;	//全反馈
			*pXk = (*p)*afa;
			if(*pMax<INF/2)
			{
				if(*pTotal+*pXk>*pMax)			*pXk = *pMax-*pTotal;
				else if(*pTotal+*pXk<-*pMax)	*pXk = -*pMax-*pTotal;
			}
			*p -= *pXk;	//卡尔曼滤波的状态估计值“部分”(x*afa)并入解算值，使解算值接近真值，而剩余部分(x*(1-afa))继续留在卡尔曼滤波器中
			*pTotal += *pXk;
		}
		else
		{
			*pXk = 0.0;
		}
	}
//	arm_sub_f32(sins.qnb, kf.FBXk, sins.qnb, 3);
	arm_sub_f32(sins.vn, kf.FBXk+3, sins.vn, 2);
	for(int i=0;i<3;i++) sins.pos[i]-= *(kf.FBXk+6+i);
//	arm_sub_f32(sins.pos, kf.FBXk+6, sins.pos, 2);
//	arm_add_f32(sins.eb, kf.FBXk+9, sins.eb, 3);
//	arm_add_f32(sins.db, kf.FBXk+12, sins.db, 3);
}
//状态估计、方差阵限制（使-Xmax<=Xk<=Xmax、Pmin<=diag(Pk)<=Pmax）
void XPConstrain(){
	int nq1=kf.nq+1;
	float *px=kf.Xk_data,*pxmax=kf.Xmax,*p=*kf.Pk_data,*pmin=kf.Pmin,*pminEnd=kf.Pmin+kf.nq,*pmax=kf.Pmax;
//	print_mat_name(kf.Xk, "Xk_before_Constrain");
//	print_mat_name(kf.Pk, "Pk_before_Constrain");
	for(int i=0; pmin<pminEnd; ++i){
		if(*px>*pxmax)	*px = *pxmax;	
		else if(*px<-*pxmax)	*px = -*pxmax;	// Xk constrain
		
		if(*p<*pmin)	*p = *pmin;	// Pk constrain
		else if(*p>*pmax){
			float sqf=sqrt((*pmax)/(*p))*0.95;			
			float *prow=*kf.Pk_data+i*kf.nq;//第i行行首
			float *prowEnd=prow+kf.nq;//第i行行尾
			float *pclm=*kf.Pk_data+i;//第i列列首
			for(;prow<prowEnd; prow++,pclm+=kf.nq){
				*prow *= sqf;
				*pclm *= sqf;
			}
		}
		px++;pxmax++;//到Xk_data下一个状态
		p+=nq1;//到Pk_data下一个对角元素
		pmin++;pmax++;
	}
//	print_mat_name(kf.Xk, "Xk_after_Constrain");
//	print_mat_name(kf.Pk, "Pk_after_Constrain");
}

/**
  * @brief  卡尔曼滤波更新
  * @note   
  * @param  att: 角度（单位:°）
  * @param  wb: 角速度
  * @param  wb: 加速度
  * @retval None
  */
void KF_update(vec3f att, vec3f wb, vec3f ab,int nSamples, double ts, int nStep){
	if(!kf.KFinit_complete) return;
	SINS_update(att, wb, ab, nSamples, ts);

	kf.tdts = sins.nts;
	kf.kftk = sins.tk;

	KF_SetFt();
	KF_SetMeas_flag();	//得到Zk
	/*状态转移矩阵离散化 https://blog.csdn.net/l2014010671/article/details/91126676*/
	// Fk = I+Ft*ts
	arm_mat_scale_f32(&kf.Ft,kf.tdts,&kf.Ft);
	add_eye2mat(&kf.Ft);
	// Qk = Qt*ts
	arm_mat_scale_f32(&kf.Qt, kf.tdts, &kf.Qt);
/*状态预测*/
	// Xk = Fk*Xk
	arm_mat_mult_f32(&kf.Ft, &kf.Xk, &kf.Xk1);
	arm_copy_f32(kf.Xk1.pData, kf.Xk.pData, 15);
	// PK1 = Fk*Pk*(Fk)^T + Qk
	arm_mat_mult_f32(&kf.Ft, &kf.Pk, &kf.Pk1);
	arm_mat_trans_f32(&kf.Ft, &kf.FtT);
	
	arm_mat_mult_f32(&kf.Pk1, &kf.FtT, &kf.Pk);//没有进行量测更新也需更新预测协方差
	arm_mat_add_f32(&kf.Pk, &kf.Qt, &kf.Pk);
	
	symmetry(*kf.Pk_data,15);
	
//	print_mat_name(kf.Pk,"pk");
/*量测更新*/
	if(kf.measflag){
		/*增益计算*/
		//PHT
		arm_mat_mult_f32(&kf.Pk,&kf.HT, &kf.PHT);
		//S=HPH'+R
		arm_mat_mult_f32(&kf.Hk, &kf.PHT, &kf.Sk);
		arm_mat_add_f32(&kf.Sk, &kf.Rt, &kf.Sk);	//加入量测噪声	
		//K=PHT/Sk
		int res = arm_mat_inverse_f32(&kf.Sk, &kf.Sk_inv);
//		printf("res=%d\r\n",res);
		arm_mat_mult_f32(&kf.PHT,&kf.Sk_inv, &kf.Kk);
//		print_mat_name(kf.Kk,"Kk");
		/*状态更新*/
		arm_mat_mult_f32(&kf.Hk,&kf.Xk, &kf.Y);
		arm_mat_sub_f32(&kf.Zk, &kf.Y, &kf.Y);
		arm_mat_mult_f32(&kf.Kk, &kf.Y, &kf.Dx);
//		print_mat_name(kf.Kk, "Kk");
//		arm_mat_mult_f32(&kf.HT, &kf.Y, &kf.Dx);
		arm_mat_add_f32(&kf.Xk, &kf.Dx, &kf.Xk);

		/*估计协方差更新*/
		arm_mat_mult_f32(&kf.Hk,&kf.Pk, &kf.HP);
		arm_mat_mult_f32(&kf.Kk,&kf.HP, &kf.KHP);
		arm_mat_sub_f32(&kf.Pk, &kf.KHP, &kf.Pk);
	}
	//方差限制与对称化
	XPConstrain();
	
//	print_mat_name(kf.Pk, "Pk_symmetry");

	KF_feedback(sins.nts);
}



