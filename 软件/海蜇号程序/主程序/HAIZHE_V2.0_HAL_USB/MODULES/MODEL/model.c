#include "model.h"
#include "sensor.h"

MODEL model;


//���pwm-��������
float input_pwm_to_force(int type, u16 pwm){	
	float force=0;
	float p = (float)pwm/1000.0f;
	
	if(p>=1.0f && p<1.5f){
		switch(type){
			case CW:force = (26.81f*p*p-109.04f*p+102.89f)*1;	break;	//������Ϊ������
			case CCW:force = (473.38f*p*p*p-1730.0f*p*p+2017.8f*p-729.25f)*1;break;
		}
	}
	else if(p>1.5f && p<=2.0f){
		switch(type){
			case CW:force = (46.358f*p*p-122.73f*p+79.529f)*-1; break;	//�����Ϸ�����
			case CCW:force = (34.4f*p*p-77.255f*p+38.072f)*-1;	break;
		}
	}
	else if(is_equal(p, 1.5)) force=0;

	return force;
}
/**
  * @brief  ����ѧģ�ͳ�ʼ��
  * @note   
  * @param  att: �Ƕȣ���λ:�㣩
  * @param  vn: �ٶ�
  * @param  pos: ��γ��
  * @retval None
  */

void model_init(vec3f att, vec3f vn, vec3f pos, float tk){
	model.dt = tk;
	arm_copy_f32(att, model.att, 3);
	arm_copy_f32(vn, model.vn, 3);
	arm_copy_f32(pos, model.pos, 3);
//	model.pos[0] *= 1e7f;
//	model.pos[1] *= 1e7f;	//��γ�ȷŴ�1e7
	
	//�������
	u8 Moter_Dir[6] = {CCW,CW,CCW,CW,CW,CCW};
	memcpy(model.Moter_Dir, Moter_Dir, 6);
	//������� -> �ɲ����õ�
	model.m = 3.75f;
	model.G = model.m * GRAVITY_MSS;
	model.B = (model.m+0.995f) * GRAVITY_MSS;
	model.l = 0.21062;
	model.R = 0.0975;//90+15/2mm
	//��������
	model.Zg[0] = 0; model.Zg[1] = 0; model.Zg[2] = 11.15f*1e-3f;
	//ת������
	model.Ix = 2*model.m*model.R*model.R/5;
	model.Iy = model.Ix;
	model.Iz = model.Ix;
	//�������嶯���� -> ��CCM����õ�
	model.hf.Xu = 0.7318;
	model.hf.Yv = 0.7318;
	model.hf.Zw = 0.3747;
	model.hf.Kp = 0.001359;
	model.hf.Mq = 0.001359;
	model.hf.Nr = 0.006736;
	//��������
	model.hf.Xdu = 1.9897;
	model.hf.Ydv = 1.9897;
	model.hf.Zdw = 3.1331;
	model.hf.Kdp = 0.0058;
	model.hf.Mdq = 0.0058;
	model.hf.Ndr = 0.0037;
	//�������嶯����
	model.hf.Xddu = 3.006;
	model.hf.Yddv = 3.006;
	model.hf.Zddw = 36.35;
	model.hf.Kddp = 0.000012;
	model.hf.Mddq = 0.000012;
	model.hf.Nddr = 0.008206;

//ϵͳ���̾����ʼ��
	//��������
	vec6f MRB={model.m, model.m, model.m, model.Ix, model.Iy, model.Iz};
	vec6f MA={model.hf.Xdu, model.hf.Ydv, model.hf.Zdw, model.hf.Kdp, model.hf.Mdq, model.hf.Ndr};
	arm_add_f32(MRB, MA, MRB, 6);
	set2diag(MRB, 6, *model.M_data, &model.M);
	arm_mat_init_f32(&model.M, 6, 6, *model.M_data);
	model.M_data[0][4] = model.M_data[4][0] = model.m*model.Zg[2];
	model.M_data[1][3] = model.M_data[3][1] = -model.m*model.Zg[2];
	arm_mat_init_f32(&model.M_inv, 6, 6, *model.M_inv_data);
	arm_mat_inverse_f32(&model.M, &model.M_inv); //������������
	//��������
	vec6f D_data = {model.hf.Xu, model.hf.Yv, model.hf.Zw, model.hf.Kp, model.hf.Mq, model.hf.Nr};
	set2diag(D_data, 6, *model.D_data, &model.D);	//������
//	set2diag(O61, 6, *model.D_data, &model.D);	//������
	//-inv(M)*D
	arm_mat_init_f32(&model.MinvD, 6, 6, *model.MinvD_data);
	arm_mat_mult_f32(&model.M_inv, &model.D, &model.MinvD);
	arm_mat_scale_f32(&model.MinvD, -1, &model.MinvD);
	//�˶�ѧ����
	arm_mat_init_f32(&model.J, 6, 6, *model.J_data);
	float Cphi,Sphi;
	arm_sin_cos_f32(model.att[2], &Sphi, &Cphi);
	mat3f Rz={{Cphi,-Sphi,0},{Sphi,Cphi,0},{0,0,1}};
	setMat(*Rz, 3, 3, &model.J, 0, 0);
	setMat(*O33, 3, 3, &model.J, 0, 3);
	setMat(*O33, 3, 3, &model.J, 3, 0);
	setMat(*I33, 3, 3, &model.J, 3, 3);
	//״̬ת�ƾ���A
	/*A=[0		J
	     0  	-inv(M)*D] */
	set2matf(*O12_12, 12, 12, *model.A_data, &model.A);
	setMat(*O66, 6, 6, &model.A, 0, 0);
	setMat2Mat(&model.J, &model.A, 0, 6);
	setMat(*O66, 6, 6, &model.A, 6, 0);
	setMat2Mat(&model.MinvD, &model.A, 6, 6);	
	//A��ɢ��
	arm_scale_f32(*model.A_data, model.dt, *model.A_data, 12*12);
	add_eye2mat(&model.A);
	//�����������
	float l = model.l;
	float t = sqrt(2)*l/2;
	/*ע�⣺������������еķ��ź�parameter.c�е�motor_fac��һ�����룬��Ϊ������������������ǿ�����*/
	mat6f T={	
			{0,	 0,	0,	0,	1,	1},
			{0,	 0,	0,	0,	0,	0},
			{1,	 1,	1,	1,	0,	0},
			{t, -t,	-t,	t,	0,	0},
			{-t,-t,	t,	t,	0,	0},
			{0,	0,	0,	0,	l,	-l}};
	set2matf(*T, 6, 6, *model.T_data, &model.T);
	//����ת�ƾ���H
	arm_mat_init_f32(&model.MinvT, 6, 6, *model.MinvT_data);
	arm_mat_mult_f32(&model.M_inv, &model.T, &model.MinvT);	//inv(M)*T
	set2matf(*O12_6, 12, 6, *model.H_data, &model.H);
	setMat(*O66, 6, 6, &model.H, 0, 0);
	setMat2Mat(&model.MinvT, &model.H, 6, 0);
	//H��ɢ��
	arm_scale_f32(*model.H_data, model.dt, *model.H_data, 12*6);
	//�ָ����غ�����ת�ƾ���E
	arm_mat_init_f32(&model.E, 12, 6, *model.E_data);
	setMat(*O66, 6, 6, &model.E, 0, 0);
	setMat2Mat(&model.M_inv, &model.E, 6, 0);
	//E��ɢ��
	arm_scale_f32(*model.E_data, model.dt, *model.E_data, 12*6);
	//״̬��
	//X=[u,v,w,q,p,r,x,y,z,phi,theta,pshi];
	vec12f x={0,0,0, att[0],att[1],att[2], 0,0,0, 0,0,0};
	set2vecf(x, 12, model.X_data, &model.X);
	//������
	set2vecf(O61, 6, model.U_data, &model.U);
	//����
	set2vecf(O61, 6, model.W_data, &model.W);
	//�м����
	arm_mat_init_f32(&model.X1, 12, 1, model.X1_data);
	arm_mat_init_f32(&model.X2, 12, 1, model.X2_data);
	arm_mat_init_f32(&model.X3, 12, 1, model.X3_data);
//	print_mat_name(model.A, "A");
//	print_mat_name(model.H, "H");
//	print_mat_name(model.E, "E");
//	print_mat_name(model.X, "X");
//	print_mat_name(model.U, "U");
//	print_mat_name(model.W, "W");
	model.model_init_complete = true;
	printf("[OK] model init complete! \r\n");
}

void model_updata_core(vec3f att, vec6f force){
	//��������
	arm_copy_f32(force, model.U_data, 6);
	//�������Ÿ���
	model.W_data[2] = model.G-model.B;
	//����״̬ת�ƾ���A
	float Cphi,Sphi;
	arm_sin_cos_f32(att[2], &Sphi, &Cphi);
	mat3f Rz={{Cphi,-Sphi,0},{Sphi,Cphi,0},{0,0,1}};
	setMat(*Rz, 3, 3, &model.J, 0, 0);
	arm_scale_f32(*model.J_data, model.dt, *model.J_data, 12*6);	//J��ɢ��
	setMat2Mat(&model.J, &model.A, 0, 6);
	//��¼��һʱ���ٶȺ�λ��
	vec3f vb, pos;
	arm_copy_f32(model.X_data+6, vb, 3);
	arm_copy_f32(model.X_data, pos, 3);
	//״̬����
	arm_mat_mult_f32(&model.A, &model.X, &model.X1);
	arm_mat_mult_f32(&model.H, &model.U, &model.X2);
	arm_mat_mult_f32(&model.E, &model.W, &model.X3);
	arm_copy_f32(O12_1, model.X_data, 12);	//ע�⣺�˴���Ҫ����գ�����
	arm_mat_add_f32(&model.X,  &model.X1,  &model.X);
	arm_mat_add_f32(&model.X,  &model.X2,  &model.X);
	arm_mat_add_f32(&model.X,  &model.X3,  &model.X);
	//������ϵ���ٶȸ���
	arm_sub_f32(model.X_data+6, vb, model.ab, 3);
	arm_scale_f32(model.ab, 1/model.dt, model.ab, 3);
	//����޷������>=0 -> ֻ������ˮ�棬����������
	model.pos[2] = constrain_float(model.pos[2], 0, 1000);
	if(is_zero(model.pos[2]) && (model.X_data[2]-pos[2])<0) {	//���Ϊ0����Ȼ����
		model.X_data[8]=0;	
		model.ab[2]= 0;
	}
	//��������ϵ�ٶȸ��� d=d1-d0=(v1+v0)*T/2
	model.vn[0] = (model.X_data[0]-pos[0])*2/model.dt-model.vn[0];
	model.vn[1] = (model.X_data[1]-pos[1])*2/model.dt-model.vn[1];
	model.vn[2] = (model.X_data[2]-pos[2])*2/model.dt-model.vn[2];
	//��γ��λ�ø���
	model.pos[0] += (model.X_data[0]-pos[0])/111316.666f;
	model.pos[1] += (model.X_data[1]-pos[1])/111316.666f/cosf(model.pos[0]* DEG_TO_RAD);
	model.pos[2] += (model.X_data[2]-pos[2]);
}

void model_update(vec3f att){
	if(!model.model_init_complete) return;
	//pwm-����
	u16 *pwm = get_motorout();
	vec6f force;
	for(int i=0;i<6;i++){
		model.motor_input[i] = *pwm;	//��������
		force[i] = input_pwm_to_force(model.Moter_Dir[i], *pwm);	
		pwm++;
	}
	model_updata_core(att, force);
}

