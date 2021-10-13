#include "kf_test.h"
#include "kf.h"
#include "malloc.h"
#include "sensor.h"
#include "parameter.h"

/*卡尔曼滤波测试-对应python绘图脚本kf_test.py*/


//卡尔曼滤波测试
void Test_print_info(int num){
	printf("%.2f ",(float)num/100.0f);
	printf("%.5f %.5f %.5f %.5f ",sins.an[0],sins.vn[0],sins.dn[0],sins.pos[0]);
	printf("%.5f %.5f %.5f %.5f ",sins.an[1],sins.vn[1],sins.dn[1],sins.pos[1]);
//	printf("%.5f %.5f %.5f %.5f ",sins.an[2],sins.vn[2],sins.dn[2],sins.pos[2]);
//	printf("%.5f %.5f %.5f %f ",kf.Zk_data[0],kf.Zk_data[1],kf.Zk_data[3]*eth.RMh,kf.Zk_data[4]*eth.RMh);
	printf("\r\n");
}
//打印预测数据
void Test_print_Predict(int sec, vec3f *vn, vec3f *pos){
	printf("0 0 0 0 0 0 0 \r\n");
	for(int i=0;i<sec;i++){
		printf("%d ",i+1);
		printf("%.5f %.5f %.5f ",(*(vn+i))[0],(*(vn+i))[1],(*(vn+i))[2]);
		printf("%.5f %.5f %.5f ",(*(pos+i))[0],(*(pos+i))[1],(*(pos+i))[2]);
		printf("\r\n");
	}
}

void Test_Reference(vec3f *att, vec3f *wb, vec3f *ab, int time){
	KF_init(O31, O31, 0.01);
	for(int i=0;i<time;i++){
		KF_update(*(att+i), *(wb+i), *(ab+i), 1, 0.01, 1);
		Test_print_info(i);
	}
	kf.KFinit_complete = false;
}

void Test_DR_NO_KF(vec3f *att, vec3f *wb, vec3f *ab, int time,float* err){
	KF_init(O31, O31, 0.01);
	for(int i=0;i<time;i++){
		vec3f _ab={(*(ab+i))[0]+*(err+i), (*(ab+i))[1]+*(err+i), (*(ab+i))[2]};//x单方向
		KF_update(*(att+i), *(wb+i), _ab, 1, 0.01, 1);
		Test_print_info(i);
	}
	kf.KFinit_complete = false;
}
void Test_DR_With_KF(vec3f *att, vec3f *wb, vec3f *ab, vec3f *vn, vec3f *pos, int time,float *err){
	KF_init(O31, O31, 0.01);
	for(int i=0;i<time;i++){
		vec3f _ab={(*(ab+i))[0]+*(err+i), (*(ab+i))[1]+*(err+i), (*(ab+i))[2]};
		if(i%100==0 && i>0) {
			static int sec=0;
			KF_SetMeas(*(vn+sec), *(pos+sec), 1);	//设置观测量完全准确
			sec++;
		}
		KF_update(*(att+i), *(wb+i), _ab, 1, 0.01, 1);
//		Test_print_info(i);
	}
	kf.KFinit_complete = false;

}
void test(vec3f *att, vec3f *wb, vec3f *ab, vec3f *vn, vec3f *pos, int num, float* err){
//	Test_Reference(att, wb, ab, num);
//	Test_DR_NO_KF(att, wb, ab, num,err);
	Test_DR_With_KF(att, wb, ab, vn, pos, num, err);
}

void kF_test(int sec, int type){
	KF_init(O31, O31, 0.01);
	//注意：在给att等成员变量赋值取值时需要使用(*(att+i))[0]，而不能使用*(att+i)[0]！！！
	int num=sec*100;
	float *err = mymalloc(SRAMIN, num*4);
	vec3f *att = mymalloc(SRAMIN, num*12);	//姿态
	vec3f *wb = mymalloc(SRAMIN, num*12);	//角速度
	vec3f *ab = mymalloc(SRAMIN, num*12);	//加速度
	vec3f *vn = mymalloc(SRAMIN, sec*12);	//观测量，速度设置为1秒一次
	vec3f *pos = mymalloc(SRAMIN, sec*12);	//观测量，位置设置为1秒一次
//	eth.RMh = 1;eth.cosL= 1;eth.RNh = 1;
	int i=0;
	for(i=0;i<num;i++)	(*(wb+i))[0]=(*(wb+i))[1]=(*(wb+i))[2]=0;
	switch(type){
		case KF_Line://直线运动
			for(i=0;i<num;i++)	*(err+i)=rand()%100/100.0f;//产生0~1随机数-0.5f
			for(i=0;i<num/2;i++)	{(*(ab+i))[0]=1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}	//只向x轴运动,前半段加速，后半段减速
			for(i=num/2;i<num;i++)	{(*(ab+i))[0]=-1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}
			for(i=0;i<num;i++)	(*(att+i))[0]=(*(att+i))[1]=(*(att+i))[2]=0;
//			printf("%d %.2f \r\n",i,(*(att+i))[2]);
			for(i=1;i<=sec;i++)	{
				(*(vn+i-1))[0]=i;(*(vn+i-1))[1]=0;(*(vn+i-1))[2]=0;	//v=a(*t
				(*(pos+i-1))[0]=0.5*i*i/eth.RMh;(*(pos+i-1))[1]=0;(*(pos+i-1))[2]=0; //p=a*t*t/2
			}
			break;
		 case KF_PolyLine: //折线运动
			for(i=0;i<num;i++)	*(err+i)=(rand()%100/100.0f-0.5f)/1.0f;
		 	for(i=0;i<num/4;i++)	{(*(ab+i))[0]=1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}	//只向x轴运动,前半段加速，后半段减速
			for(i=num/4;i<num/2;i++)	{(*(ab+i))[0]=-1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}
			for(i=num/2;i<num*3/4;i++)	{(*(ab+i))[0]=1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}	
			for(i=num*3/4;i<num;i++)	{(*(ab+i))[0]=-1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}
			
			for(i=0;i<num/2;i++)	{(*(att+i))[0]=(*(att+i))[1]=(*(att+i))[2]=0;}
			for(i=num/2;i<num;i++)	{(*(att+i))[0]=(*(att+i))[1]=0;(*(att+i))[2]=90;}//转向y轴
	
			for(i=1;i<=sec/4;i++)	{
				(*(vn+i-1))[0]=i;(*(vn+i-1))[1]=0;(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=0.5*i*i/eth.RMh;(*(pos+i-1))[1]=0;(*(pos+i-1))[2]=0; 
			}
			for(i=sec/4;i<=sec/2;i++)	{
				(*(vn+i-1))[0]=sec/4-(i-sec/4);(*(vn+i-1))[1]=0;(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=(0.5*(sec/4)*(sec/4)+sec/4*(i-sec/4)-0.5*(i-sec/4)*(i-sec/4))/eth.RMh;
				(*(pos+i-1))[1]=0;(*(pos+i-1))[2]=0; 
			}
			for(i=sec/2;i<=sec*3/4;i++)	{
				(*(vn+i-1))[0]=0;(*(vn+i-1))[1]=i-sec/2;(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=sec/4*sec/4/eth.RMh;
				(*(pos+i-1))[1]=0.5*(i-sec/2)*(i-sec/2)/(eth.cosL*eth.RNh);
				(*(pos+i-1))[2]=0; 
			}
			for(i=sec*3/4;i<=sec;i++)	{
				(*(vn+i-1))[0]=0;(*(vn+i-1))[1]=sec/4-(i-sec*3/4);(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=sec/4*sec/4/eth.RMh;
				(*(pos+i-1))[1]=(0.5*(sec/4)*(sec/4)+sec/4*(i-sec*3/4)-0.5*(i-sec*3/4)*(i-sec*3/4))/(eth.cosL*eth.RNh);
				(*(pos+i-1))[2]=0; 
			}
		 	break;
		 case KF_Square: //四边形
		 	//将时间设置为8的倍数
			for(i=0;i<num;i++)	*(err+i)=(rand()%100/100.0f-0.5f)/1.0f;
		 	for(i=0;i<num/8;i++)	{(*(ab+i))[0]=1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}	//只向x轴运动,前半段加速，后半段减速
			for(i=num/8;i<num/4;i++)	{(*(ab+i))[0]=-1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}
			for(i=num/4;i<num*3/8;i++)	{(*(ab+i))[0]=1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}	
			for(i=num*3/8;i<num/2;i++)	{(*(ab+i))[0]=-1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}
			for(i=num/2;i<num*5/8;i++)	{(*(ab+i))[0]=1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}	//只向x轴运动,前半段加速，后半段减速
			for(i=num*5/8;i<num*3/4;i++)	{(*(ab+i))[0]=-1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}
			for(i=num*3/4;i<num*7/8;i++)	{(*(ab+i))[0]=1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}	
			for(i=num*7/8;i<num;i++)	{(*(ab+i))[0]=-1;(*(ab+i))[1]=0;(*(ab+i))[2]=glv.g0;}
		
			for(i=0;i<num/4;i++)	 (*(att+i))[0]=(*(att+i))[1]=(*(att+i))[2]=0;
			for(i=num/4;i<num/2;i++)	 {(*(att+i))[0]=(*(att+i))[1]=0;(*(att+i))[2]=90;}
			for(i=num/2;i<3*num/4;i++)  {(*(att+i))[0]=(*(att+i))[1]=0;(*(att+i))[2]=180;}
			for(i=3*num/4;i<num;i++)	 {(*(att+i))[0]=(*(att+i))[1]=0;(*(att+i))[2]=270;}

			for(i=1;i<sec/8;i++)	{//x正向加速（以下说的x,y轴指导航坐标系）
				(*(vn+i-1))[0]=i;(*(vn+i-1))[1]=0;(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=0.5*i*i/eth.RMh;(*(pos+i-1))[1]=0;(*(pos+i-1))[2]=0; 
			}
			for(i=sec/8;i<sec/4;i++)	{//x正向减速
				(*(vn+i-1))[0]=sec/8-(i-sec/8);(*(vn+i-1))[1]=0;(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=(0.5*(sec/8)*(sec/8)+sec/8*(i-sec/8)-0.5*(i-sec/8)*(i-sec/8))/eth.RMh;
				(*(pos+i-1))[1]=0;(*(pos+i-1))[2]=0; 
			}
			for(i=sec/4;i<sec*3/8;i++)	{//y正向加速
				(*(vn+i-1))[0]=0;(*(vn+i-1))[1]=i-sec/4;(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=sec/8*sec/8/eth.RMh;
				(*(pos+i-1))[1]=0.5*(i-sec/4)*(i-sec/4)/(eth.cosL*eth.RNh);
				(*(pos+i-1))[2]=0; 
			}
			for(i=sec*3/8;i<sec/2;i++)	{//y正向减速
				(*(vn+i-1))[0]=0;(*(vn+i-1))[1]=sec/8-(i-sec*3/8);(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=sec/8*sec/8/eth.RMh;
				(*(pos+i-1))[1]=(0.5*(sec/8)*(sec/8)+sec/8*(i-sec*3/8)-0.5*(i-sec*3/8)*(i-sec*3/8))/(eth.cosL*eth.RNh);
				(*(pos+i-1))[2]=0; 
			}
			for(i=sec/2;i<sec*5/8;i++)	{//x反向加速
				(*(vn+i-1))[0]=-(i-sec/2);(*(vn+i-1))[1]=0;(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=(sec/8*sec/8-0.5*(i-sec/2)*(i-sec/2))/eth.RMh;
				(*(pos+i-1))[1]=sec/8*sec/8/(eth.cosL*eth.RNh);(*(pos+i-1))[2]=0; 
			}
			for(i=sec*5/8;i<sec*3/4;i++)	{//x反向减速
				(*(vn+i-1))[0]=-(sec/8-(i-sec*5/8));(*(vn+i-1))[1]=0;(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=(0.5*sec/8*sec/8-sec/8*(i-sec*5/8)+0.5*(i-sec*5/8)*(i-sec*5/8))/eth.RMh;
				(*(pos+i-1))[1]=sec/8*sec/8/(eth.cosL*eth.RNh);(*(pos+i-1))[2]=0; 
			}
			for(i=sec*3/4;i<sec*7/8;i++)	{//y反向加速
				(*(vn+i-1))[0]=0;(*(vn+i-1))[1]=-(i-sec*3/4);(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=0;
				(*(pos+i-1))[1]=(sec/8*sec/8-0.5*(i-sec*3/4)*(i-sec*3/4))/(eth.cosL*eth.RNh);
				(*(pos+i-1))[2]=0;
			}
			for(i=sec*7/8;i<sec;i++)	{//y反向减速
				(*(vn+i-1))[0]=0;(*(vn+i-1))[1]=-(sec/8-(i-sec*7/8));(*(vn+i-1))[2]=0;	
				(*(pos+i-1))[0]=0;
				(*(pos+i-1))[1]=(0.5*sec/8*sec/8-sec/8*(i-sec*7/8)+0.5*(i-sec*7/8)*(i-sec*7/8))/(eth.cosL*eth.RNh);
				(*(pos+i-1))[2]=0; 
			}
//			Test_print_Predict(sec,vn,pos);
			break;
		 case KF_Circule: 	//圆形
			break;
	}
	test(att, wb, ab, vn, pos, num, err);

}
//卡尔曼滤波增益计算测试
void KF_Kk_test(){
	arm_matrix_instance_f32 P, H,HT,PHT,HPHT,K,TEMP,R;
	mat15f P_data;
	mat6_15f H_data;
	mat15_6f HT_data;
	mat15_6f K_data;
	mat6f HPHT_data;
	mat15_6f PHT_data;
	mat6f TEMP_data;
	mat6f R_data;

	mat15f p={	1,	0,	0,	1,	2,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				0,	2,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				0,	0,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				1,	0,	0,	4,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				2,	0,	0,	0,	5,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				3,	0,	0,	0,	0,	6,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				0,	0,	0,	0,	0,	0,	7,	0,	0,	0,	0,	0,	0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	8,	0,	0,	0,	0,	0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	0,	9,	0,	0,	0,	0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	0,	0,	10, 0,	0,	0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	11,  0, 0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	12, 0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	13,  0,  0,  
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	14,  0,  
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	15};
	set2matf(*p, 15, 15, *P_data, &P);

	vec6f r = {0,1,1,1,1,1};	
	set2diag_sq(r,6,*R_data,&R);

	set2matf(*O6_15, 6, 15, *H_data, &H);
	setMat3(O33,&H,0,0);
	setMat3(I33,&H,0,3);
	setMat3(O33,&H,0,6);
	setMat3(O33,&H,0,9);
	setMat3(O33,&H,0,12);
	setMat3(O33,&H,3,0);
	setMat3(O33,&H,3,3);
	setMat3(I33,&H,3,6);
	setMat3(O33,&H,3,9);
	setMat3(O33,&H,3,12);		
	set2matf(*O15_6, 15, 6, *HT_data, &HT);
	arm_mat_trans_f32(&H, &HT);
	set2matf(*O15_6, 15, 6, *K_data, &K);
	set2matf(*O66, 6, 6, *HPHT_data, &HPHT);
	set2matf(*O15_6, 15, 6, *PHT_data, &PHT);
	set2matf(*O66, 6, 6, *TEMP_data, &TEMP);

	arm_mat_mult_f32(&P,&HT, &PHT);
	arm_mat_mult_f32(&H, &PHT, &HPHT);
	arm_mat_add_f32(&HPHT, &R, &HPHT); 	//加入量测噪声
	int res = arm_mat_inverse_f32(&HPHT, &TEMP);	//注意求逆不能用同一个矩阵！！！
	printf("res=%d\r\n",res);
	arm_mat_mult_f32(&PHT,&TEMP, &K);
	print_mat_name(K,"K");
		

}
void KF_Pk_test(){
	arm_matrix_instance_f32 P, F, FT, FP;
	mat15f P_data,F_data,FT_data,FP_data;

	mat15f p={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	set2diag_sq(*p,15,*P_data,&P);



	mat15f f={	1,	0,	0,	1,	2,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				0,	2,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				0,	0,	3,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				1,	0,	0,	4,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				2,	0,	0,	0,	5,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				3,	0,	0,	0,	0,	6,	0,	0,	0,	0,	0,	0,	0,	0,	0,	
				0,	0,	0,	0,	0,	0,	7,	0,	0,	0,	0,	0,	0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	8,	0,	0,	0,	0,	0,	0,	0,	
				0,	0,	0,	0,	10,	0,	0,	0,	9,	0,	0,	0,	0,	0,	0,	
				0,	0,	0,	0,	20,	0,	50,	0,	0,	10, 0,	0,	0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	11,  0, 0,	0,	0,	
				0,	0,	0,	0,	0,	0,	70,	0,	0,	0,	0,	12, 0,	0,	0,	
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	13,  0,  0,  
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	14,  0,  
				0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	15};
	set2matf(*f, 15, 15, *F_data, &F);
	set2matf(*O15_15, 15, 15, *FT_data, &FT);
	set2matf(*O15_15, 15, 15, *FP_data, &FP);

	arm_mat_mult_f32(&F, &P, &FP);
	arm_mat_trans_f32(&F, &FT);
	arm_mat_mult_f32(&FP, &FT, &P);
	
	print_mat_name(P,"P");

}

