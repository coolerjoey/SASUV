#include "model_test.h"
#include "mymath.h"
#include "model.h"
#include "parameter.h"
#include "malloc.h"

/*
	ģ�͸��²��Գ��򣬴�ӡ������Ϣ��ʹ��Ŀ¼�µ�model_test.py�ű�����
*/

void print_ab(){printf("%.5f %.5f %.5f ",model.ab[0], model.ab[1], model.ab[2]);}//bϵ���ٶ�
void print_wb(){printf("%.5f %.5f %.5f ",model.X_data[9], model.X_data[10], model.X_data[11]);}//bϵ���ٶ�
void print_vb(){printf("%.5f %.5f %.5f ",model.X_data[6], model.X_data[7], model.X_data[8]);}//bϵ�ٶ�
void print_pn(){printf("%.5f %.5f %.5f ",model.X_data[0], model.X_data[1], model.X_data[2]);}//nϵ���λ��

void _model_test_print(int i, MODEL_TEST_TYPE type){
		printf("%.2f ",(float)i/100.0f);
		switch(type){
			case MODEL_RISE:
			case MODEL_Line:
			case MODEL_PolyLine: 
			case MODEL_Square: 
			case MODEL_Circule: 
			case MODEL_DIV:	print_ab();print_vb();print_pn();break;
			case MODEL_SPIN: 
			case MODEL_ROLL:
			case MODEL_PITCH:print_wb();break;
		}
		printf("\r\n"); 
}

void model_test(int sec, MODEL_TEST_TYPE type){
	int num=sec*100;
	vec3f pos;	//��ʼλ��
	vec3f att;
	vec6f force;	//����
	int i=0;
	switch(type){
		case MODEL_RISE:	//����������ֱ����
			pos[0]=0; pos[1]=0; pos[2]=1;	//ˮ��1m
			arm_copy_f32(O61, force, 6);
			arm_copy_f32(O31, att, 3);
			model_init(O31, O31, pos, 0.01);
			for(int i=0;i<num;i++)	{
				model_updata_core(att, force);//����ģ��ϵͳ
				_model_test_print(i, MODEL_RISE);
			}
			break;
		case MODEL_Line:{		//ֱ���˶�
			vec6f f = {0,0,0,0,0.48,0.48};
			arm_copy_f32(f, force, 6);	//ˮƽ��1N
			arm_copy_f32(O31, att, 3);
			arm_copy_f32(O31, pos, 3);
			model_init(O31, O31, pos, 0.01);
			for(i=0;i<num;i++)	{
				model_updata_core(att, force);//����ģ��ϵͳ
				_model_test_print(i, MODEL_Line);
			}
			break;
		}
		case MODEL_PolyLine:{	//����
			arm_copy_f32(O31, pos, 3);
			vec6f f = {0,0,0,0,1,1};
			arm_copy_f32(O31, att, 3);
			arm_copy_f32(f, force, 6); //ˮƽ��1N
			model_init(O31, O31, pos, 0.01);
			for(i=0;i<num/3;i++)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_PolyLine);
			}
//			arm_copy_f32(O31, force, 6);	//����ˮƽ����
			arm_copy_f32(f, force, 6); //ˮƽ��1N
			for(i=num/3;i<num*2/3;i++)	{
				vec3f a = {0, 0, 90*3*(i-num/3)/num};	//��num/3ʱ���ڵ�����90��
				arm_copy_f32(a, att, 3);
				model_updata_core(att, force);
				_model_test_print(i, MODEL_PolyLine);
			}
			arm_copy_f32(f, force, 6); //ˮƽ��1N
			att[2] = 90;	//ת��90��
			for(i=num*2/3;i<num;++i)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_PolyLine);
			}
			break;
		}
		case MODEL_Square:{	//�ı���
			arm_copy_f32(O31, pos, 3);
			vec6f f = {0,0,0,0,1,1};
			model_init(O31, O31, pos, 0.01);
			arm_copy_f32(f, force, 6); //ˮƽ��1N
			arm_copy_f32(O31, att, 3);
			for(i=0;i<num/8;i++)	{	
				model_updata_core(att, force);
				_model_test_print(i, MODEL_Square);
			}
//			arm_copy_f32(O31, force, 6);	//����ˮƽ����
			arm_copy_f32(f, force, 6); //����ˮƽ��Ϊ1N
			for(i=num/8;i<num/4;++i)	{
				vec3f a = {0, 0, 90*8*(i-num/8)/num};	//��num/8ʱ���ڵ�����90��
				arm_copy_f32(a, att, 3);
				model_updata_core(att, force);
				_model_test_print(i, MODEL_Square);
			}
			att[2] = 90;	//ת��90��
			arm_copy_f32(f, force, 6);
			for(i=num/4;i<num*3/8;++i)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_Square);
			}
//			arm_copy_f32(O31, force, 6);	//����ˮƽ����
			arm_copy_f32(f, force, 6); //����ˮƽ��Ϊ1N
			for(i=num*3/8;i<num/2;++i)	{
				vec3f a = {0, 0, 90+90*8*(i-num*3/8)/num};	//��ת��
				arm_copy_f32(a, att, 3);
				model_updata_core(att, force);
				_model_test_print(i, MODEL_Square);
			}
			att[2] = 180;	//ת��180��
			arm_copy_f32(f, force, 6);
			for(i=num/2;i<num*5/8;++i)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_Square);
			}
//			arm_copy_f32(O31, force, 6);	//����ˮƽ����
			arm_copy_f32(f, force, 6); //����ˮƽ��Ϊ1N
			for(i=num*5/8;i<num*3/4;++i)	{
				vec3f a = {0, 0, 180+90*8*(i-num*5/8)/num};	//�ٴ�ת��
				arm_copy_f32(a, att, 3);
				model_updata_core(att, force);
				_model_test_print(i, MODEL_Square);
			}
			att[2] = 270;	//ת��180��
			arm_copy_f32(f, force, 6);
			for(i=num*3/4;i<num;++i)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_Square);
			}
			break;
		}
		case MODEL_Circule:
			//TODO
			break;
		case MODEL_SPIN:{
			arm_copy_f32(O31, pos, 3);
			model_init(O31, O31, pos, 0.01);
			vec6f f={0, 0, 0, 0, 1, -1};
			arm_copy_f32(f, force, 6);
			arm_copy_f32(O31, att, 3);
			for(i=0;i<num;i++)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_SPIN);
			}
			break;
		}
		case MODEL_ROLL:{
			arm_copy_f32(O31, pos, 3);
			model_init(O31, O31, pos, 0.01);
			vec6f f={1, -1, -1, 1, 0, 0};
			arm_copy_f32(f, force, 6);
			arm_copy_f32(O31, att, 3);
			for(i=0;i<num;i++)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_ROLL);
			}
			break;
		}
		case MODEL_PITCH:{
			arm_copy_f32(O31, pos, 3);
			model_init(O31, O31, pos, 0.01);
			vec6f f={-1, -1, 1, 1, 0, 0};
			arm_copy_f32(f, force, 6);
			arm_copy_f32(O31, att, 3);
			for(i=0;i<num;i++)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_PITCH);
			}
			break;
		}
		case MODEL_DIV:{
			arm_copy_f32(O31, pos, 3);
			model_init(O31, O31, pos, 0.01);
			vec6f f={2.6, 2.6, 2.6, 2.6, 0, 0};
			arm_copy_f32(f, force, 6);
			arm_copy_f32(O31, att, 3);
			for(i=0;i<num;i++)	{
				model_updata_core(att, force);
				_model_test_print(i, MODEL_DIV);
			}
			break;
		}
	}
}



