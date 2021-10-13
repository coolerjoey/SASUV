#include "mode_task.h"
#include "global_para.h"
#include "motors.h"
#include "attitude_controller.h"
#include "position_controller.h"

extern SYS_FLAGS sys_flag;

TASK_LIST Task_List[Task_MAX]; //�Զ�ģʽ�����б�

u8 task_init(){
	if(!sys_check_ins()){	//���ahrs
		return false;
	}
	return true;
}


//��������Ŀ��ֵ
void set_task_value(float roll,float pitch,float yaw,float depth){
		Sensor_target.ins.euler[0] = roll;
		Sensor_target.ins.euler[1] = pitch;
		Sensor_target.ins.euler[2] = yaw;
		Sensor_target.barometer.depth = depth;
}

void set_task(){
	static u8 auto_task_num=0;	//������������ĵڼ�������
	static float depth_hold=0.0;	//��������趨ֵ
	static float yaw_hold=0.0;
	static u32 second_task_start=0;					//��������ÿ�������Ŀ�ʼ����ʱ��
	
	//���㵱ǰ����ʱ���Ƿ񳬹��趨ʱ��
	if(sys_flag.task_auto_switch || sys_time.one_second-second_task_start>=Task_List[auto_task_num-1].task_beat){//�����������趨����ʱ�䣬�����趨ֵ
		sys_flag.task_auto_switch = true;//ʹ�������趨��־
		second_task_start = sys_time.one_second;//����������ʼʱ��
		if(auto_task_num == vp.task_total_num){//��������ִ�����
			sys_flag.task_run = false;
			auto_task_num = 0;
			sys_flag.task_auto_switch = false;
			second_task_start = 0;
			depth_hold = 0;
//			openmv_record_control("fe 0 14 0 ff");	//��openmv�����������ָ��
			printf("������� \r\n");
			return;
		}
	}
	
//����������̬�����,��������ʱ����һ�μ���
	if(sys_flag.task_auto_switch){
		printf("�������� %d ,���õ� %d ������,��ʼʱ�� %d \r\n",vp.task_total_num,auto_task_num+1,second_task_start);
//		sum_init(); //pid����������
		yaw_hold = Sensor_latest.ins.euler[2];
//		for (i=0;i<MOTORS_MAX_NUMER;i++) PID_motor_out[i] = 1050;//�ɲο�ֵΪ1125		
		switch(Task_List[auto_task_num].task_list){
			case 0://Ӧ�ò����л����հ׶���
				break;
			case 1://�³�
				depth_hold = Task_List[auto_task_num].task_val;
				set_task_value(0,0,yaw_hold,Task_List[auto_task_num].task_val);
				break;
			case 2://��ǰ����x������Ϊǰ�����򣬼�pitchΪ���Ƕ�ʱ��ǰ
				set_task_value(0,-Task_List[auto_task_num].task_val,yaw_hold,depth_hold);//��y������Ϊǰ�����򣬼�rollΪ���Ƕ�ʱǰ��
				
				break;
			case 3://���
				set_task_value(0,Task_List[auto_task_num].task_val,yaw_hold,depth_hold);//rollΪ���Ƕ�ʱ����
				
				break;
			case 4://����
				set_task_value(Task_List[auto_task_num].task_val,0,yaw_hold,depth_hold);//��x�Ḻ��Ϊ���Ʒ��򣬼�pitchΪ���Ƕ�ʱ����
				
				break;
			case 5://����
				set_task_value(-Task_List[auto_task_num].task_val,0,yaw_hold,depth_hold);//pitchΪ���Ƕ�ʱ����
				
				break;
			case 6://��ת
				yaw_hold += Task_List[auto_task_num].task_val;//�ڵ�ǰyaw�ǻ�������תĿ��Ƕ� or �ۼӽǶȣ� Sensor_latest.ins.euler[2]
				if(yaw_hold>360)  yaw_hold -= 360; //��ת��һ��
				set_task_value(0,0,yaw_hold,depth_hold);	
				
				break;
			case 7://��ת
				yaw_hold -= Task_List[auto_task_num].task_val;//�ڵ�ǰyaw�ǻ�������תĿ��Ƕ�  Sensor_latest.ins.euler[2]
				if(yaw_hold<-360)  yaw_hold += 360; //��ת��һ��
				set_task_value(0,0,yaw_hold,depth_hold);
				
				break;
		}
		auto_task_num++;
		sys_flag.task_auto_switch = false;//��������趨��־
	}

}

//����滮ģʽ
void task_run(){

	if(!sys_flag.task_run){//û�з��Ϳ�ʼ����
		return;
	}
	//��������
	set_task();
	//������̬������
	att_controller_run(Sensor_target.ins.euler[0],Sensor_target.ins.euler[1],Sensor_target.ins.euler[2]);
	//������ȿ�����
	pos_z_controller_run(Sensor_target.barometer.depth);
}




