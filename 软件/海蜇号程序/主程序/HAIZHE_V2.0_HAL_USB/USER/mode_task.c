#include "mode_task.h"
#include "global_para.h"
#include "motors.h"
#include "attitude_controller.h"
#include "position_controller.h"

extern SYS_FLAGS sys_flag;

TASK_LIST Task_List[Task_MAX]; //自动模式任务列表

u8 task_init(){
	if(!sys_check_ins()){	//检查ahrs
		return false;
	}
	return true;
}


//设置任务目标值
void set_task_value(float roll,float pitch,float yaw,float depth){
		Sensor_target.ins.euler[0] = roll;
		Sensor_target.ins.euler[1] = pitch;
		Sensor_target.ins.euler[2] = yaw;
		Sensor_target.barometer.depth = depth;
}

void set_task(){
	static u8 auto_task_num=0;	//现在运行任务的第几个动作
	static float depth_hold=0.0;	//保存深度设定值
	static float yaw_hold=0.0;
	static u32 second_task_start=0;					//任务里面每个动作的开始运行时间
	
	//计算当前任务时间是否超过设定时间
	if(sys_flag.task_auto_switch || sys_time.one_second-second_task_start>=Task_List[auto_task_num-1].task_beat){//超过该任务设定运行时间，更新设定值
		sys_flag.task_auto_switch = true;//使能任务设定标志
		second_task_start = sys_time.one_second;//更新任务起始时间
		if(auto_task_num == vp.task_total_num){//所有任务执行完毕
			sys_flag.task_run = false;
			auto_task_num = 0;
			sys_flag.task_auto_switch = false;
			second_task_start = 0;
			depth_hold = 0;
//			openmv_record_control("fe 0 14 0 ff");	//向openmv发送任务结束指令
			printf("任务结束 \r\n");
			return;
		}
	}
	
//更新期望姿态和深度,更新任务时设置一次即可
	if(sys_flag.task_auto_switch){
		printf("总任务数 %d ,设置第 %d 个任务,开始时间 %d \r\n",vp.task_total_num,auto_task_num+1,second_task_start);
//		sum_init(); //pid积分项清零
		yaw_hold = Sensor_latest.ins.euler[2];
//		for (i=0;i<MOTORS_MAX_NUMER;i++) PID_motor_out[i] = 1050;//旧参考值为1125		
		switch(Task_List[auto_task_num].task_list){
			case 0://应该不会切换到空白动作
				break;
			case 1://下沉
				depth_hold = Task_List[auto_task_num].task_val;
				set_task_value(0,0,yaw_hold,Task_List[auto_task_num].task_val);
				break;
			case 2://向前，以x轴正向为前进方向，即pitch为负角度时向前
				set_task_value(0,-Task_List[auto_task_num].task_val,yaw_hold,depth_hold);//以y轴正向为前进方向，即roll为正角度时前进
				
				break;
			case 3://向后
				set_task_value(0,Task_List[auto_task_num].task_val,yaw_hold,depth_hold);//roll为负角度时后退
				
				break;
			case 4://左移
				set_task_value(Task_List[auto_task_num].task_val,0,yaw_hold,depth_hold);//以x轴负向为左移方向，即pitch为正角度时左移
				
				break;
			case 5://右移
				set_task_value(-Task_List[auto_task_num].task_val,0,yaw_hold,depth_hold);//pitch为负角度时右移
				
				break;
			case 6://左转
				yaw_hold += Task_List[auto_task_num].task_val;//在当前yaw角基础上左转目标角度 or 累加角度？ Sensor_latest.ins.euler[2]
				if(yaw_hold>360)  yaw_hold -= 360; //正转过一周
				set_task_value(0,0,yaw_hold,depth_hold);	
				
				break;
			case 7://右转
				yaw_hold -= Task_List[auto_task_num].task_val;//在当前yaw角基础上左转目标角度  Sensor_latest.ins.euler[2]
				if(yaw_hold<-360)  yaw_hold += 360; //反转过一周
				set_task_value(0,0,yaw_hold,depth_hold);
				
				break;
		}
		auto_task_num++;
		sys_flag.task_auto_switch = false;//清空任务设定标志
	}

}

//任务规划模式
void task_run(){

	if(!sys_flag.task_run){//没有发送开始命令
		return;
	}
	//任务设置
	set_task();
	//运行姿态控制器
	att_controller_run(Sensor_target.ins.euler[0],Sensor_target.ins.euler[1],Sensor_target.ins.euler[2]);
	//运行深度控制器
	pos_z_controller_run(Sensor_target.barometer.depth);
}




