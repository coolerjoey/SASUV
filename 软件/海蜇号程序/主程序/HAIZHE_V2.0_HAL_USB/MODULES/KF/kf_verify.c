#include "kf_verify.h"
#include "kf.h"
#include "malloc.h"
#include "sensor.h"
#include "parameter.h"

//æ≈÷·kf≤‚ ‘
static bool show_task_pass=false;
void KF_AHRS_verify(){
	static bool kf_verify_enable = false;
	static u32 start_time = 0;
	static int num = 0;
	if(sys_flag.kf_verify != kf_verify_enable){
		kf_verify_enable = sys_flag.kf_verify;
		if(!sys_flag.kf_verify) return;	//Õ£÷π≤‚ ‘
		KF_init(O31, O31, 0.01);	//ø™ º≤‚ ‘£¨Ω¯––≥ı ºªØ
		start_time = sys_time.one_second;
		show_task_pass = sys_flag.show_task_pass;
		sys_flag.show_task_pass = false;
		printf("Start KF_AHRS_verify \r\n");
	}
	else if(!sys_flag.kf_verify) return;
	if(sys_time.one_second-start_time == vp.kf_verify_sec){	//≥¨ ±ÕÀ≥ˆ≤‚ ‘
		printf("Finish KF_AHRS_verify \r\n");		
		kf.KFinit_complete = false;
		sys_flag.kf_verify = false;
		sys_flag.show_task_pass = show_task_pass;
		kf_verify_enable = false;
		num = 0;
		return;
	}
//	vec3f acc = {1,0,9.8};
	KF_update(Sensor_latest.ins.euler,Sensor_latest.ins.gyro,Sensor_latest.ins.acc,1,0.01,15);	//‘À––ø®∂˚¬¸¬À≤®

	printf("%.2f ",(float)num++/100.0f);
	printf("%.5f %.5f %.5f ",Sensor_latest.ins.acc[0],Sensor_latest.ins.acc[1],Sensor_latest.ins.acc[2]);
//	printf("%.5f %.5f %.5f ",Sensor_latest.ins.euler_rad[0],Sensor_latest.ins.euler_rad[1],Sensor_latest.ins.euler_rad[2]);
	printf("%.5f %.5f %.5f ",sins.an[0],sins.an[1],sins.an[2]);
	printf("%.5f %.5f %.5f ",sins.vn[0],sins.vn[1],sins.vn[2]);
	printf("%.5f %.5f %.5f ",sins.dn[0],sins.dn[1],sins.dn[2]);
//	printf("%.5f %.5f %.5f %f ",kf.Zk_data[0],kf.Zk_data[1],kf.Zk_data[3]*eth.RMh,kf.Zk_data[4]*eth.RMh);
	printf("\r\n");
}

