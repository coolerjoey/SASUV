#include "control_mode.h"
#include "mode_manual.h"
#include "mode_auto.h"
#include "mode_stabilize.h"
#include "mode_althold.h"
#include "mode_task.h"
#include "mode_surface.h"
#include "mode_rtl.h"
#include "global_para.h"


bool success = false;	//记录模式能否设置为设定模式

CONTROL_MODE_CHAR control_mode_char[]={
	{"STABILIZE",STABILIZE},
	{"ALT_HOLD",ALT_HOLD},
	{"AUTO",AUTO},
	{"GUIDED",GUIDED},
	{"LOITER",LOITER},
	{"RTL",RTL},
	{"CIRCLE",CIRCLE},
	{"SURFACE",SURFACE},
	{"POSHOLD",POSHOLD},
	{"MANUAL",MANUAL},
	{"TASK",TASK},	
};

bool set_mode(CONTROL_MODE mode){
	char mode_str[10]="";
	if(mode == vp.control_mode){
		return true;
	}
	switch (mode) {
    case STABILIZE:
        success = stabilize_init();
		sprintf(mode_str,"%s","STABILIZE");
        break;
    case ALT_HOLD:
        success = althold_init();
		sprintf(mode_str,"%s","ALT_HOLD");
        break;
    case AUTO:
        success = auto_init();
		sprintf(mode_str,"%s","AUTO");
        break;
	case GUIDED:
//		success = circle_init();
		success = true;
		sprintf(mode_str,"%s","GUIDED");
		break;
	case CIRCLE:
//		success = circle_init();
		sprintf(mode_str,"%s","CIRCLE");
		break;
	case LOITER:
//		success = loiter_init();
		sprintf(mode_str,"%s","LOITER");
		break;
	case RTL:
		success = rtl_init();
		sprintf(mode_str,"%s","RTL");
		break;
    case SURFACE:
        success = surface_init();
		sprintf(mode_str,"%s","SURFACE");
        break;
	case POSHOLD:
//		success = poshold_init();
		sprintf(mode_str,"%s","POSHOLD");
		break;
    case MANUAL:
        success = manual_init();
		sprintf(mode_str,"%s","MANUAL");
        break;
	case TASK:
		success = task_init();
		sprintf(mode_str,"%s","TASK");
		break;
    default:
        success = false;
		sprintf(mode_str,"%s","UNKNOW");
        break;
    }
	vp.control_mode = success?mode:vp.control_mode;
	if(success) printf("[gcs->] change to mode %s \r\n",mode_str);
	else {
		printf("[gcs->] Failed change to mode %s!\r\n",mode_str);
	}
    return success;
}

//更新控制模式
void update_control_mode(){
	switch(vp.control_mode){
		case MANUAL:
			manual_run();
			break;
		case STABILIZE:
			stabilize_run();
			break;
		case AUTO:
			auto_run();
			break;
		case CIRCLE:
//			circle_run();
			break;
		case LOITER:
//			loiter_run();
			break;	
		case ALT_HOLD:
			althold_run();
//			althold_run_test();
			break;
		case TASK:
			task_run();
			break;
		case SURFACE:
			surface_run();
			break;
		case RTL:
			rtl_run();
			break;
		default:
			break;
	}
}


