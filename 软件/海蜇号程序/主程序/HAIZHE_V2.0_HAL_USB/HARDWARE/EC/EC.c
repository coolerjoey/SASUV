#include "EC.h"
#include "config.h"
#include "EC_TCM.h"
#include "EC_DDM350B.h"

ElectronicCompass ec={0};

void ec_init(){
#if EC_TYPE==DDM350B
	ec_ddm_init();
#elif EC_TYPE==TCM
	ec_tcm_init();
#endif
}

void ec_update(){
#if EC_TYPE==DDM350B
	ec_ddm_update();
	ddm_get_euler(ec.euler);
#elif EC_TYPE==TCM
	ec_tcm_update();
	tcm_get_euler(ec.euler);
	tcm_get_acc(ec.acc);
	tcm_get_temp(&ec.temp);
#endif
}

void ec_check(){
#if EC_TYPE==DDM350B
		ec_ddm_check();
#elif EC_TYPE==TCM
		ec_tcm_check();
#endif

}

void ec_get_euler(vec3f euler){
	arm_copy_f32(ec.euler, euler, 3);
}
void ec_get_acc(vec3f acc){
	arm_copy_f32(ec.acc, acc, 3);
}
void ec_get_mag(vec3f mag){
	arm_copy_f32(ec.mag, mag, 3);
}
void ec_get_temp(float *temp){
	*temp = ec.temp;
}


void ec_calibration(){
	
}

