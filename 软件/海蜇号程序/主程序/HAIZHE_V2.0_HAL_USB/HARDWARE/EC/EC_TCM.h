#ifndef _EC_TCM_H
#define _EC_TCM_H

#include "mymath.h"

enum
{
	// Frame IDs (Commands)
	tcm_kGetModInfo=1, // 1
	tcm_kGetModInfoResp=2, // 2
	tcm_kSetDataComponents=3, // 3
	tcm_kGetData=4, // 4
	tcm_kGetDataResp=5, // 5
	// Data Component IDs
	tcm_kHeading = 5, // 5 - type Float32
	tcm_kTemperature = 7, // 7 - type Float32
	tcm_kAccelX = 21, // 21 - type Float32
	tcm_kAccelY, // 22 - type Float32
	tcm_kAccelZ, // 23 - type Float32
	tcm_kPitch, // 24 - type Float32
	tcm_kRoll, // 25 - type Float32
};
enum
{
	tcm_kBufferSize = 512, // max size of input buffer
	tcm_kPacketMinSize = 5 // min size of serial pactcm_ket
};

typedef struct{
	// integer Euler angles (Degrees * 100)
	int32_t roll_sensor;
	int32_t pitch_sensor;
	int32_t yaw_sensor;
	float temp;
	vec3f euler;
	vec3f acc;
}EC_TCM;
extern EC_TCM tcm;

void ec_tcm_init(void);
void ec_tcm_update(void);
void ec_tcm_trigger(void);
void ec_tcm_check(void);
void tcm_get_euler(vec3f);
void tcm_get_acc(vec3f);
void tcm_get_temp(float *temp);

#endif


