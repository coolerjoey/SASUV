#ifndef _EC_DDM350B_H
#define _EC_DDM350B_H

#include "mymath.h"

enum 
{
	// Frame IDs (Commands)
	ddm_kGetData = 0x04,
	ddm_kGetDataResp = 0x84,
	ddm_kGetMagIncline = 0x86,
};

enum 
{
	ddm_kBufferSize = 512, // max size of input buffer
	ddm_kPacketMinSize = 4 // min size of serial packet
};

typedef struct{
	// integer Euler angles (Degrees * 100)
	int32_t roll_sensor;
	int32_t pitch_sensor;
	int32_t yaw_sensor;
	vec3f euler;
}DDM;
extern DDM ddm;

void ec_ddm_init(void);
void ec_ddm_update(void);
void ec_ddm_trigger(void);
void ec_ddm_check(void);
void ddm_get_euler(vec3f);

#endif


