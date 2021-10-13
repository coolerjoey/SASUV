#ifndef _FLASH_H
#define _FLASH_H

#include "sys.h"
#include "flash_w25qxx.h"

#define FLASHTYPE W25Q_xx	//flash����
#define FLASH_SECTOR_SIZE W25QXX_Sector_Size	//flash������С
#define PARAM_SAVED_MAX 4096	//flash��󱣴����ݸ���

//���4096������
//ע�⣺�����ӻ��߼�С��������ʱ��ϵͳ���ϵ�ʱ���Զ�ʹ��Ĭ��ֵ�����µĲ����������ڸ��Ĳ���ǰ����Ӧ����Ĭ��ֵ����Ϊ��ǰֵ������
//��û�����ӻ��߼�С��ֻ���޸ģ���Ҫ�ֶ����������ͬ�������ڸ��Ĳ���ǰ����Ӧ����Ĭ��ֵ����Ϊ��ǰֵ������
typedef enum {
	param_num = 0,	//��һ���������ڼ�¼��ǰ��Ҫ����Ĳ���������һ��Ҫ���ڵ�һ����
	
	sysid_my_gcs = 1,	
	frame_class,
	
	uartA_baud,	
	uartB_baud,	
	uartC_baud,	
	uartD_baud,	
	uartE_baud,	
	uartF_baud,	
	
	pos_init_roll,
	pos_init_pitch,
	pos_init_yaw,	

	throttle_hover,	
	
	remoteCtrl_type,
	remoteCtrl_unConVal_ch1,
	remoteCtrl_unConVal_ch2,
	remoteCtrl_unConVal_ch3,
	remoteCtrl_unConVal_ch4,
	remoteCtrl_unConVal_ch5,
	remoteCtrl_unConVal_ch6,
	remoteCtrl_unConVal_ch7,
	remoteCtrl_unConVal_ch8,
	remoteCtrl_ConVal_ch1_min,
	remoteCtrl_ConVal_ch1_mid,
	remoteCtrl_ConVal_ch1_max,
	remoteCtrl_ConVal_ch2_min,
	remoteCtrl_ConVal_ch2_mid,
	remoteCtrl_ConVal_ch2_max,
	remoteCtrl_ConVal_ch3_min,
	remoteCtrl_ConVal_ch3_mid,
	remoteCtrl_ConVal_ch3_max,
	remoteCtrl_ConVal_ch4_min,
	remoteCtrl_ConVal_ch4_mid,
	remoteCtrl_ConVal_ch4_max,
	remoteCtrl_ConVal_ch5_min,
	remoteCtrl_ConVal_ch5_mid,
	remoteCtrl_ConVal_ch5_max,
	remoteCtrl_ConVal_ch6_min,
	remoteCtrl_ConVal_ch6_mid,
	remoteCtrl_ConVal_ch6_max,
	remoteCtrl_ConVal_ch7_min,
	remoteCtrl_ConVal_ch7_mid,
	remoteCtrl_ConVal_ch7_max,
	remoteCtrl_ConVal_ch8_min,
	remoteCtrl_ConVal_ch8_mid,
	remoteCtrl_ConVal_ch8_max,
	
	pidAngleRoll_kp,
	pidAngleRoll_ki,
	pidAngleRoll_kd,
	pidAngleRoll_iLimit,
	pidAngleRoll_outputLimit,
	pidAnglePitch_kp,
	pidAnglePitch_ki,
	pidAnglePitch_kd,
	pidAnglePitch_iLimit,
	pidAnglePitch_outputLimit,
	pidAngleYaw_kp,
	pidAngleYaw_ki,
	pidAngleYaw_kd,
	pidAngleYaw_iLimit,
	pidAngleYaw_outputLimit,
	pidRateRoll_kp,
	pidRateRoll_ki,
	pidRateRoll_kd,
	pidRateRoll_iLimit,
	pidRateRoll_outputLimit,
	pidRatePitch_kp,
	pidRatePitch_ki,
	pidRatePitch_kd,
	pidRatePitch_iLimit,
	pidRatePitch_outputLimit,
	pidRateYaw_kp,
	pidRateYaw_ki,
	pidRateYaw_kd,
	pidRateYaw_iLimit,
	pidRateYaw_outputLimit,
	pidPosZ_kp,
	pidPosZ_ki,
	pidPosZ_kd,
	pidPosZ_iLimit,
	pidPosZ_outputLimit,
	pidVelZ_kp,
	pidVelZ_ki,
	pidVelZ_kd,
	pidVelZ_iLimit,
	pidVelZ_outputLimit,
	pidAccZ_kp,
	pidAccZ_ki,
	pidAccZ_kd,
	pidAccZ_iLimit,
	pidAccZ_outputLimit,
	
	para_last 	//һ��Ҫ�����������ͳ��ö�ٸ���
	
}flash_para_list;

//��������
typedef enum{
	PARAM_NONE = 0,
	PARAM_INT8,
	PARAM_INT16,
	PARAM_INT32,
	PARAM_FLOAT,
	PARAM_VECTOR3F,
	PARAM_VECTOR6F,
	PARAM_MATRIX3F
}var_type;



void flash_ext_init(void);
void flash_write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void flash_read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);

void flash_parameter_write(void);
void flash_parameter_read(void);
void flash_write_param(u8* param_value,flash_para_list paralist);
//void flash_read_param_u32(flash_para_list,u32* );
//void flash_write_param_u32(u32 ,flash_para_list );


#endif

