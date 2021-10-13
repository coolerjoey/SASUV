#ifndef _JOYSTICK_H
#define _JOYSTICK_H

#include "sys.h"

//��������
typedef enum {
    k_none                  = 0,            ///< disabled
    k_shift                 = 1,            ///< "shift" buttons to allow more functions
    k_arm_toggle            = 2,            ///< arm/disarm vehicle toggle �����л�
    k_arm                   = 3,            ///< arm vehicle
    k_disarm                = 4,            ///< disarm vehicle

    k_mode_manual           = 5,            ///< enter enter manual mode
    k_mode_stabilize        = 6,            ///< enter stabilize mode
    k_mode_depth_hold       = 7,            ///< enter depth hold mode
    k_mode_poshold          = 8,            ///< enter poshold mode
    k_mode_auto             = 9,            ///< enter auto mode
    k_mode_circle           = 10,           ///< enter circle mode
    k_mode_guided           = 11,           ///< enter guided mode
    k_mode_rtl              = 12,           //����ģʽ

	k_speed_gain_inc		= 18,			//������������
	k_speed_gain_dec		= 19,			//���������С

	k_baro_vibration		= 20,			//��ȼ�У׼
	k_acc_vibration			= 21,			//���ٶȼ�У׼
	k_return_boot			= 22,			//�ص�bootloader
	k_att_toggle			= 23,			//�л�ˮƽ��̬��Դ
	k_follow_toggle			= 24,			//stabilize��ģʽ���涯�л�
	
    k_gain_toggle           = 41,           ///< toggle different gain settings
    k_gain_inc              = 42,           ///< increase control gain ң�ص������
    k_gain_dec              = 43,           ///< decrease control gain
    k_trim_roll_inc         = 44,           ///< increase roll trim
    k_trim_roll_dec         = 45,           ///< decrease roll trim
    k_trim_pitch_inc        = 46,           ///< increase pitch trim
    k_trim_pitch_dec        = 47,           ///< decrease pitch trim
    k_input_hold_set        = 48,           ///< toggle input hold (trim to current controls)
    k_roll_pitch_toggle     = 49,           ///< adjust roll/pitch input instead of forward/lateral

    // 50 reserved for future function

    k_relay_1_on            = 51,           ///< trigger relay on
    k_relay_1_off           = 52,           ///< trigger relay off
    k_relay_1_toggle        = 53,           ///< trigger relay toggle
    k_relay_2_on            = 54,           ///< trigger relay on
    k_relay_2_off           = 55,           ///< trigger relay off
    k_relay_2_toggle        = 56,           ///< trigger relay toggle
	
    k_custom_1              = 91,           ///< custom user button 1
    k_custom_2              = 92,           ///< custom user button 2
    k_custom_3              = 93,           ///< custom user button 3
    k_custom_4              = 94,           ///< custom user button 4
    k_custom_5              = 95,           ///< custom user button 5
    k_custom_6              = 96,           ///< custom user button 6

    // 108+ reserved for future functions
    k_nr_btn_functions         ///< This must be the last enum value (only add new values _before_ this one)
} button_function_t;


typedef struct{
	int16_t left_ud;	//��ҡ������
	int16_t left_lr;	//��ҡ������
	int16_t right_ud;	//��ҡ������
	int16_t right_lr;	//��ҡ������
	uint16_t button;	//��ť
}JOYSTICK;
extern JOYSTICK joystic ;

void handle_joystick(s16 x, s16 y, s16 z, s16 r, u16 buttons);
void handle_jsbutton_press(u8 button, bool shift, bool held);
void handle_jsbutton_release(u8 button);
void joystick_init(void);

#endif
