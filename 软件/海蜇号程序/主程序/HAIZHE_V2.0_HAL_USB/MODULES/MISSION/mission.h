#ifndef __MISSION_H
#define __MISSION_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#include <sys.h>
#include "defines.h"
#include "mavlink.h"

extern char mission_file_name[20];


#define LOCATION_DEPTH_MAX_M 83000
#define MISSION_BIN_COMMAND_SIZE 19

// jump command structure
typedef struct PACKED {
  uint16_t target;		  // target command id
  int16_t num_times;	  // num times to repeat.  -1 = repeat forever
} Jump_Command;

// condition delay command structure
typedef struct PACKED  {
  float seconds;		  // period of delay in seconds
}Conditional_Delay_Command;

// condition delay command typedef structure
typedef struct PACKED  {
  float meters; 		  // distance from next waypoint in meters
}Conditional_Distance_Command;

// condition yaw command typedef structure
typedef struct PACKED  {
  float angle_deg;		  // target angle in degrees (0=north, 90=east)
  float turn_rate_dps;	  // turn rate in degrees / second (0=use default)
  int8_t direction; 	  // -1 = ccw, +1 = cw
  uint8_t relative_angle; // 0 = absolute angle, 1 = relative angle
}Yaw_Command;

// change speed command typedef structure
typedef struct PACKED  {
  uint8_t speed_type;	  // 0=airspeed, 1=ground speed
  float target_ms;		  // target speed in m/s, -1 means no change
  float throttle_pct;	  // throttle as a percentage (i.e. 0 ~ 100), -1 means no change
}Change_Speed_Command;

// set relay command typedef structure
typedef struct PACKED  {
  uint8_t num;			  // relay number from 1 to 4
  uint8_t state;		  // on = 3.3V or 5V (depending upon board), off = 0V.	only used for do-set-relay, not for do-repeat-relay
}Set_Relay_Command;

// repeat relay command typedef structure
typedef struct PACKED  {
  uint8_t num;			  // relay number from 1 to 4
  int16_t repeat_count;   // number of times to trigger the relay
  float cycle_time; 	  // cycle time in seconds (the time between peaks or the time the relay is on and off for each cycle?)
}Repeat_Relay_Command;

// set servo command typedef structure
typedef struct PACKED  {
  uint8_t channel;		  // servo channel
  uint16_t pwm; 		  // pwm value for servo
}Set_Servo_Command;

// repeat servo command typedef structure
typedef struct PACKED  {
  uint8_t channel;		  // servo channel
  uint16_t pwm; 		  // pwm value for servo
  int16_t repeat_count;   // number of times to move the servo (returns to trim in between)
  float cycle_time; 	  // cycle time in seconds (the time between peaks or the time the servo is at the specified pwm value for each cycle?)
}Repeat_Servo_Command;

// mount control command typedef structure
typedef struct PACKED  {
  float pitch;			  // pitch angle in degrees
  float roll;			  // roll angle in degrees
  float yaw;			  // yaw angle (relative to vehicle heading) in degrees
}Mount_Control;

// digicam control command typedef structure
typedef struct PACKED  {
  uint8_t shooting_mode;  // ProgramAuto = 1, AV = 2, TV = 3, Man=4, IntelligentAuto=5, SuperiorAuto=6
  uint16_t shutter_speed;
  uint8_t aperture; 	  // F stop number * 10
  uint16_t ISO; 		  // 80, 100, 200, etc
  uint8_t exposure_type;
  uint8_t cmd_id;
  float engine_cutoff_time;   // seconds
}Digicam_Configure;

// digicam control command typedef structure
typedef struct PACKED  {
  uint8_t session;		  // 1 = on, 0 = off
  uint8_t zoom_pos;
  int8_t zoom_step; 	  // +1 = zoom in, -1 = zoom out
  uint8_t focus_lock;
  uint8_t shooting_cmd;
  uint8_t cmd_id;
}Digicam_Control;

// set cam trigger distance command typedef structure
typedef struct PACKED  {
  float meters; 		  // distance
}Cam_Trigg_Distance;

// gripper command typedef structure
typedef struct PACKED  {
  uint8_t num;			  // gripper number
  uint8_t action;		  // action (0 = release, 1 = grab)
}Gripper_Command;

// high altitude balloon altitude wait
typedef struct PACKED  {
  float altitude; // meters
  float descent_rate; // m/s
  uint8_t wiggle_time; // seconds
}Altitude_Wait;

// nav guided command
typedef struct PACKED  {
  // max time is held in p1 field
  float alt_min;		  // min alt below which the command will be aborted.  0 for no lower alt limit
  float alt_max;		  // max alt above which the command will be aborted.  0 for no upper alt limit
  float horiz_max;		  // max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
}Guided_Limits_Command;

// do VTOL transition
typedef struct PACKED  {
  uint8_t target_state;
}Do_VTOL_Transition;

// navigation delay command typedef structure
typedef struct PACKED  {
  float seconds; // period of delay in seconds
  int8_t hour_utc; // absolute time's hour (utc)
  int8_t min_utc; // absolute time's min (utc)
  int8_t sec_utc; // absolute time's sec (utc)
}Navigation_Delay_Command;

// DO_ENGINE_CONTROL support
typedef struct PACKED  {
  bool start_control; // start or stop engine
  bool cold_start; // use cold start procedure
  uint16_t height_delay_cm; // height delay for start
}Do_Engine_Control;

typedef struct PACKED  {
    uint8_t relative_alt : 1;           // 1 if altitude is relateive to home
    uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
    uint8_t origin_alt   : 1;           // this altitude is above ekf origin
    uint8_t loiter_xtrack : 1;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location
}Location_Option_Flags;
typedef struct PACKED  {
    union PACKED{
        Location_Option_Flags flags;                    ///< options bitmask (1<<0 = relative altitude)
        uint8_t options;                                /// allows writing all flags to eeprom as one byte
    };
    // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    float alt;                                     ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
    float lat;                                        ///< param 3 - Latitude * 10**7
    float lng;                                        ///< param 4 - Longitude * 10**7
}Location;

typedef union PACKED  {
  // jump structure
  Jump_Command jump;

  // conditional delay
  Conditional_Delay_Command delay;

  // conditional distance
  Conditional_Distance_Command distance;

  // conditional yaw
  Yaw_Command yaw;

  // change speed
  Change_Speed_Command speed;

  // do-set-relay
  Set_Relay_Command relay;

  // do-repeat-relay
  Repeat_Relay_Command repeat_relay;

  // do-set-servo
  Set_Servo_Command servo;

  // do-repeate-servo
  Repeat_Servo_Command repeat_servo;

  // mount control
  Mount_Control mount_control;

  // camera configure
  Digicam_Configure digicam_configure;

  // camera control
  Digicam_Control digicam_control;

  // cam trigg distance
  Cam_Trigg_Distance cam_trigg_dist;

  // do-gripper
  Gripper_Command gripper;

  // do-guided-limits
  Guided_Limits_Command guided_limits;

  // cam trigg distance
  Altitude_Wait altitude_wait;

  // do vtol transition
  Do_VTOL_Transition do_vtol_transition;

  // DO_ENGINE_CONTROL
  Do_Engine_Control do_engine_control;
  
  // location
  Location location;	  // Waypoint location

  // navigation delay
  Navigation_Delay_Command nav_delay;

  // raw bytes, for reading/writing to eeprom. Note that only 10 bytes are available
  // if a 16 bit command ID is used
  uint8_t bytes[12];
}Content;

// command structure
typedef struct  {
    uint16_t index;             // this commands position in the command list
    uint16_t id;                // mavlink command id
    uint16_t p1;                // general purpose parameter 1
    Content content;
}Mission_Command;
extern Mission_Command _nav_cmd;	//正在运行的当前航点

typedef struct{
	bool reached_destination	: 1;
	bool fast_waypoint			: 1;
	bool slowing_down			: 1;
	bool recalc_wp_leash		: 1;
}FLAG;

typedef struct{
	FLAG flag;
	Location home;	//返航点
	Location destination; //下个航点
	Location origin;		//正在进行的航迹的起始航点
}WP_NAV;
extern WP_NAV wp_nav;

void mission_cmd_to_bin(const Mission_Command *cmd);
bool mission_cmd_from_bin(u16 index, Mission_Command *cmd);
MAV_MISSION_RESULT mavlink_to_mission_cmd(const mavlink_mission_item_t packet, Mission_Command *cmd);
MAV_MISSION_RESULT mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t packet, Mission_Command *cmd);
bool mission_cmd_to_mavlink(const Mission_Command *cmd, mavlink_mission_item_t *packet);
bool mission_cmd_to_mavlink_int(const Mission_Command *cmd, mavlink_mission_item_int_t *packet);
void mission_update(void);
bool mission_start(void);
void mission_init(void);
bool mission_reset(void);


#ifdef __cplusplus
}
#endif
   
#endif /*__LOGGER_H*/
