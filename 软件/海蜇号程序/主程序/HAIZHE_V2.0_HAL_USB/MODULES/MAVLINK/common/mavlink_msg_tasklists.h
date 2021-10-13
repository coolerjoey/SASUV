#pragma once
// MESSAGE TASKLISTS PACKING

#define MAVLINK_MSG_ID_TASKLISTS 12

MAVPACKED(
typedef struct __mavlink_tasklists_t {
 int16_t task_1_value; /*< task 1 value*/
 int16_t task_2_value; /*< task 2 value*/
 int16_t task_3_value; /*< task 3 value*/
 int16_t task_4_value; /*< task 4 value*/
 int16_t task_5_value; /*< task 5 value*/
 int16_t task_6_value; /*< task 6 value*/
 int16_t task_7_value; /*< task 7 value*/
 int16_t task_8_value; /*< task 8 value*/
 uint8_t task_1; /*< task 1*/
 uint8_t task_1_duration; /*< task 1 duration*/
 uint8_t task_2; /*< task 2*/
 uint8_t task_2_duration; /*< task 2 duration*/
 uint8_t task_3; /*< task 3*/
 uint8_t task_3_duration; /*< task 3 duration*/
 uint8_t task_4; /*< task 4*/
 uint8_t task_4_duration; /*< task 4 duration*/
 uint8_t task_5; /*< task 5*/
 uint8_t task_5_duration; /*< task 5 duration*/
 uint8_t task_6; /*< task 6*/
 uint8_t task_6_duration; /*< task 6 duration*/
 uint8_t task_7; /*< task 7*/
 uint8_t task_7_duration; /*< task 7 duration*/
 uint8_t task_8; /*< task 8*/
 uint8_t task_8_duration; /*< task 8 duration*/
}) mavlink_tasklists_t;

#define MAVLINK_MSG_ID_TASKLISTS_LEN 32
#define MAVLINK_MSG_ID_TASKLISTS_MIN_LEN 32
#define MAVLINK_MSG_ID_12_LEN 32
#define MAVLINK_MSG_ID_12_MIN_LEN 32

#define MAVLINK_MSG_ID_TASKLISTS_CRC 153
#define MAVLINK_MSG_ID_12_CRC 153



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASKLISTS { \
    12, \
    "TASKLISTS", \
    24, \
    {  { "task_1_value", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_tasklists_t, task_1_value) }, \
         { "task_2_value", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_tasklists_t, task_2_value) }, \
         { "task_3_value", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_tasklists_t, task_3_value) }, \
         { "task_4_value", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_tasklists_t, task_4_value) }, \
         { "task_5_value", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_tasklists_t, task_5_value) }, \
         { "task_6_value", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_tasklists_t, task_6_value) }, \
         { "task_7_value", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_tasklists_t, task_7_value) }, \
         { "task_8_value", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_tasklists_t, task_8_value) }, \
         { "task_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_tasklists_t, task_1) }, \
         { "task_1_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_tasklists_t, task_1_duration) }, \
         { "task_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_tasklists_t, task_2) }, \
         { "task_2_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_tasklists_t, task_2_duration) }, \
         { "task_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_tasklists_t, task_3) }, \
         { "task_3_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_tasklists_t, task_3_duration) }, \
         { "task_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_tasklists_t, task_4) }, \
         { "task_4_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_tasklists_t, task_4_duration) }, \
         { "task_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_tasklists_t, task_5) }, \
         { "task_5_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_tasklists_t, task_5_duration) }, \
         { "task_6", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_tasklists_t, task_6) }, \
         { "task_6_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_tasklists_t, task_6_duration) }, \
         { "task_7", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_tasklists_t, task_7) }, \
         { "task_7_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_tasklists_t, task_7_duration) }, \
         { "task_8", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_tasklists_t, task_8) }, \
         { "task_8_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_tasklists_t, task_8_duration) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASKLISTS { \
    "TASKLISTS", \
    24, \
    {  { "task_1_value", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_tasklists_t, task_1_value) }, \
         { "task_2_value", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_tasklists_t, task_2_value) }, \
         { "task_3_value", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_tasklists_t, task_3_value) }, \
         { "task_4_value", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_tasklists_t, task_4_value) }, \
         { "task_5_value", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_tasklists_t, task_5_value) }, \
         { "task_6_value", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_tasklists_t, task_6_value) }, \
         { "task_7_value", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_tasklists_t, task_7_value) }, \
         { "task_8_value", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_tasklists_t, task_8_value) }, \
         { "task_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_tasklists_t, task_1) }, \
         { "task_1_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_tasklists_t, task_1_duration) }, \
         { "task_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_tasklists_t, task_2) }, \
         { "task_2_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_tasklists_t, task_2_duration) }, \
         { "task_3", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_tasklists_t, task_3) }, \
         { "task_3_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_tasklists_t, task_3_duration) }, \
         { "task_4", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_tasklists_t, task_4) }, \
         { "task_4_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_tasklists_t, task_4_duration) }, \
         { "task_5", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_tasklists_t, task_5) }, \
         { "task_5_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_tasklists_t, task_5_duration) }, \
         { "task_6", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_tasklists_t, task_6) }, \
         { "task_6_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_tasklists_t, task_6_duration) }, \
         { "task_7", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_tasklists_t, task_7) }, \
         { "task_7_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_tasklists_t, task_7_duration) }, \
         { "task_8", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_tasklists_t, task_8) }, \
         { "task_8_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_tasklists_t, task_8_duration) }, \
         } \
}
#endif

/**
 * @brief Pack a tasklists message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param task_1 task 1
 * @param task_1_duration task 1 duration
 * @param task_1_value task 1 value
 * @param task_2 task 2
 * @param task_2_duration task 2 duration
 * @param task_2_value task 2 value
 * @param task_3 task 3
 * @param task_3_duration task 3 duration
 * @param task_3_value task 3 value
 * @param task_4 task 4
 * @param task_4_duration task 4 duration
 * @param task_4_value task 4 value
 * @param task_5 task 5
 * @param task_5_duration task 5 duration
 * @param task_5_value task 5 value
 * @param task_6 task 6
 * @param task_6_duration task 6 duration
 * @param task_6_value task 6 value
 * @param task_7 task 7
 * @param task_7_duration task 7 duration
 * @param task_7_value task 7 value
 * @param task_8 task 8
 * @param task_8_duration task 8 duration
 * @param task_8_value task 8 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tasklists_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t task_1, uint8_t task_1_duration, int16_t task_1_value, uint8_t task_2, uint8_t task_2_duration, int16_t task_2_value, uint8_t task_3, uint8_t task_3_duration, int16_t task_3_value, uint8_t task_4, uint8_t task_4_duration, int16_t task_4_value, uint8_t task_5, uint8_t task_5_duration, int16_t task_5_value, uint8_t task_6, uint8_t task_6_duration, int16_t task_6_value, uint8_t task_7, uint8_t task_7_duration, int16_t task_7_value, uint8_t task_8, uint8_t task_8_duration, int16_t task_8_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASKLISTS_LEN];
    _mav_put_int16_t(buf, 0, task_1_value);
    _mav_put_int16_t(buf, 2, task_2_value);
    _mav_put_int16_t(buf, 4, task_3_value);
    _mav_put_int16_t(buf, 6, task_4_value);
    _mav_put_int16_t(buf, 8, task_5_value);
    _mav_put_int16_t(buf, 10, task_6_value);
    _mav_put_int16_t(buf, 12, task_7_value);
    _mav_put_int16_t(buf, 14, task_8_value);
    _mav_put_uint8_t(buf, 16, task_1);
    _mav_put_uint8_t(buf, 17, task_1_duration);
    _mav_put_uint8_t(buf, 18, task_2);
    _mav_put_uint8_t(buf, 19, task_2_duration);
    _mav_put_uint8_t(buf, 20, task_3);
    _mav_put_uint8_t(buf, 21, task_3_duration);
    _mav_put_uint8_t(buf, 22, task_4);
    _mav_put_uint8_t(buf, 23, task_4_duration);
    _mav_put_uint8_t(buf, 24, task_5);
    _mav_put_uint8_t(buf, 25, task_5_duration);
    _mav_put_uint8_t(buf, 26, task_6);
    _mav_put_uint8_t(buf, 27, task_6_duration);
    _mav_put_uint8_t(buf, 28, task_7);
    _mav_put_uint8_t(buf, 29, task_7_duration);
    _mav_put_uint8_t(buf, 30, task_8);
    _mav_put_uint8_t(buf, 31, task_8_duration);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASKLISTS_LEN);
#else
    mavlink_tasklists_t packet;
    packet.task_1_value = task_1_value;
    packet.task_2_value = task_2_value;
    packet.task_3_value = task_3_value;
    packet.task_4_value = task_4_value;
    packet.task_5_value = task_5_value;
    packet.task_6_value = task_6_value;
    packet.task_7_value = task_7_value;
    packet.task_8_value = task_8_value;
    packet.task_1 = task_1;
    packet.task_1_duration = task_1_duration;
    packet.task_2 = task_2;
    packet.task_2_duration = task_2_duration;
    packet.task_3 = task_3;
    packet.task_3_duration = task_3_duration;
    packet.task_4 = task_4;
    packet.task_4_duration = task_4_duration;
    packet.task_5 = task_5;
    packet.task_5_duration = task_5_duration;
    packet.task_6 = task_6;
    packet.task_6_duration = task_6_duration;
    packet.task_7 = task_7;
    packet.task_7_duration = task_7_duration;
    packet.task_8 = task_8;
    packet.task_8_duration = task_8_duration;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASKLISTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TASKLISTS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TASKLISTS_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_LEN, MAVLINK_MSG_ID_TASKLISTS_CRC);
}

/**
 * @brief Pack a tasklists message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param task_1 task 1
 * @param task_1_duration task 1 duration
 * @param task_1_value task 1 value
 * @param task_2 task 2
 * @param task_2_duration task 2 duration
 * @param task_2_value task 2 value
 * @param task_3 task 3
 * @param task_3_duration task 3 duration
 * @param task_3_value task 3 value
 * @param task_4 task 4
 * @param task_4_duration task 4 duration
 * @param task_4_value task 4 value
 * @param task_5 task 5
 * @param task_5_duration task 5 duration
 * @param task_5_value task 5 value
 * @param task_6 task 6
 * @param task_6_duration task 6 duration
 * @param task_6_value task 6 value
 * @param task_7 task 7
 * @param task_7_duration task 7 duration
 * @param task_7_value task 7 value
 * @param task_8 task 8
 * @param task_8_duration task 8 duration
 * @param task_8_value task 8 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tasklists_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t task_1,uint8_t task_1_duration,int16_t task_1_value,uint8_t task_2,uint8_t task_2_duration,int16_t task_2_value,uint8_t task_3,uint8_t task_3_duration,int16_t task_3_value,uint8_t task_4,uint8_t task_4_duration,int16_t task_4_value,uint8_t task_5,uint8_t task_5_duration,int16_t task_5_value,uint8_t task_6,uint8_t task_6_duration,int16_t task_6_value,uint8_t task_7,uint8_t task_7_duration,int16_t task_7_value,uint8_t task_8,uint8_t task_8_duration,int16_t task_8_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASKLISTS_LEN];
    _mav_put_int16_t(buf, 0, task_1_value);
    _mav_put_int16_t(buf, 2, task_2_value);
    _mav_put_int16_t(buf, 4, task_3_value);
    _mav_put_int16_t(buf, 6, task_4_value);
    _mav_put_int16_t(buf, 8, task_5_value);
    _mav_put_int16_t(buf, 10, task_6_value);
    _mav_put_int16_t(buf, 12, task_7_value);
    _mav_put_int16_t(buf, 14, task_8_value);
    _mav_put_uint8_t(buf, 16, task_1);
    _mav_put_uint8_t(buf, 17, task_1_duration);
    _mav_put_uint8_t(buf, 18, task_2);
    _mav_put_uint8_t(buf, 19, task_2_duration);
    _mav_put_uint8_t(buf, 20, task_3);
    _mav_put_uint8_t(buf, 21, task_3_duration);
    _mav_put_uint8_t(buf, 22, task_4);
    _mav_put_uint8_t(buf, 23, task_4_duration);
    _mav_put_uint8_t(buf, 24, task_5);
    _mav_put_uint8_t(buf, 25, task_5_duration);
    _mav_put_uint8_t(buf, 26, task_6);
    _mav_put_uint8_t(buf, 27, task_6_duration);
    _mav_put_uint8_t(buf, 28, task_7);
    _mav_put_uint8_t(buf, 29, task_7_duration);
    _mav_put_uint8_t(buf, 30, task_8);
    _mav_put_uint8_t(buf, 31, task_8_duration);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASKLISTS_LEN);
#else
    mavlink_tasklists_t packet;
    packet.task_1_value = task_1_value;
    packet.task_2_value = task_2_value;
    packet.task_3_value = task_3_value;
    packet.task_4_value = task_4_value;
    packet.task_5_value = task_5_value;
    packet.task_6_value = task_6_value;
    packet.task_7_value = task_7_value;
    packet.task_8_value = task_8_value;
    packet.task_1 = task_1;
    packet.task_1_duration = task_1_duration;
    packet.task_2 = task_2;
    packet.task_2_duration = task_2_duration;
    packet.task_3 = task_3;
    packet.task_3_duration = task_3_duration;
    packet.task_4 = task_4;
    packet.task_4_duration = task_4_duration;
    packet.task_5 = task_5;
    packet.task_5_duration = task_5_duration;
    packet.task_6 = task_6;
    packet.task_6_duration = task_6_duration;
    packet.task_7 = task_7;
    packet.task_7_duration = task_7_duration;
    packet.task_8 = task_8;
    packet.task_8_duration = task_8_duration;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASKLISTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TASKLISTS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TASKLISTS_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_LEN, MAVLINK_MSG_ID_TASKLISTS_CRC);
}

/**
 * @brief Encode a tasklists struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tasklists C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tasklists_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tasklists_t* tasklists)
{
    return mavlink_msg_tasklists_pack(system_id, component_id, msg, tasklists->task_1, tasklists->task_1_duration, tasklists->task_1_value, tasklists->task_2, tasklists->task_2_duration, tasklists->task_2_value, tasklists->task_3, tasklists->task_3_duration, tasklists->task_3_value, tasklists->task_4, tasklists->task_4_duration, tasklists->task_4_value, tasklists->task_5, tasklists->task_5_duration, tasklists->task_5_value, tasklists->task_6, tasklists->task_6_duration, tasklists->task_6_value, tasklists->task_7, tasklists->task_7_duration, tasklists->task_7_value, tasklists->task_8, tasklists->task_8_duration, tasklists->task_8_value);
}

/**
 * @brief Encode a tasklists struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tasklists C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tasklists_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tasklists_t* tasklists)
{
    return mavlink_msg_tasklists_pack_chan(system_id, component_id, chan, msg, tasklists->task_1, tasklists->task_1_duration, tasklists->task_1_value, tasklists->task_2, tasklists->task_2_duration, tasklists->task_2_value, tasklists->task_3, tasklists->task_3_duration, tasklists->task_3_value, tasklists->task_4, tasklists->task_4_duration, tasklists->task_4_value, tasklists->task_5, tasklists->task_5_duration, tasklists->task_5_value, tasklists->task_6, tasklists->task_6_duration, tasklists->task_6_value, tasklists->task_7, tasklists->task_7_duration, tasklists->task_7_value, tasklists->task_8, tasklists->task_8_duration, tasklists->task_8_value);
}

/**
 * @brief Send a tasklists message
 * @param chan MAVLink channel to send the message
 *
 * @param task_1 task 1
 * @param task_1_duration task 1 duration
 * @param task_1_value task 1 value
 * @param task_2 task 2
 * @param task_2_duration task 2 duration
 * @param task_2_value task 2 value
 * @param task_3 task 3
 * @param task_3_duration task 3 duration
 * @param task_3_value task 3 value
 * @param task_4 task 4
 * @param task_4_duration task 4 duration
 * @param task_4_value task 4 value
 * @param task_5 task 5
 * @param task_5_duration task 5 duration
 * @param task_5_value task 5 value
 * @param task_6 task 6
 * @param task_6_duration task 6 duration
 * @param task_6_value task 6 value
 * @param task_7 task 7
 * @param task_7_duration task 7 duration
 * @param task_7_value task 7 value
 * @param task_8 task 8
 * @param task_8_duration task 8 duration
 * @param task_8_value task 8 value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tasklists_send(mavlink_channel_t chan, uint8_t task_1, uint8_t task_1_duration, int16_t task_1_value, uint8_t task_2, uint8_t task_2_duration, int16_t task_2_value, uint8_t task_3, uint8_t task_3_duration, int16_t task_3_value, uint8_t task_4, uint8_t task_4_duration, int16_t task_4_value, uint8_t task_5, uint8_t task_5_duration, int16_t task_5_value, uint8_t task_6, uint8_t task_6_duration, int16_t task_6_value, uint8_t task_7, uint8_t task_7_duration, int16_t task_7_value, uint8_t task_8, uint8_t task_8_duration, int16_t task_8_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASKLISTS_LEN];
    _mav_put_int16_t(buf, 0, task_1_value);
    _mav_put_int16_t(buf, 2, task_2_value);
    _mav_put_int16_t(buf, 4, task_3_value);
    _mav_put_int16_t(buf, 6, task_4_value);
    _mav_put_int16_t(buf, 8, task_5_value);
    _mav_put_int16_t(buf, 10, task_6_value);
    _mav_put_int16_t(buf, 12, task_7_value);
    _mav_put_int16_t(buf, 14, task_8_value);
    _mav_put_uint8_t(buf, 16, task_1);
    _mav_put_uint8_t(buf, 17, task_1_duration);
    _mav_put_uint8_t(buf, 18, task_2);
    _mav_put_uint8_t(buf, 19, task_2_duration);
    _mav_put_uint8_t(buf, 20, task_3);
    _mav_put_uint8_t(buf, 21, task_3_duration);
    _mav_put_uint8_t(buf, 22, task_4);
    _mav_put_uint8_t(buf, 23, task_4_duration);
    _mav_put_uint8_t(buf, 24, task_5);
    _mav_put_uint8_t(buf, 25, task_5_duration);
    _mav_put_uint8_t(buf, 26, task_6);
    _mav_put_uint8_t(buf, 27, task_6_duration);
    _mav_put_uint8_t(buf, 28, task_7);
    _mav_put_uint8_t(buf, 29, task_7_duration);
    _mav_put_uint8_t(buf, 30, task_8);
    _mav_put_uint8_t(buf, 31, task_8_duration);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS, buf, MAVLINK_MSG_ID_TASKLISTS_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_LEN, MAVLINK_MSG_ID_TASKLISTS_CRC);
#else
    mavlink_tasklists_t packet;
    packet.task_1_value = task_1_value;
    packet.task_2_value = task_2_value;
    packet.task_3_value = task_3_value;
    packet.task_4_value = task_4_value;
    packet.task_5_value = task_5_value;
    packet.task_6_value = task_6_value;
    packet.task_7_value = task_7_value;
    packet.task_8_value = task_8_value;
    packet.task_1 = task_1;
    packet.task_1_duration = task_1_duration;
    packet.task_2 = task_2;
    packet.task_2_duration = task_2_duration;
    packet.task_3 = task_3;
    packet.task_3_duration = task_3_duration;
    packet.task_4 = task_4;
    packet.task_4_duration = task_4_duration;
    packet.task_5 = task_5;
    packet.task_5_duration = task_5_duration;
    packet.task_6 = task_6;
    packet.task_6_duration = task_6_duration;
    packet.task_7 = task_7;
    packet.task_7_duration = task_7_duration;
    packet.task_8 = task_8;
    packet.task_8_duration = task_8_duration;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS, (const char *)&packet, MAVLINK_MSG_ID_TASKLISTS_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_LEN, MAVLINK_MSG_ID_TASKLISTS_CRC);
#endif
}

/**
 * @brief Send a tasklists message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tasklists_send_struct(mavlink_channel_t chan, const mavlink_tasklists_t* tasklists)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tasklists_send(chan, tasklists->task_1, tasklists->task_1_duration, tasklists->task_1_value, tasklists->task_2, tasklists->task_2_duration, tasklists->task_2_value, tasklists->task_3, tasklists->task_3_duration, tasklists->task_3_value, tasklists->task_4, tasklists->task_4_duration, tasklists->task_4_value, tasklists->task_5, tasklists->task_5_duration, tasklists->task_5_value, tasklists->task_6, tasklists->task_6_duration, tasklists->task_6_value, tasklists->task_7, tasklists->task_7_duration, tasklists->task_7_value, tasklists->task_8, tasklists->task_8_duration, tasklists->task_8_value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS, (const char *)tasklists, MAVLINK_MSG_ID_TASKLISTS_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_LEN, MAVLINK_MSG_ID_TASKLISTS_CRC);
#endif
}

#if MAVLINK_MSG_ID_TASKLISTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tasklists_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t task_1, uint8_t task_1_duration, int16_t task_1_value, uint8_t task_2, uint8_t task_2_duration, int16_t task_2_value, uint8_t task_3, uint8_t task_3_duration, int16_t task_3_value, uint8_t task_4, uint8_t task_4_duration, int16_t task_4_value, uint8_t task_5, uint8_t task_5_duration, int16_t task_5_value, uint8_t task_6, uint8_t task_6_duration, int16_t task_6_value, uint8_t task_7, uint8_t task_7_duration, int16_t task_7_value, uint8_t task_8, uint8_t task_8_duration, int16_t task_8_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, task_1_value);
    _mav_put_int16_t(buf, 2, task_2_value);
    _mav_put_int16_t(buf, 4, task_3_value);
    _mav_put_int16_t(buf, 6, task_4_value);
    _mav_put_int16_t(buf, 8, task_5_value);
    _mav_put_int16_t(buf, 10, task_6_value);
    _mav_put_int16_t(buf, 12, task_7_value);
    _mav_put_int16_t(buf, 14, task_8_value);
    _mav_put_uint8_t(buf, 16, task_1);
    _mav_put_uint8_t(buf, 17, task_1_duration);
    _mav_put_uint8_t(buf, 18, task_2);
    _mav_put_uint8_t(buf, 19, task_2_duration);
    _mav_put_uint8_t(buf, 20, task_3);
    _mav_put_uint8_t(buf, 21, task_3_duration);
    _mav_put_uint8_t(buf, 22, task_4);
    _mav_put_uint8_t(buf, 23, task_4_duration);
    _mav_put_uint8_t(buf, 24, task_5);
    _mav_put_uint8_t(buf, 25, task_5_duration);
    _mav_put_uint8_t(buf, 26, task_6);
    _mav_put_uint8_t(buf, 27, task_6_duration);
    _mav_put_uint8_t(buf, 28, task_7);
    _mav_put_uint8_t(buf, 29, task_7_duration);
    _mav_put_uint8_t(buf, 30, task_8);
    _mav_put_uint8_t(buf, 31, task_8_duration);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS, buf, MAVLINK_MSG_ID_TASKLISTS_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_LEN, MAVLINK_MSG_ID_TASKLISTS_CRC);
#else
    mavlink_tasklists_t *packet = (mavlink_tasklists_t *)msgbuf;
    packet->task_1_value = task_1_value;
    packet->task_2_value = task_2_value;
    packet->task_3_value = task_3_value;
    packet->task_4_value = task_4_value;
    packet->task_5_value = task_5_value;
    packet->task_6_value = task_6_value;
    packet->task_7_value = task_7_value;
    packet->task_8_value = task_8_value;
    packet->task_1 = task_1;
    packet->task_1_duration = task_1_duration;
    packet->task_2 = task_2;
    packet->task_2_duration = task_2_duration;
    packet->task_3 = task_3;
    packet->task_3_duration = task_3_duration;
    packet->task_4 = task_4;
    packet->task_4_duration = task_4_duration;
    packet->task_5 = task_5;
    packet->task_5_duration = task_5_duration;
    packet->task_6 = task_6;
    packet->task_6_duration = task_6_duration;
    packet->task_7 = task_7;
    packet->task_7_duration = task_7_duration;
    packet->task_8 = task_8;
    packet->task_8_duration = task_8_duration;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS, (const char *)packet, MAVLINK_MSG_ID_TASKLISTS_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_LEN, MAVLINK_MSG_ID_TASKLISTS_CRC);
#endif
}
#endif

#endif

// MESSAGE TASKLISTS UNPACKING


/**
 * @brief Get field task_1 from tasklists message
 *
 * @return task 1
 */
static inline uint8_t mavlink_msg_tasklists_get_task_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field task_1_duration from tasklists message
 *
 * @return task 1 duration
 */
static inline uint8_t mavlink_msg_tasklists_get_task_1_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field task_1_value from tasklists message
 *
 * @return task 1 value
 */
static inline int16_t mavlink_msg_tasklists_get_task_1_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field task_2 from tasklists message
 *
 * @return task 2
 */
static inline uint8_t mavlink_msg_tasklists_get_task_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field task_2_duration from tasklists message
 *
 * @return task 2 duration
 */
static inline uint8_t mavlink_msg_tasklists_get_task_2_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field task_2_value from tasklists message
 *
 * @return task 2 value
 */
static inline int16_t mavlink_msg_tasklists_get_task_2_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field task_3 from tasklists message
 *
 * @return task 3
 */
static inline uint8_t mavlink_msg_tasklists_get_task_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field task_3_duration from tasklists message
 *
 * @return task 3 duration
 */
static inline uint8_t mavlink_msg_tasklists_get_task_3_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field task_3_value from tasklists message
 *
 * @return task 3 value
 */
static inline int16_t mavlink_msg_tasklists_get_task_3_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field task_4 from tasklists message
 *
 * @return task 4
 */
static inline uint8_t mavlink_msg_tasklists_get_task_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field task_4_duration from tasklists message
 *
 * @return task 4 duration
 */
static inline uint8_t mavlink_msg_tasklists_get_task_4_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field task_4_value from tasklists message
 *
 * @return task 4 value
 */
static inline int16_t mavlink_msg_tasklists_get_task_4_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field task_5 from tasklists message
 *
 * @return task 5
 */
static inline uint8_t mavlink_msg_tasklists_get_task_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field task_5_duration from tasklists message
 *
 * @return task 5 duration
 */
static inline uint8_t mavlink_msg_tasklists_get_task_5_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field task_5_value from tasklists message
 *
 * @return task 5 value
 */
static inline int16_t mavlink_msg_tasklists_get_task_5_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field task_6 from tasklists message
 *
 * @return task 6
 */
static inline uint8_t mavlink_msg_tasklists_get_task_6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field task_6_duration from tasklists message
 *
 * @return task 6 duration
 */
static inline uint8_t mavlink_msg_tasklists_get_task_6_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field task_6_value from tasklists message
 *
 * @return task 6 value
 */
static inline int16_t mavlink_msg_tasklists_get_task_6_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field task_7 from tasklists message
 *
 * @return task 7
 */
static inline uint8_t mavlink_msg_tasklists_get_task_7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field task_7_duration from tasklists message
 *
 * @return task 7 duration
 */
static inline uint8_t mavlink_msg_tasklists_get_task_7_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field task_7_value from tasklists message
 *
 * @return task 7 value
 */
static inline int16_t mavlink_msg_tasklists_get_task_7_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field task_8 from tasklists message
 *
 * @return task 8
 */
static inline uint8_t mavlink_msg_tasklists_get_task_8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field task_8_duration from tasklists message
 *
 * @return task 8 duration
 */
static inline uint8_t mavlink_msg_tasklists_get_task_8_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field task_8_value from tasklists message
 *
 * @return task 8 value
 */
static inline int16_t mavlink_msg_tasklists_get_task_8_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Decode a tasklists message into a struct
 *
 * @param msg The message to decode
 * @param tasklists C-struct to decode the message contents into
 */
static inline void mavlink_msg_tasklists_decode(const mavlink_message_t* msg, mavlink_tasklists_t* tasklists)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tasklists->task_1_value = mavlink_msg_tasklists_get_task_1_value(msg);
    tasklists->task_2_value = mavlink_msg_tasklists_get_task_2_value(msg);
    tasklists->task_3_value = mavlink_msg_tasklists_get_task_3_value(msg);
    tasklists->task_4_value = mavlink_msg_tasklists_get_task_4_value(msg);
    tasklists->task_5_value = mavlink_msg_tasklists_get_task_5_value(msg);
    tasklists->task_6_value = mavlink_msg_tasklists_get_task_6_value(msg);
    tasklists->task_7_value = mavlink_msg_tasklists_get_task_7_value(msg);
    tasklists->task_8_value = mavlink_msg_tasklists_get_task_8_value(msg);
    tasklists->task_1 = mavlink_msg_tasklists_get_task_1(msg);
    tasklists->task_1_duration = mavlink_msg_tasklists_get_task_1_duration(msg);
    tasklists->task_2 = mavlink_msg_tasklists_get_task_2(msg);
    tasklists->task_2_duration = mavlink_msg_tasklists_get_task_2_duration(msg);
    tasklists->task_3 = mavlink_msg_tasklists_get_task_3(msg);
    tasklists->task_3_duration = mavlink_msg_tasklists_get_task_3_duration(msg);
    tasklists->task_4 = mavlink_msg_tasklists_get_task_4(msg);
    tasklists->task_4_duration = mavlink_msg_tasklists_get_task_4_duration(msg);
    tasklists->task_5 = mavlink_msg_tasklists_get_task_5(msg);
    tasklists->task_5_duration = mavlink_msg_tasklists_get_task_5_duration(msg);
    tasklists->task_6 = mavlink_msg_tasklists_get_task_6(msg);
    tasklists->task_6_duration = mavlink_msg_tasklists_get_task_6_duration(msg);
    tasklists->task_7 = mavlink_msg_tasklists_get_task_7(msg);
    tasklists->task_7_duration = mavlink_msg_tasklists_get_task_7_duration(msg);
    tasklists->task_8 = mavlink_msg_tasklists_get_task_8(msg);
    tasklists->task_8_duration = mavlink_msg_tasklists_get_task_8_duration(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASKLISTS_LEN? msg->len : MAVLINK_MSG_ID_TASKLISTS_LEN;
        memset(tasklists, 0, MAVLINK_MSG_ID_TASKLISTS_LEN);
    memcpy(tasklists, _MAV_PAYLOAD(msg), len);
#endif
}
