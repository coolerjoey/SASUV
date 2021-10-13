#pragma once
// MESSAGE TASKLISTS_ABOUT PACKING

#define MAVLINK_MSG_ID_TASKLISTS_ABOUT 13

MAVPACKED(
typedef struct __mavlink_tasklists_about_t {
 uint8_t task_run; /*< run tasklists*/
 uint8_t task_return; /*< return tasklists */
}) mavlink_tasklists_about_t;

#define MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN 2
#define MAVLINK_MSG_ID_TASKLISTS_ABOUT_MIN_LEN 2
#define MAVLINK_MSG_ID_13_LEN 2
#define MAVLINK_MSG_ID_13_MIN_LEN 2

#define MAVLINK_MSG_ID_TASKLISTS_ABOUT_CRC 152
#define MAVLINK_MSG_ID_13_CRC 152



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASKLISTS_ABOUT { \
    13, \
    "TASKLISTS_ABOUT", \
    2, \
    {  { "task_run", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_tasklists_about_t, task_run) }, \
         { "task_return", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_tasklists_about_t, task_return) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASKLISTS_ABOUT { \
    "TASKLISTS_ABOUT", \
    2, \
    {  { "task_run", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_tasklists_about_t, task_run) }, \
         { "task_return", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_tasklists_about_t, task_return) }, \
         } \
}
#endif

/**
 * @brief Pack a tasklists_about message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param task_run run tasklists
 * @param task_return return tasklists 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tasklists_about_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t task_run, uint8_t task_return)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN];
    _mav_put_uint8_t(buf, 0, task_run);
    _mav_put_uint8_t(buf, 1, task_return);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN);
#else
    mavlink_tasklists_about_t packet;
    packet.task_run = task_run;
    packet.task_return = task_return;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TASKLISTS_ABOUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TASKLISTS_ABOUT_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_CRC);
}

/**
 * @brief Pack a tasklists_about message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param task_run run tasklists
 * @param task_return return tasklists 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tasklists_about_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t task_run,uint8_t task_return)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN];
    _mav_put_uint8_t(buf, 0, task_run);
    _mav_put_uint8_t(buf, 1, task_return);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN);
#else
    mavlink_tasklists_about_t packet;
    packet.task_run = task_run;
    packet.task_return = task_return;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TASKLISTS_ABOUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TASKLISTS_ABOUT_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_CRC);
}

/**
 * @brief Encode a tasklists_about struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tasklists_about C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tasklists_about_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tasklists_about_t* tasklists_about)
{
    return mavlink_msg_tasklists_about_pack(system_id, component_id, msg, tasklists_about->task_run, tasklists_about->task_return);
}

/**
 * @brief Encode a tasklists_about struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tasklists_about C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tasklists_about_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tasklists_about_t* tasklists_about)
{
    return mavlink_msg_tasklists_about_pack_chan(system_id, component_id, chan, msg, tasklists_about->task_run, tasklists_about->task_return);
}

/**
 * @brief Send a tasklists_about message
 * @param chan MAVLink channel to send the message
 *
 * @param task_run run tasklists
 * @param task_return return tasklists 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tasklists_about_send(mavlink_channel_t chan, uint8_t task_run, uint8_t task_return)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN];
    _mav_put_uint8_t(buf, 0, task_run);
    _mav_put_uint8_t(buf, 1, task_return);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS_ABOUT, buf, MAVLINK_MSG_ID_TASKLISTS_ABOUT_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_CRC);
#else
    mavlink_tasklists_about_t packet;
    packet.task_run = task_run;
    packet.task_return = task_return;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS_ABOUT, (const char *)&packet, MAVLINK_MSG_ID_TASKLISTS_ABOUT_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_CRC);
#endif
}

/**
 * @brief Send a tasklists_about message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tasklists_about_send_struct(mavlink_channel_t chan, const mavlink_tasklists_about_t* tasklists_about)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tasklists_about_send(chan, tasklists_about->task_run, tasklists_about->task_return);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS_ABOUT, (const char *)tasklists_about, MAVLINK_MSG_ID_TASKLISTS_ABOUT_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tasklists_about_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t task_run, uint8_t task_return)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, task_run);
    _mav_put_uint8_t(buf, 1, task_return);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS_ABOUT, buf, MAVLINK_MSG_ID_TASKLISTS_ABOUT_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_CRC);
#else
    mavlink_tasklists_about_t *packet = (mavlink_tasklists_about_t *)msgbuf;
    packet->task_run = task_run;
    packet->task_return = task_return;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASKLISTS_ABOUT, (const char *)packet, MAVLINK_MSG_ID_TASKLISTS_ABOUT_MIN_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN, MAVLINK_MSG_ID_TASKLISTS_ABOUT_CRC);
#endif
}
#endif

#endif

// MESSAGE TASKLISTS_ABOUT UNPACKING


/**
 * @brief Get field task_run from tasklists_about message
 *
 * @return run tasklists
 */
static inline uint8_t mavlink_msg_tasklists_about_get_task_run(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field task_return from tasklists_about message
 *
 * @return return tasklists 
 */
static inline uint8_t mavlink_msg_tasklists_about_get_task_return(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a tasklists_about message into a struct
 *
 * @param msg The message to decode
 * @param tasklists_about C-struct to decode the message contents into
 */
static inline void mavlink_msg_tasklists_about_decode(const mavlink_message_t* msg, mavlink_tasklists_about_t* tasklists_about)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tasklists_about->task_run = mavlink_msg_tasklists_about_get_task_run(msg);
    tasklists_about->task_return = mavlink_msg_tasklists_about_get_task_return(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN? msg->len : MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN;
        memset(tasklists_about, 0, MAVLINK_MSG_ID_TASKLISTS_ABOUT_LEN);
    memcpy(tasklists_about, _MAV_PAYLOAD(msg), len);
#endif
}
