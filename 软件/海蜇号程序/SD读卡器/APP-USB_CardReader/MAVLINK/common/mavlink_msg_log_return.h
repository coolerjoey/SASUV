#pragma once
// MESSAGE LOG_RETURN PACKING

#define MAVLINK_MSG_ID_LOG_RETURN 180

MAVPACKED(
typedef struct __mavlink_log_return_t {
 uint32_t log_return; /*< the log to be returned*/
 uint32_t log_delete; /*< the log to be deleted*/
}) mavlink_log_return_t;

#define MAVLINK_MSG_ID_LOG_RETURN_LEN 8
#define MAVLINK_MSG_ID_LOG_RETURN_MIN_LEN 8
#define MAVLINK_MSG_ID_180_LEN 8
#define MAVLINK_MSG_ID_180_MIN_LEN 8

#define MAVLINK_MSG_ID_LOG_RETURN_CRC 51
#define MAVLINK_MSG_ID_180_CRC 51



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOG_RETURN { \
    180, \
    "LOG_RETURN", \
    2, \
    {  { "log_return", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_log_return_t, log_return) }, \
         { "log_delete", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_log_return_t, log_delete) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOG_RETURN { \
    "LOG_RETURN", \
    2, \
    {  { "log_return", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_log_return_t, log_return) }, \
         { "log_delete", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_log_return_t, log_delete) }, \
         } \
}
#endif

/**
 * @brief Pack a log_return message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param log_return the log to be returned
 * @param log_delete the log to be deleted
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_log_return_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t log_return, uint32_t log_delete)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOG_RETURN_LEN];
    _mav_put_uint32_t(buf, 0, log_return);
    _mav_put_uint32_t(buf, 4, log_delete);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOG_RETURN_LEN);
#else
    mavlink_log_return_t packet;
    packet.log_return = log_return;
    packet.log_delete = log_delete;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOG_RETURN_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOG_RETURN;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOG_RETURN_MIN_LEN, MAVLINK_MSG_ID_LOG_RETURN_LEN, MAVLINK_MSG_ID_LOG_RETURN_CRC);
}

/**
 * @brief Pack a log_return message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param log_return the log to be returned
 * @param log_delete the log to be deleted
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_log_return_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t log_return,uint32_t log_delete)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOG_RETURN_LEN];
    _mav_put_uint32_t(buf, 0, log_return);
    _mav_put_uint32_t(buf, 4, log_delete);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOG_RETURN_LEN);
#else
    mavlink_log_return_t packet;
    packet.log_return = log_return;
    packet.log_delete = log_delete;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOG_RETURN_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOG_RETURN;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOG_RETURN_MIN_LEN, MAVLINK_MSG_ID_LOG_RETURN_LEN, MAVLINK_MSG_ID_LOG_RETURN_CRC);
}

/**
 * @brief Encode a log_return struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param log_return C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_log_return_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_log_return_t* log_return)
{
    return mavlink_msg_log_return_pack(system_id, component_id, msg, log_return->log_return, log_return->log_delete);
}

/**
 * @brief Encode a log_return struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param log_return C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_log_return_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_log_return_t* log_return)
{
    return mavlink_msg_log_return_pack_chan(system_id, component_id, chan, msg, log_return->log_return, log_return->log_delete);
}

/**
 * @brief Send a log_return message
 * @param chan MAVLink channel to send the message
 *
 * @param log_return the log to be returned
 * @param log_delete the log to be deleted
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_log_return_send(mavlink_channel_t chan, uint32_t log_return, uint32_t log_delete)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOG_RETURN_LEN];
    _mav_put_uint32_t(buf, 0, log_return);
    _mav_put_uint32_t(buf, 4, log_delete);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_RETURN, buf, MAVLINK_MSG_ID_LOG_RETURN_MIN_LEN, MAVLINK_MSG_ID_LOG_RETURN_LEN, MAVLINK_MSG_ID_LOG_RETURN_CRC);
#else
    mavlink_log_return_t packet;
    packet.log_return = log_return;
    packet.log_delete = log_delete;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_RETURN, (const char *)&packet, MAVLINK_MSG_ID_LOG_RETURN_MIN_LEN, MAVLINK_MSG_ID_LOG_RETURN_LEN, MAVLINK_MSG_ID_LOG_RETURN_CRC);
#endif
}

/**
 * @brief Send a log_return message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_log_return_send_struct(mavlink_channel_t chan, const mavlink_log_return_t* log_return)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_log_return_send(chan, log_return->log_return, log_return->log_delete);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_RETURN, (const char *)log_return, MAVLINK_MSG_ID_LOG_RETURN_MIN_LEN, MAVLINK_MSG_ID_LOG_RETURN_LEN, MAVLINK_MSG_ID_LOG_RETURN_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOG_RETURN_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_log_return_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t log_return, uint32_t log_delete)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, log_return);
    _mav_put_uint32_t(buf, 4, log_delete);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_RETURN, buf, MAVLINK_MSG_ID_LOG_RETURN_MIN_LEN, MAVLINK_MSG_ID_LOG_RETURN_LEN, MAVLINK_MSG_ID_LOG_RETURN_CRC);
#else
    mavlink_log_return_t *packet = (mavlink_log_return_t *)msgbuf;
    packet->log_return = log_return;
    packet->log_delete = log_delete;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOG_RETURN, (const char *)packet, MAVLINK_MSG_ID_LOG_RETURN_MIN_LEN, MAVLINK_MSG_ID_LOG_RETURN_LEN, MAVLINK_MSG_ID_LOG_RETURN_CRC);
#endif
}
#endif

#endif

// MESSAGE LOG_RETURN UNPACKING


/**
 * @brief Get field log_return from log_return message
 *
 * @return the log to be returned
 */
static inline uint32_t mavlink_msg_log_return_get_log_return(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field log_delete from log_return message
 *
 * @return the log to be deleted
 */
static inline uint32_t mavlink_msg_log_return_get_log_delete(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a log_return message into a struct
 *
 * @param msg The message to decode
 * @param log_return C-struct to decode the message contents into
 */
static inline void mavlink_msg_log_return_decode(const mavlink_message_t* msg, mavlink_log_return_t* log_return)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    log_return->log_return = mavlink_msg_log_return_get_log_return(msg);
    log_return->log_delete = mavlink_msg_log_return_get_log_delete(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOG_RETURN_LEN? msg->len : MAVLINK_MSG_ID_LOG_RETURN_LEN;
        memset(log_return, 0, MAVLINK_MSG_ID_LOG_RETURN_LEN);
    memcpy(log_return, _MAV_PAYLOAD(msg), len);
#endif
}
