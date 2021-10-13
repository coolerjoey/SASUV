#pragma once
// MESSAGE DATA_TIME PACKING

#define MAVLINK_MSG_ID_DATA_TIME 14

MAVPACKED(
typedef struct __mavlink_data_time_t {
 uint8_t year; /*< year*/
 uint8_t month; /*< month */
 uint8_t day; /*< day */
 uint8_t hour; /*< hour */
 uint8_t minute; /*< minute */
 uint8_t second; /*< second */
 uint8_t week; /*< week */
}) mavlink_data_time_t;

#define MAVLINK_MSG_ID_DATA_TIME_LEN 7
#define MAVLINK_MSG_ID_DATA_TIME_MIN_LEN 7
#define MAVLINK_MSG_ID_14_LEN 7
#define MAVLINK_MSG_ID_14_MIN_LEN 7

#define MAVLINK_MSG_ID_DATA_TIME_CRC 200
#define MAVLINK_MSG_ID_14_CRC 200



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DATA_TIME { \
    14, \
    "DATA_TIME", \
    7, \
    {  { "year", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_data_time_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_data_time_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_data_time_t, day) }, \
         { "hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_data_time_t, hour) }, \
         { "minute", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_data_time_t, minute) }, \
         { "second", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_data_time_t, second) }, \
         { "week", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_data_time_t, week) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DATA_TIME { \
    "DATA_TIME", \
    7, \
    {  { "year", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_data_time_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_data_time_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_data_time_t, day) }, \
         { "hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_data_time_t, hour) }, \
         { "minute", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_data_time_t, minute) }, \
         { "second", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_data_time_t, second) }, \
         { "week", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_data_time_t, week) }, \
         } \
}
#endif

/**
 * @brief Pack a data_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param year year
 * @param month month 
 * @param day day 
 * @param hour hour 
 * @param minute minute 
 * @param second second 
 * @param week week 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t week)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA_TIME_LEN];
    _mav_put_uint8_t(buf, 0, year);
    _mav_put_uint8_t(buf, 1, month);
    _mav_put_uint8_t(buf, 2, day);
    _mav_put_uint8_t(buf, 3, hour);
    _mav_put_uint8_t(buf, 4, minute);
    _mav_put_uint8_t(buf, 5, second);
    _mav_put_uint8_t(buf, 6, week);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA_TIME_LEN);
#else
    mavlink_data_time_t packet;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.minute = minute;
    packet.second = second;
    packet.week = week;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA_TIME;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA_TIME_MIN_LEN, MAVLINK_MSG_ID_DATA_TIME_LEN, MAVLINK_MSG_ID_DATA_TIME_CRC);
}

/**
 * @brief Pack a data_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param year year
 * @param month month 
 * @param day day 
 * @param hour hour 
 * @param minute minute 
 * @param second second 
 * @param week week 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t second,uint8_t week)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA_TIME_LEN];
    _mav_put_uint8_t(buf, 0, year);
    _mav_put_uint8_t(buf, 1, month);
    _mav_put_uint8_t(buf, 2, day);
    _mav_put_uint8_t(buf, 3, hour);
    _mav_put_uint8_t(buf, 4, minute);
    _mav_put_uint8_t(buf, 5, second);
    _mav_put_uint8_t(buf, 6, week);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA_TIME_LEN);
#else
    mavlink_data_time_t packet;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.minute = minute;
    packet.second = second;
    packet.week = week;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA_TIME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA_TIME_MIN_LEN, MAVLINK_MSG_ID_DATA_TIME_LEN, MAVLINK_MSG_ID_DATA_TIME_CRC);
}

/**
 * @brief Encode a data_time struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data_time_t* data_time)
{
    return mavlink_msg_data_time_pack(system_id, component_id, msg, data_time->year, data_time->month, data_time->day, data_time->hour, data_time->minute, data_time->second, data_time->week);
}

/**
 * @brief Encode a data_time struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_time_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_data_time_t* data_time)
{
    return mavlink_msg_data_time_pack_chan(system_id, component_id, chan, msg, data_time->year, data_time->month, data_time->day, data_time->hour, data_time->minute, data_time->second, data_time->week);
}

/**
 * @brief Send a data_time message
 * @param chan MAVLink channel to send the message
 *
 * @param year year
 * @param month month 
 * @param day day 
 * @param hour hour 
 * @param minute minute 
 * @param second second 
 * @param week week 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_time_send(mavlink_channel_t chan, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t week)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA_TIME_LEN];
    _mav_put_uint8_t(buf, 0, year);
    _mav_put_uint8_t(buf, 1, month);
    _mav_put_uint8_t(buf, 2, day);
    _mav_put_uint8_t(buf, 3, hour);
    _mav_put_uint8_t(buf, 4, minute);
    _mav_put_uint8_t(buf, 5, second);
    _mav_put_uint8_t(buf, 6, week);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TIME, buf, MAVLINK_MSG_ID_DATA_TIME_MIN_LEN, MAVLINK_MSG_ID_DATA_TIME_LEN, MAVLINK_MSG_ID_DATA_TIME_CRC);
#else
    mavlink_data_time_t packet;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.minute = minute;
    packet.second = second;
    packet.week = week;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TIME, (const char *)&packet, MAVLINK_MSG_ID_DATA_TIME_MIN_LEN, MAVLINK_MSG_ID_DATA_TIME_LEN, MAVLINK_MSG_ID_DATA_TIME_CRC);
#endif
}

/**
 * @brief Send a data_time message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_data_time_send_struct(mavlink_channel_t chan, const mavlink_data_time_t* data_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_data_time_send(chan, data_time->year, data_time->month, data_time->day, data_time->hour, data_time->minute, data_time->second, data_time->week);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TIME, (const char *)data_time, MAVLINK_MSG_ID_DATA_TIME_MIN_LEN, MAVLINK_MSG_ID_DATA_TIME_LEN, MAVLINK_MSG_ID_DATA_TIME_CRC);
#endif
}

#if MAVLINK_MSG_ID_DATA_TIME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_data_time_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t week)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, year);
    _mav_put_uint8_t(buf, 1, month);
    _mav_put_uint8_t(buf, 2, day);
    _mav_put_uint8_t(buf, 3, hour);
    _mav_put_uint8_t(buf, 4, minute);
    _mav_put_uint8_t(buf, 5, second);
    _mav_put_uint8_t(buf, 6, week);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TIME, buf, MAVLINK_MSG_ID_DATA_TIME_MIN_LEN, MAVLINK_MSG_ID_DATA_TIME_LEN, MAVLINK_MSG_ID_DATA_TIME_CRC);
#else
    mavlink_data_time_t *packet = (mavlink_data_time_t *)msgbuf;
    packet->year = year;
    packet->month = month;
    packet->day = day;
    packet->hour = hour;
    packet->minute = minute;
    packet->second = second;
    packet->week = week;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_TIME, (const char *)packet, MAVLINK_MSG_ID_DATA_TIME_MIN_LEN, MAVLINK_MSG_ID_DATA_TIME_LEN, MAVLINK_MSG_ID_DATA_TIME_CRC);
#endif
}
#endif

#endif

// MESSAGE DATA_TIME UNPACKING


/**
 * @brief Get field year from data_time message
 *
 * @return year
 */
static inline uint8_t mavlink_msg_data_time_get_year(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field month from data_time message
 *
 * @return month 
 */
static inline uint8_t mavlink_msg_data_time_get_month(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field day from data_time message
 *
 * @return day 
 */
static inline uint8_t mavlink_msg_data_time_get_day(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field hour from data_time message
 *
 * @return hour 
 */
static inline uint8_t mavlink_msg_data_time_get_hour(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field minute from data_time message
 *
 * @return minute 
 */
static inline uint8_t mavlink_msg_data_time_get_minute(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field second from data_time message
 *
 * @return second 
 */
static inline uint8_t mavlink_msg_data_time_get_second(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field week from data_time message
 *
 * @return week 
 */
static inline uint8_t mavlink_msg_data_time_get_week(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a data_time message into a struct
 *
 * @param msg The message to decode
 * @param data_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_data_time_decode(const mavlink_message_t* msg, mavlink_data_time_t* data_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    data_time->year = mavlink_msg_data_time_get_year(msg);
    data_time->month = mavlink_msg_data_time_get_month(msg);
    data_time->day = mavlink_msg_data_time_get_day(msg);
    data_time->hour = mavlink_msg_data_time_get_hour(msg);
    data_time->minute = mavlink_msg_data_time_get_minute(msg);
    data_time->second = mavlink_msg_data_time_get_second(msg);
    data_time->week = mavlink_msg_data_time_get_week(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DATA_TIME_LEN? msg->len : MAVLINK_MSG_ID_DATA_TIME_LEN;
        memset(data_time, 0, MAVLINK_MSG_ID_DATA_TIME_LEN);
    memcpy(data_time, _MAV_PAYLOAD(msg), len);
#endif
}
