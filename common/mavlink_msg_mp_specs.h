#pragma once
// MESSAGE MP_SPECS PACKING

#define MAVLINK_MSG_ID_MP_SPECS 225


typedef struct __mavlink_mp_specs_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float mp_position[3]; /*<  Moving Platform Position*/
 float mp_velocity[3]; /*<  Moving Platform Velocity*/
} mavlink_mp_specs_t;

#define MAVLINK_MSG_ID_MP_SPECS_LEN 32
#define MAVLINK_MSG_ID_MP_SPECS_MIN_LEN 32
#define MAVLINK_MSG_ID_225_LEN 32
#define MAVLINK_MSG_ID_225_MIN_LEN 32

#define MAVLINK_MSG_ID_MP_SPECS_CRC 43
#define MAVLINK_MSG_ID_225_CRC 43

#define MAVLINK_MSG_MP_SPECS_FIELD_MP_POSITION_LEN 3
#define MAVLINK_MSG_MP_SPECS_FIELD_MP_VELOCITY_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MP_SPECS { \
    225, \
    "MP_SPECS", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mp_specs_t, time_usec) }, \
         { "mp_position", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_mp_specs_t, mp_position) }, \
         { "mp_velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_mp_specs_t, mp_velocity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MP_SPECS { \
    "MP_SPECS", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mp_specs_t, time_usec) }, \
         { "mp_position", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_mp_specs_t, mp_position) }, \
         { "mp_velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_mp_specs_t, mp_velocity) }, \
         } \
}
#endif

/**
 * @brief Pack a mp_specs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param mp_position  Moving Platform Position
 * @param mp_velocity  Moving Platform Velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mp_specs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const float *mp_position, const float *mp_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MP_SPECS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float_array(buf, 8, mp_position, 3);
    _mav_put_float_array(buf, 20, mp_velocity, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MP_SPECS_LEN);
#else
    mavlink_mp_specs_t packet;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.mp_position, mp_position, sizeof(float)*3);
    mav_array_memcpy(packet.mp_velocity, mp_velocity, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MP_SPECS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MP_SPECS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MP_SPECS_MIN_LEN, MAVLINK_MSG_ID_MP_SPECS_LEN, MAVLINK_MSG_ID_MP_SPECS_CRC);
}

/**
 * @brief Pack a mp_specs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param mp_position  Moving Platform Position
 * @param mp_velocity  Moving Platform Velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mp_specs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const float *mp_position,const float *mp_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MP_SPECS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float_array(buf, 8, mp_position, 3);
    _mav_put_float_array(buf, 20, mp_velocity, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MP_SPECS_LEN);
#else
    mavlink_mp_specs_t packet;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.mp_position, mp_position, sizeof(float)*3);
    mav_array_memcpy(packet.mp_velocity, mp_velocity, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MP_SPECS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MP_SPECS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MP_SPECS_MIN_LEN, MAVLINK_MSG_ID_MP_SPECS_LEN, MAVLINK_MSG_ID_MP_SPECS_CRC);
}

/**
 * @brief Encode a mp_specs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mp_specs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mp_specs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mp_specs_t* mp_specs)
{
    return mavlink_msg_mp_specs_pack(system_id, component_id, msg, mp_specs->time_usec, mp_specs->mp_position, mp_specs->mp_velocity);
}

/**
 * @brief Encode a mp_specs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mp_specs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mp_specs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mp_specs_t* mp_specs)
{
    return mavlink_msg_mp_specs_pack_chan(system_id, component_id, chan, msg, mp_specs->time_usec, mp_specs->mp_position, mp_specs->mp_velocity);
}

/**
 * @brief Send a mp_specs message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param mp_position  Moving Platform Position
 * @param mp_velocity  Moving Platform Velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mp_specs_send(mavlink_channel_t chan, uint64_t time_usec, const float *mp_position, const float *mp_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MP_SPECS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float_array(buf, 8, mp_position, 3);
    _mav_put_float_array(buf, 20, mp_velocity, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MP_SPECS, buf, MAVLINK_MSG_ID_MP_SPECS_MIN_LEN, MAVLINK_MSG_ID_MP_SPECS_LEN, MAVLINK_MSG_ID_MP_SPECS_CRC);
#else
    mavlink_mp_specs_t packet;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.mp_position, mp_position, sizeof(float)*3);
    mav_array_memcpy(packet.mp_velocity, mp_velocity, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MP_SPECS, (const char *)&packet, MAVLINK_MSG_ID_MP_SPECS_MIN_LEN, MAVLINK_MSG_ID_MP_SPECS_LEN, MAVLINK_MSG_ID_MP_SPECS_CRC);
#endif
}

/**
 * @brief Send a mp_specs message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mp_specs_send_struct(mavlink_channel_t chan, const mavlink_mp_specs_t* mp_specs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mp_specs_send(chan, mp_specs->time_usec, mp_specs->mp_position, mp_specs->mp_velocity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MP_SPECS, (const char *)mp_specs, MAVLINK_MSG_ID_MP_SPECS_MIN_LEN, MAVLINK_MSG_ID_MP_SPECS_LEN, MAVLINK_MSG_ID_MP_SPECS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MP_SPECS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mp_specs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const float *mp_position, const float *mp_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float_array(buf, 8, mp_position, 3);
    _mav_put_float_array(buf, 20, mp_velocity, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MP_SPECS, buf, MAVLINK_MSG_ID_MP_SPECS_MIN_LEN, MAVLINK_MSG_ID_MP_SPECS_LEN, MAVLINK_MSG_ID_MP_SPECS_CRC);
#else
    mavlink_mp_specs_t *packet = (mavlink_mp_specs_t *)msgbuf;
    packet->time_usec = time_usec;
    mav_array_memcpy(packet->mp_position, mp_position, sizeof(float)*3);
    mav_array_memcpy(packet->mp_velocity, mp_velocity, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MP_SPECS, (const char *)packet, MAVLINK_MSG_ID_MP_SPECS_MIN_LEN, MAVLINK_MSG_ID_MP_SPECS_LEN, MAVLINK_MSG_ID_MP_SPECS_CRC);
#endif
}
#endif

#endif

// MESSAGE MP_SPECS UNPACKING


/**
 * @brief Get field time_usec from mp_specs message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_mp_specs_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mp_position from mp_specs message
 *
 * @return  Moving Platform Position
 */
static inline uint16_t mavlink_msg_mp_specs_get_mp_position(const mavlink_message_t* msg, float *mp_position)
{
    return _MAV_RETURN_float_array(msg, mp_position, 3,  8);
}

/**
 * @brief Get field mp_velocity from mp_specs message
 *
 * @return  Moving Platform Velocity
 */
static inline uint16_t mavlink_msg_mp_specs_get_mp_velocity(const mavlink_message_t* msg, float *mp_velocity)
{
    return _MAV_RETURN_float_array(msg, mp_velocity, 3,  20);
}

/**
 * @brief Decode a mp_specs message into a struct
 *
 * @param msg The message to decode
 * @param mp_specs C-struct to decode the message contents into
 */
static inline void mavlink_msg_mp_specs_decode(const mavlink_message_t* msg, mavlink_mp_specs_t* mp_specs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mp_specs->time_usec = mavlink_msg_mp_specs_get_time_usec(msg);
    mavlink_msg_mp_specs_get_mp_position(msg, mp_specs->mp_position);
    mavlink_msg_mp_specs_get_mp_velocity(msg, mp_specs->mp_velocity);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MP_SPECS_LEN? msg->len : MAVLINK_MSG_ID_MP_SPECS_LEN;
        memset(mp_specs, 0, MAVLINK_MSG_ID_MP_SPECS_LEN);
    memcpy(mp_specs, _MAV_PAYLOAD(msg), len);
#endif
}
