#pragma once
// MESSAGE SM_STATE PACKING

#define MAVLINK_MSG_ID_SM_STATE 226


typedef struct __mavlink_sm_state_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 int32_t state; /*<  State machine state*/
} mavlink_sm_state_t;

#define MAVLINK_MSG_ID_SM_STATE_LEN 12
#define MAVLINK_MSG_ID_SM_STATE_MIN_LEN 12
#define MAVLINK_MSG_ID_226_LEN 12
#define MAVLINK_MSG_ID_226_MIN_LEN 12

#define MAVLINK_MSG_ID_SM_STATE_CRC 177
#define MAVLINK_MSG_ID_226_CRC 177



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SM_STATE { \
    226, \
    "SM_STATE", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sm_state_t, time_usec) }, \
         { "state", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_sm_state_t, state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SM_STATE { \
    "SM_STATE", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sm_state_t, time_usec) }, \
         { "state", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_sm_state_t, state) }, \
         } \
}
#endif

/**
 * @brief Pack a sm_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param state  State machine state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sm_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, int32_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SM_STATE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SM_STATE_LEN);
#else
    mavlink_sm_state_t packet;
    packet.time_usec = time_usec;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SM_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SM_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SM_STATE_MIN_LEN, MAVLINK_MSG_ID_SM_STATE_LEN, MAVLINK_MSG_ID_SM_STATE_CRC);
}

/**
 * @brief Pack a sm_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param state  State machine state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sm_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,int32_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SM_STATE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SM_STATE_LEN);
#else
    mavlink_sm_state_t packet;
    packet.time_usec = time_usec;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SM_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SM_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SM_STATE_MIN_LEN, MAVLINK_MSG_ID_SM_STATE_LEN, MAVLINK_MSG_ID_SM_STATE_CRC);
}

/**
 * @brief Encode a sm_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sm_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sm_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sm_state_t* sm_state)
{
    return mavlink_msg_sm_state_pack(system_id, component_id, msg, sm_state->time_usec, sm_state->state);
}

/**
 * @brief Encode a sm_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sm_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sm_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sm_state_t* sm_state)
{
    return mavlink_msg_sm_state_pack_chan(system_id, component_id, chan, msg, sm_state->time_usec, sm_state->state);
}

/**
 * @brief Send a sm_state message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param state  State machine state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sm_state_send(mavlink_channel_t chan, uint64_t time_usec, int32_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SM_STATE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_STATE, buf, MAVLINK_MSG_ID_SM_STATE_MIN_LEN, MAVLINK_MSG_ID_SM_STATE_LEN, MAVLINK_MSG_ID_SM_STATE_CRC);
#else
    mavlink_sm_state_t packet;
    packet.time_usec = time_usec;
    packet.state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_STATE, (const char *)&packet, MAVLINK_MSG_ID_SM_STATE_MIN_LEN, MAVLINK_MSG_ID_SM_STATE_LEN, MAVLINK_MSG_ID_SM_STATE_CRC);
#endif
}

/**
 * @brief Send a sm_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sm_state_send_struct(mavlink_channel_t chan, const mavlink_sm_state_t* sm_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sm_state_send(chan, sm_state->time_usec, sm_state->state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_STATE, (const char *)sm_state, MAVLINK_MSG_ID_SM_STATE_MIN_LEN, MAVLINK_MSG_ID_SM_STATE_LEN, MAVLINK_MSG_ID_SM_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SM_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sm_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, int32_t state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_STATE, buf, MAVLINK_MSG_ID_SM_STATE_MIN_LEN, MAVLINK_MSG_ID_SM_STATE_LEN, MAVLINK_MSG_ID_SM_STATE_CRC);
#else
    mavlink_sm_state_t *packet = (mavlink_sm_state_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SM_STATE, (const char *)packet, MAVLINK_MSG_ID_SM_STATE_MIN_LEN, MAVLINK_MSG_ID_SM_STATE_LEN, MAVLINK_MSG_ID_SM_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE SM_STATE UNPACKING


/**
 * @brief Get field time_usec from sm_state message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_sm_state_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from sm_state message
 *
 * @return  State machine state
 */
static inline int32_t mavlink_msg_sm_state_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a sm_state message into a struct
 *
 * @param msg The message to decode
 * @param sm_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_sm_state_decode(const mavlink_message_t* msg, mavlink_sm_state_t* sm_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sm_state->time_usec = mavlink_msg_sm_state_get_time_usec(msg);
    sm_state->state = mavlink_msg_sm_state_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SM_STATE_LEN? msg->len : MAVLINK_MSG_ID_SM_STATE_LEN;
        memset(sm_state, 0, MAVLINK_MSG_ID_SM_STATE_LEN);
    memcpy(sm_state, _MAV_PAYLOAD(msg), len);
#endif
}
