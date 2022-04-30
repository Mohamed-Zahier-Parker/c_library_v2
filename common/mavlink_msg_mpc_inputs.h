#pragma once
// MESSAGE MPC_INPUTS PACKING

#define MAVLINK_MSG_ID_MPC_INPUTS 227


typedef struct __mavlink_mpc_inputs_t {
 uint64_t time_usec; /*< [us]  */
 float mpc_ref_in[2]; /*<   */
 float mpc_mo_in[2]; /*<   */
 int32_t state; /*<  State machine state*/
 float grd_vel; /*<  Ground Speed of UAV*/
 int8_t ramp_trajectory; /*<  Boolean Ramp trajectory tracking(1 or 0)*/
 int8_t mpc_activate; /*<  Boolean mpc activation(1 or 0)*/
} mavlink_mpc_inputs_t;

#define MAVLINK_MSG_ID_MPC_INPUTS_LEN 34
#define MAVLINK_MSG_ID_MPC_INPUTS_MIN_LEN 34
#define MAVLINK_MSG_ID_227_LEN 34
#define MAVLINK_MSG_ID_227_MIN_LEN 34

#define MAVLINK_MSG_ID_MPC_INPUTS_CRC 194
#define MAVLINK_MSG_ID_227_CRC 194

#define MAVLINK_MSG_MPC_INPUTS_FIELD_MPC_REF_IN_LEN 2
#define MAVLINK_MSG_MPC_INPUTS_FIELD_MPC_MO_IN_LEN 2

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MPC_INPUTS { \
    227, \
    "MPC_INPUTS", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mpc_inputs_t, time_usec) }, \
         { "mpc_ref_in", NULL, MAVLINK_TYPE_FLOAT, 2, 8, offsetof(mavlink_mpc_inputs_t, mpc_ref_in) }, \
         { "mpc_mo_in", NULL, MAVLINK_TYPE_FLOAT, 2, 16, offsetof(mavlink_mpc_inputs_t, mpc_mo_in) }, \
         { "state", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_mpc_inputs_t, state) }, \
         { "grd_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mpc_inputs_t, grd_vel) }, \
         { "ramp_trajectory", NULL, MAVLINK_TYPE_INT8_T, 0, 32, offsetof(mavlink_mpc_inputs_t, ramp_trajectory) }, \
         { "mpc_activate", NULL, MAVLINK_TYPE_INT8_T, 0, 33, offsetof(mavlink_mpc_inputs_t, mpc_activate) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MPC_INPUTS { \
    "MPC_INPUTS", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mpc_inputs_t, time_usec) }, \
         { "mpc_ref_in", NULL, MAVLINK_TYPE_FLOAT, 2, 8, offsetof(mavlink_mpc_inputs_t, mpc_ref_in) }, \
         { "mpc_mo_in", NULL, MAVLINK_TYPE_FLOAT, 2, 16, offsetof(mavlink_mpc_inputs_t, mpc_mo_in) }, \
         { "state", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_mpc_inputs_t, state) }, \
         { "grd_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mpc_inputs_t, grd_vel) }, \
         { "ramp_trajectory", NULL, MAVLINK_TYPE_INT8_T, 0, 32, offsetof(mavlink_mpc_inputs_t, ramp_trajectory) }, \
         { "mpc_activate", NULL, MAVLINK_TYPE_INT8_T, 0, 33, offsetof(mavlink_mpc_inputs_t, mpc_activate) }, \
         } \
}
#endif

/**
 * @brief Pack a mpc_inputs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us]  
 * @param mpc_ref_in   
 * @param mpc_mo_in   
 * @param state  State machine state
 * @param grd_vel  Ground Speed of UAV
 * @param ramp_trajectory  Boolean Ramp trajectory tracking(1 or 0)
 * @param mpc_activate  Boolean mpc activation(1 or 0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mpc_inputs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const float *mpc_ref_in, const float *mpc_mo_in, int32_t state, float grd_vel, int8_t ramp_trajectory, int8_t mpc_activate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPC_INPUTS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 24, state);
    _mav_put_float(buf, 28, grd_vel);
    _mav_put_int8_t(buf, 32, ramp_trajectory);
    _mav_put_int8_t(buf, 33, mpc_activate);
    _mav_put_float_array(buf, 8, mpc_ref_in, 2);
    _mav_put_float_array(buf, 16, mpc_mo_in, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPC_INPUTS_LEN);
#else
    mavlink_mpc_inputs_t packet;
    packet.time_usec = time_usec;
    packet.state = state;
    packet.grd_vel = grd_vel;
    packet.ramp_trajectory = ramp_trajectory;
    packet.mpc_activate = mpc_activate;
    mav_array_memcpy(packet.mpc_ref_in, mpc_ref_in, sizeof(float)*2);
    mav_array_memcpy(packet.mpc_mo_in, mpc_mo_in, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPC_INPUTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPC_INPUTS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MPC_INPUTS_MIN_LEN, MAVLINK_MSG_ID_MPC_INPUTS_LEN, MAVLINK_MSG_ID_MPC_INPUTS_CRC);
}

/**
 * @brief Pack a mpc_inputs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us]  
 * @param mpc_ref_in   
 * @param mpc_mo_in   
 * @param state  State machine state
 * @param grd_vel  Ground Speed of UAV
 * @param ramp_trajectory  Boolean Ramp trajectory tracking(1 or 0)
 * @param mpc_activate  Boolean mpc activation(1 or 0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mpc_inputs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const float *mpc_ref_in,const float *mpc_mo_in,int32_t state,float grd_vel,int8_t ramp_trajectory,int8_t mpc_activate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPC_INPUTS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 24, state);
    _mav_put_float(buf, 28, grd_vel);
    _mav_put_int8_t(buf, 32, ramp_trajectory);
    _mav_put_int8_t(buf, 33, mpc_activate);
    _mav_put_float_array(buf, 8, mpc_ref_in, 2);
    _mav_put_float_array(buf, 16, mpc_mo_in, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPC_INPUTS_LEN);
#else
    mavlink_mpc_inputs_t packet;
    packet.time_usec = time_usec;
    packet.state = state;
    packet.grd_vel = grd_vel;
    packet.ramp_trajectory = ramp_trajectory;
    packet.mpc_activate = mpc_activate;
    mav_array_memcpy(packet.mpc_ref_in, mpc_ref_in, sizeof(float)*2);
    mav_array_memcpy(packet.mpc_mo_in, mpc_mo_in, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPC_INPUTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPC_INPUTS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MPC_INPUTS_MIN_LEN, MAVLINK_MSG_ID_MPC_INPUTS_LEN, MAVLINK_MSG_ID_MPC_INPUTS_CRC);
}

/**
 * @brief Encode a mpc_inputs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mpc_inputs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mpc_inputs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mpc_inputs_t* mpc_inputs)
{
    return mavlink_msg_mpc_inputs_pack(system_id, component_id, msg, mpc_inputs->time_usec, mpc_inputs->mpc_ref_in, mpc_inputs->mpc_mo_in, mpc_inputs->state, mpc_inputs->grd_vel, mpc_inputs->ramp_trajectory, mpc_inputs->mpc_activate);
}

/**
 * @brief Encode a mpc_inputs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mpc_inputs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mpc_inputs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mpc_inputs_t* mpc_inputs)
{
    return mavlink_msg_mpc_inputs_pack_chan(system_id, component_id, chan, msg, mpc_inputs->time_usec, mpc_inputs->mpc_ref_in, mpc_inputs->mpc_mo_in, mpc_inputs->state, mpc_inputs->grd_vel, mpc_inputs->ramp_trajectory, mpc_inputs->mpc_activate);
}

/**
 * @brief Send a mpc_inputs message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us]  
 * @param mpc_ref_in   
 * @param mpc_mo_in   
 * @param state  State machine state
 * @param grd_vel  Ground Speed of UAV
 * @param ramp_trajectory  Boolean Ramp trajectory tracking(1 or 0)
 * @param mpc_activate  Boolean mpc activation(1 or 0)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mpc_inputs_send(mavlink_channel_t chan, uint64_t time_usec, const float *mpc_ref_in, const float *mpc_mo_in, int32_t state, float grd_vel, int8_t ramp_trajectory, int8_t mpc_activate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPC_INPUTS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 24, state);
    _mav_put_float(buf, 28, grd_vel);
    _mav_put_int8_t(buf, 32, ramp_trajectory);
    _mav_put_int8_t(buf, 33, mpc_activate);
    _mav_put_float_array(buf, 8, mpc_ref_in, 2);
    _mav_put_float_array(buf, 16, mpc_mo_in, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_INPUTS, buf, MAVLINK_MSG_ID_MPC_INPUTS_MIN_LEN, MAVLINK_MSG_ID_MPC_INPUTS_LEN, MAVLINK_MSG_ID_MPC_INPUTS_CRC);
#else
    mavlink_mpc_inputs_t packet;
    packet.time_usec = time_usec;
    packet.state = state;
    packet.grd_vel = grd_vel;
    packet.ramp_trajectory = ramp_trajectory;
    packet.mpc_activate = mpc_activate;
    mav_array_memcpy(packet.mpc_ref_in, mpc_ref_in, sizeof(float)*2);
    mav_array_memcpy(packet.mpc_mo_in, mpc_mo_in, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_INPUTS, (const char *)&packet, MAVLINK_MSG_ID_MPC_INPUTS_MIN_LEN, MAVLINK_MSG_ID_MPC_INPUTS_LEN, MAVLINK_MSG_ID_MPC_INPUTS_CRC);
#endif
}

/**
 * @brief Send a mpc_inputs message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mpc_inputs_send_struct(mavlink_channel_t chan, const mavlink_mpc_inputs_t* mpc_inputs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mpc_inputs_send(chan, mpc_inputs->time_usec, mpc_inputs->mpc_ref_in, mpc_inputs->mpc_mo_in, mpc_inputs->state, mpc_inputs->grd_vel, mpc_inputs->ramp_trajectory, mpc_inputs->mpc_activate);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_INPUTS, (const char *)mpc_inputs, MAVLINK_MSG_ID_MPC_INPUTS_MIN_LEN, MAVLINK_MSG_ID_MPC_INPUTS_LEN, MAVLINK_MSG_ID_MPC_INPUTS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MPC_INPUTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mpc_inputs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const float *mpc_ref_in, const float *mpc_mo_in, int32_t state, float grd_vel, int8_t ramp_trajectory, int8_t mpc_activate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 24, state);
    _mav_put_float(buf, 28, grd_vel);
    _mav_put_int8_t(buf, 32, ramp_trajectory);
    _mav_put_int8_t(buf, 33, mpc_activate);
    _mav_put_float_array(buf, 8, mpc_ref_in, 2);
    _mav_put_float_array(buf, 16, mpc_mo_in, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_INPUTS, buf, MAVLINK_MSG_ID_MPC_INPUTS_MIN_LEN, MAVLINK_MSG_ID_MPC_INPUTS_LEN, MAVLINK_MSG_ID_MPC_INPUTS_CRC);
#else
    mavlink_mpc_inputs_t *packet = (mavlink_mpc_inputs_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->state = state;
    packet->grd_vel = grd_vel;
    packet->ramp_trajectory = ramp_trajectory;
    packet->mpc_activate = mpc_activate;
    mav_array_memcpy(packet->mpc_ref_in, mpc_ref_in, sizeof(float)*2);
    mav_array_memcpy(packet->mpc_mo_in, mpc_mo_in, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPC_INPUTS, (const char *)packet, MAVLINK_MSG_ID_MPC_INPUTS_MIN_LEN, MAVLINK_MSG_ID_MPC_INPUTS_LEN, MAVLINK_MSG_ID_MPC_INPUTS_CRC);
#endif
}
#endif

#endif

// MESSAGE MPC_INPUTS UNPACKING


/**
 * @brief Get field time_usec from mpc_inputs message
 *
 * @return [us]  
 */
static inline uint64_t mavlink_msg_mpc_inputs_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mpc_ref_in from mpc_inputs message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_mpc_inputs_get_mpc_ref_in(const mavlink_message_t* msg, float *mpc_ref_in)
{
    return _MAV_RETURN_float_array(msg, mpc_ref_in, 2,  8);
}

/**
 * @brief Get field mpc_mo_in from mpc_inputs message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_mpc_inputs_get_mpc_mo_in(const mavlink_message_t* msg, float *mpc_mo_in)
{
    return _MAV_RETURN_float_array(msg, mpc_mo_in, 2,  16);
}

/**
 * @brief Get field state from mpc_inputs message
 *
 * @return  State machine state
 */
static inline int32_t mavlink_msg_mpc_inputs_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field grd_vel from mpc_inputs message
 *
 * @return  Ground Speed of UAV
 */
static inline float mavlink_msg_mpc_inputs_get_grd_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field ramp_trajectory from mpc_inputs message
 *
 * @return  Boolean Ramp trajectory tracking(1 or 0)
 */
static inline int8_t mavlink_msg_mpc_inputs_get_ramp_trajectory(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  32);
}

/**
 * @brief Get field mpc_activate from mpc_inputs message
 *
 * @return  Boolean mpc activation(1 or 0)
 */
static inline int8_t mavlink_msg_mpc_inputs_get_mpc_activate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  33);
}

/**
 * @brief Decode a mpc_inputs message into a struct
 *
 * @param msg The message to decode
 * @param mpc_inputs C-struct to decode the message contents into
 */
static inline void mavlink_msg_mpc_inputs_decode(const mavlink_message_t* msg, mavlink_mpc_inputs_t* mpc_inputs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mpc_inputs->time_usec = mavlink_msg_mpc_inputs_get_time_usec(msg);
    mavlink_msg_mpc_inputs_get_mpc_ref_in(msg, mpc_inputs->mpc_ref_in);
    mavlink_msg_mpc_inputs_get_mpc_mo_in(msg, mpc_inputs->mpc_mo_in);
    mpc_inputs->state = mavlink_msg_mpc_inputs_get_state(msg);
    mpc_inputs->grd_vel = mavlink_msg_mpc_inputs_get_grd_vel(msg);
    mpc_inputs->ramp_trajectory = mavlink_msg_mpc_inputs_get_ramp_trajectory(msg);
    mpc_inputs->mpc_activate = mavlink_msg_mpc_inputs_get_mpc_activate(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MPC_INPUTS_LEN? msg->len : MAVLINK_MSG_ID_MPC_INPUTS_LEN;
        memset(mpc_inputs, 0, MAVLINK_MSG_ID_MPC_INPUTS_LEN);
    memcpy(mpc_inputs, _MAV_PAYLOAD(msg), len);
#endif
}
