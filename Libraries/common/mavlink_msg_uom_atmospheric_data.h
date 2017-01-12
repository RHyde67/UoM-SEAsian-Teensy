// MESSAGE UOM_ATMOSPHERIC_DATA PACKING

#define MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA 225

typedef struct __mavlink_uom_atmospheric_data_t
{
 float roll; /*< Roll angle (rad, -pi..+pi)*/
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 float yaw; /*< Yaw angle (rad, -pi..+pi)*/
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int32_t lat; /*< Latitude, expressed as * 1E7*/
 int32_t lon; /*< Longitude, expressed as * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
 float airspeed; /*< Current airspeed in m/s*/
 float groundspeed; /*< Current ground speed in m/s*/
 float press_abs; /*< Absolute pressure (hectopascal)*/
 float press_diff; /*< Differential pressure 1 (hectopascal)*/
 float OP1; /*< Working Electrode voltage in mV*/
 float OP2; /*< Auxiliary Electrode voltage in mV*/
 float CO; /*< Measured CO Value in ppb*/
 int16_t vx; /*< Ground X Speed (Latitude), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude), expressed as m/s * 100*/
 int16_t heading; /*< Current heading in degrees, in compass units (0..360, 0=north)*/
 int16_t temperature; /*< Temperature measurement (0.01 degrees celsius)*/
} mavlink_uom_atmospheric_data_t;

#define MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN 82
#define MAVLINK_MSG_ID_225_LEN 82

#define MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_CRC 26
#define MAVLINK_MSG_ID_225_CRC 26



#define MAVLINK_MESSAGE_INFO_UOM_ATMOSPHERIC_DATA { \
	"UOM_ATMOSPHERIC_DATA", \
	23, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_uom_atmospheric_data_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_uom_atmospheric_data_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uom_atmospheric_data_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uom_atmospheric_data_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_uom_atmospheric_data_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_uom_atmospheric_data_t, yawspeed) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_uom_atmospheric_data_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_uom_atmospheric_data_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_uom_atmospheric_data_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_uom_atmospheric_data_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_uom_atmospheric_data_t, relative_alt) }, \
         { "airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_uom_atmospheric_data_t, airspeed) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_uom_atmospheric_data_t, groundspeed) }, \
         { "press_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_uom_atmospheric_data_t, press_abs) }, \
         { "press_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_uom_atmospheric_data_t, press_diff) }, \
         { "OP1", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_uom_atmospheric_data_t, OP1) }, \
         { "OP2", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_uom_atmospheric_data_t, OP2) }, \
         { "CO", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_uom_atmospheric_data_t, CO) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 72, offsetof(mavlink_uom_atmospheric_data_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 74, offsetof(mavlink_uom_atmospheric_data_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 76, offsetof(mavlink_uom_atmospheric_data_t, vz) }, \
         { "heading", NULL, MAVLINK_TYPE_INT16_T, 0, 78, offsetof(mavlink_uom_atmospheric_data_t, heading) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 80, offsetof(mavlink_uom_atmospheric_data_t, temperature) }, \
         } \
}


/**
 * @brief Pack a uom_atmospheric_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @param OP1 Working Electrode voltage in mV
 * @param OP2 Auxiliary Electrode voltage in mV
 * @param CO Measured CO Value in ppb
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_atmospheric_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, float airspeed, float groundspeed, int16_t heading, float press_abs, float press_diff, int16_t temperature, float OP1, float OP2, float CO)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, rollspeed);
	_mav_put_float(buf, 16, pitchspeed);
	_mav_put_float(buf, 20, yawspeed);
	_mav_put_uint32_t(buf, 24, time_boot_ms);
	_mav_put_int32_t(buf, 28, lat);
	_mav_put_int32_t(buf, 32, lon);
	_mav_put_int32_t(buf, 36, alt);
	_mav_put_int32_t(buf, 40, relative_alt);
	_mav_put_float(buf, 44, airspeed);
	_mav_put_float(buf, 48, groundspeed);
	_mav_put_float(buf, 52, press_abs);
	_mav_put_float(buf, 56, press_diff);
	_mav_put_float(buf, 60, OP1);
	_mav_put_float(buf, 64, OP2);
	_mav_put_float(buf, 68, CO);
	_mav_put_int16_t(buf, 72, vx);
	_mav_put_int16_t(buf, 74, vy);
	_mav_put_int16_t(buf, 76, vz);
	_mav_put_int16_t(buf, 78, heading);
	_mav_put_int16_t(buf, 80, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#else
	mavlink_uom_atmospheric_data_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.airspeed = airspeed;
	packet.groundspeed = groundspeed;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.OP1 = OP1;
	packet.OP2 = OP2;
	packet.CO = CO;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.heading = heading;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif
}

/**
 * @brief Pack a uom_atmospheric_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @param OP1 Working Electrode voltage in mV
 * @param OP2 Auxiliary Electrode voltage in mV
 * @param CO Measured CO Value in ppb
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uom_atmospheric_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed,uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,int16_t vx,int16_t vy,int16_t vz,float airspeed,float groundspeed,int16_t heading,float press_abs,float press_diff,int16_t temperature,float OP1,float OP2,float CO)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, rollspeed);
	_mav_put_float(buf, 16, pitchspeed);
	_mav_put_float(buf, 20, yawspeed);
	_mav_put_uint32_t(buf, 24, time_boot_ms);
	_mav_put_int32_t(buf, 28, lat);
	_mav_put_int32_t(buf, 32, lon);
	_mav_put_int32_t(buf, 36, alt);
	_mav_put_int32_t(buf, 40, relative_alt);
	_mav_put_float(buf, 44, airspeed);
	_mav_put_float(buf, 48, groundspeed);
	_mav_put_float(buf, 52, press_abs);
	_mav_put_float(buf, 56, press_diff);
	_mav_put_float(buf, 60, OP1);
	_mav_put_float(buf, 64, OP2);
	_mav_put_float(buf, 68, CO);
	_mav_put_int16_t(buf, 72, vx);
	_mav_put_int16_t(buf, 74, vy);
	_mav_put_int16_t(buf, 76, vz);
	_mav_put_int16_t(buf, 78, heading);
	_mav_put_int16_t(buf, 80, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#else
	mavlink_uom_atmospheric_data_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.airspeed = airspeed;
	packet.groundspeed = groundspeed;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.OP1 = OP1;
	packet.OP2 = OP2;
	packet.CO = CO;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.heading = heading;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif
}

/**
 * @brief Encode a uom_atmospheric_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uom_atmospheric_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_atmospheric_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uom_atmospheric_data_t* uom_atmospheric_data)
{
	return mavlink_msg_uom_atmospheric_data_pack(system_id, component_id, msg, uom_atmospheric_data->roll, uom_atmospheric_data->pitch, uom_atmospheric_data->yaw, uom_atmospheric_data->rollspeed, uom_atmospheric_data->pitchspeed, uom_atmospheric_data->yawspeed, uom_atmospheric_data->time_boot_ms, uom_atmospheric_data->lat, uom_atmospheric_data->lon, uom_atmospheric_data->alt, uom_atmospheric_data->relative_alt, uom_atmospheric_data->vx, uom_atmospheric_data->vy, uom_atmospheric_data->vz, uom_atmospheric_data->airspeed, uom_atmospheric_data->groundspeed, uom_atmospheric_data->heading, uom_atmospheric_data->press_abs, uom_atmospheric_data->press_diff, uom_atmospheric_data->temperature, uom_atmospheric_data->OP1, uom_atmospheric_data->OP2, uom_atmospheric_data->CO);
}

/**
 * @brief Encode a uom_atmospheric_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uom_atmospheric_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uom_atmospheric_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uom_atmospheric_data_t* uom_atmospheric_data)
{
	return mavlink_msg_uom_atmospheric_data_pack_chan(system_id, component_id, chan, msg, uom_atmospheric_data->roll, uom_atmospheric_data->pitch, uom_atmospheric_data->yaw, uom_atmospheric_data->rollspeed, uom_atmospheric_data->pitchspeed, uom_atmospheric_data->yawspeed, uom_atmospheric_data->time_boot_ms, uom_atmospheric_data->lat, uom_atmospheric_data->lon, uom_atmospheric_data->alt, uom_atmospheric_data->relative_alt, uom_atmospheric_data->vx, uom_atmospheric_data->vy, uom_atmospheric_data->vz, uom_atmospheric_data->airspeed, uom_atmospheric_data->groundspeed, uom_atmospheric_data->heading, uom_atmospheric_data->press_abs, uom_atmospheric_data->press_diff, uom_atmospheric_data->temperature, uom_atmospheric_data->OP1, uom_atmospheric_data->OP2, uom_atmospheric_data->CO);
}

/**
 * @brief Send a uom_atmospheric_data message
 * @param chan MAVLink channel to send the message
 *
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @param OP1 Working Electrode voltage in mV
 * @param OP2 Auxiliary Electrode voltage in mV
 * @param CO Measured CO Value in ppb
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uom_atmospheric_data_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, float airspeed, float groundspeed, int16_t heading, float press_abs, float press_diff, int16_t temperature, float OP1, float OP2, float CO)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, rollspeed);
	_mav_put_float(buf, 16, pitchspeed);
	_mav_put_float(buf, 20, yawspeed);
	_mav_put_uint32_t(buf, 24, time_boot_ms);
	_mav_put_int32_t(buf, 28, lat);
	_mav_put_int32_t(buf, 32, lon);
	_mav_put_int32_t(buf, 36, alt);
	_mav_put_int32_t(buf, 40, relative_alt);
	_mav_put_float(buf, 44, airspeed);
	_mav_put_float(buf, 48, groundspeed);
	_mav_put_float(buf, 52, press_abs);
	_mav_put_float(buf, 56, press_diff);
	_mav_put_float(buf, 60, OP1);
	_mav_put_float(buf, 64, OP2);
	_mav_put_float(buf, 68, CO);
	_mav_put_int16_t(buf, 72, vx);
	_mav_put_int16_t(buf, 74, vy);
	_mav_put_int16_t(buf, 76, vz);
	_mav_put_int16_t(buf, 78, heading);
	_mav_put_int16_t(buf, 80, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA, buf, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA, buf, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif
#else
	mavlink_uom_atmospheric_data_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.airspeed = airspeed;
	packet.groundspeed = groundspeed;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.OP1 = OP1;
	packet.OP2 = OP2;
	packet.CO = CO;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.heading = heading;
	packet.temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA, (const char *)&packet, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA, (const char *)&packet, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uom_atmospheric_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, float airspeed, float groundspeed, int16_t heading, float press_abs, float press_diff, int16_t temperature, float OP1, float OP2, float CO)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, rollspeed);
	_mav_put_float(buf, 16, pitchspeed);
	_mav_put_float(buf, 20, yawspeed);
	_mav_put_uint32_t(buf, 24, time_boot_ms);
	_mav_put_int32_t(buf, 28, lat);
	_mav_put_int32_t(buf, 32, lon);
	_mav_put_int32_t(buf, 36, alt);
	_mav_put_int32_t(buf, 40, relative_alt);
	_mav_put_float(buf, 44, airspeed);
	_mav_put_float(buf, 48, groundspeed);
	_mav_put_float(buf, 52, press_abs);
	_mav_put_float(buf, 56, press_diff);
	_mav_put_float(buf, 60, OP1);
	_mav_put_float(buf, 64, OP2);
	_mav_put_float(buf, 68, CO);
	_mav_put_int16_t(buf, 72, vx);
	_mav_put_int16_t(buf, 74, vy);
	_mav_put_int16_t(buf, 76, vz);
	_mav_put_int16_t(buf, 78, heading);
	_mav_put_int16_t(buf, 80, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA, buf, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA, buf, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif
#else
	mavlink_uom_atmospheric_data_t *packet = (mavlink_uom_atmospheric_data_t *)msgbuf;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->rollspeed = rollspeed;
	packet->pitchspeed = pitchspeed;
	packet->yawspeed = yawspeed;
	packet->time_boot_ms = time_boot_ms;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->relative_alt = relative_alt;
	packet->airspeed = airspeed;
	packet->groundspeed = groundspeed;
	packet->press_abs = press_abs;
	packet->press_diff = press_diff;
	packet->OP1 = OP1;
	packet->OP2 = OP2;
	packet->CO = CO;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->heading = heading;
	packet->temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA, (const char *)packet, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA, (const char *)packet, MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE UOM_ATMOSPHERIC_DATA UNPACKING


/**
 * @brief Get field roll from uom_atmospheric_data message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_uom_atmospheric_data_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from uom_atmospheric_data message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_uom_atmospheric_data_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from uom_atmospheric_data message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_uom_atmospheric_data_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rollspeed from uom_atmospheric_data message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_uom_atmospheric_data_get_rollspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitchspeed from uom_atmospheric_data message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_uom_atmospheric_data_get_pitchspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yawspeed from uom_atmospheric_data message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_uom_atmospheric_data_get_yawspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field time_boot_ms from uom_atmospheric_data message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_uom_atmospheric_data_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field lat from uom_atmospheric_data message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_uom_atmospheric_data_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field lon from uom_atmospheric_data message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_uom_atmospheric_data_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field alt from uom_atmospheric_data message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 */
static inline int32_t mavlink_msg_uom_atmospheric_data_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field relative_alt from uom_atmospheric_data message
 *
 * @return Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_uom_atmospheric_data_get_relative_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  40);
}

/**
 * @brief Get field vx from uom_atmospheric_data message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_uom_atmospheric_data_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  72);
}

/**
 * @brief Get field vy from uom_atmospheric_data message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_uom_atmospheric_data_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  74);
}

/**
 * @brief Get field vz from uom_atmospheric_data message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_uom_atmospheric_data_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  76);
}

/**
 * @brief Get field airspeed from uom_atmospheric_data message
 *
 * @return Current airspeed in m/s
 */
static inline float mavlink_msg_uom_atmospheric_data_get_airspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field groundspeed from uom_atmospheric_data message
 *
 * @return Current ground speed in m/s
 */
static inline float mavlink_msg_uom_atmospheric_data_get_groundspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field heading from uom_atmospheric_data message
 *
 * @return Current heading in degrees, in compass units (0..360, 0=north)
 */
static inline int16_t mavlink_msg_uom_atmospheric_data_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  78);
}

/**
 * @brief Get field press_abs from uom_atmospheric_data message
 *
 * @return Absolute pressure (hectopascal)
 */
static inline float mavlink_msg_uom_atmospheric_data_get_press_abs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field press_diff from uom_atmospheric_data message
 *
 * @return Differential pressure 1 (hectopascal)
 */
static inline float mavlink_msg_uom_atmospheric_data_get_press_diff(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field temperature from uom_atmospheric_data message
 *
 * @return Temperature measurement (0.01 degrees celsius)
 */
static inline int16_t mavlink_msg_uom_atmospheric_data_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  80);
}

/**
 * @brief Get field OP1 from uom_atmospheric_data message
 *
 * @return Working Electrode voltage in mV
 */
static inline float mavlink_msg_uom_atmospheric_data_get_OP1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field OP2 from uom_atmospheric_data message
 *
 * @return Auxiliary Electrode voltage in mV
 */
static inline float mavlink_msg_uom_atmospheric_data_get_OP2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field CO from uom_atmospheric_data message
 *
 * @return Measured CO Value in ppb
 */
static inline float mavlink_msg_uom_atmospheric_data_get_CO(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Decode a uom_atmospheric_data message into a struct
 *
 * @param msg The message to decode
 * @param uom_atmospheric_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_uom_atmospheric_data_decode(const mavlink_message_t* msg, mavlink_uom_atmospheric_data_t* uom_atmospheric_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	uom_atmospheric_data->roll = mavlink_msg_uom_atmospheric_data_get_roll(msg);
	uom_atmospheric_data->pitch = mavlink_msg_uom_atmospheric_data_get_pitch(msg);
	uom_atmospheric_data->yaw = mavlink_msg_uom_atmospheric_data_get_yaw(msg);
	uom_atmospheric_data->rollspeed = mavlink_msg_uom_atmospheric_data_get_rollspeed(msg);
	uom_atmospheric_data->pitchspeed = mavlink_msg_uom_atmospheric_data_get_pitchspeed(msg);
	uom_atmospheric_data->yawspeed = mavlink_msg_uom_atmospheric_data_get_yawspeed(msg);
	uom_atmospheric_data->time_boot_ms = mavlink_msg_uom_atmospheric_data_get_time_boot_ms(msg);
	uom_atmospheric_data->lat = mavlink_msg_uom_atmospheric_data_get_lat(msg);
	uom_atmospheric_data->lon = mavlink_msg_uom_atmospheric_data_get_lon(msg);
	uom_atmospheric_data->alt = mavlink_msg_uom_atmospheric_data_get_alt(msg);
	uom_atmospheric_data->relative_alt = mavlink_msg_uom_atmospheric_data_get_relative_alt(msg);
	uom_atmospheric_data->airspeed = mavlink_msg_uom_atmospheric_data_get_airspeed(msg);
	uom_atmospheric_data->groundspeed = mavlink_msg_uom_atmospheric_data_get_groundspeed(msg);
	uom_atmospheric_data->press_abs = mavlink_msg_uom_atmospheric_data_get_press_abs(msg);
	uom_atmospheric_data->press_diff = mavlink_msg_uom_atmospheric_data_get_press_diff(msg);
	uom_atmospheric_data->OP1 = mavlink_msg_uom_atmospheric_data_get_OP1(msg);
	uom_atmospheric_data->OP2 = mavlink_msg_uom_atmospheric_data_get_OP2(msg);
	uom_atmospheric_data->CO = mavlink_msg_uom_atmospheric_data_get_CO(msg);
	uom_atmospheric_data->vx = mavlink_msg_uom_atmospheric_data_get_vx(msg);
	uom_atmospheric_data->vy = mavlink_msg_uom_atmospheric_data_get_vy(msg);
	uom_atmospheric_data->vz = mavlink_msg_uom_atmospheric_data_get_vz(msg);
	uom_atmospheric_data->heading = mavlink_msg_uom_atmospheric_data_get_heading(msg);
	uom_atmospheric_data->temperature = mavlink_msg_uom_atmospheric_data_get_temperature(msg);
#else
	memcpy(uom_atmospheric_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_UOM_ATMOSPHERIC_DATA_LEN);
#endif
}
