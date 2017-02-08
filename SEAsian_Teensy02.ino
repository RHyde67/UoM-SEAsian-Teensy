
#include "libraries\checksum.h"
#include "libraries\ardupilotmega\mavlink.h" // loads mavlink from common
#include <ADC.h>
#include <ADC_Module.h>
#include <RingBuffer.h>
#include <RingBufferDMA.h>
#include <SD.h> //Load SD card library
#include <SPI.h> //Load SPI Library
//#include "mavlink\pixhawk\pixhawk.h" // introduces many errors and slow build

// set this to the hardware serial port you wish to use
#define APSERIAL Serial1 //Autopilot Port RX(pin0) TX(pin1) 
#define TELEMSERIAL Serial3 //Telemetry Serial RX(pin7) TX(pin8)
#define toDeg(x) ((x)*57.2957795131) // *180/pi 

// ADC variables
const int OP1pin = A2; //
const int OP2pin = A3; //
ADC *adc = new ADC();
ADC::Sync_result result;

// Autopilot variables
int PixSys_id = 1;
int PixComponent_id = 1;
int TeensySys_id = 127;
int TeensyComponent_id = 1;
float roll = 0;
float pitch = 0;
float yaw = 0;
float rollspeed = 0;
float pitchspeed = 0;
float yawspeed = 0;
uint time_boot_ms;
int lat = 0;
int lon = 0;
int alt = 0;
int relative_alt = 0;
int vx = 0;
int vy = 0;
int vz = 0;
int heading = 0;
float airspeed = 0;
float groundspeed = 0;
float verticalspeed = 0;
float press_abs = 0;
float press_diff = 0;
int temperature = 0;
bool GCS_UNITS = 0; // 0 = meters, 1 = feet

// CO Sensor Variables
float CalibVal = 0.439; // Calibration value to convert from voltage into ppb, varies by sensor
float OP1 = 0; // sensor output 1
float OP2 = 0; // sensor output 2
float CO = 0; // CO reading computed from OP1 & OP2
int CO_Count = 0; // number of CO samples
float CO_Mean = 0; // Recursively calculated mean for CO readings between SD writes
const long SD_Write_Period = 100; // millisecond between SD card writes

// Raw GPS variables
uint64_t mav_utime; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
int32_t latraw; /*< Latitude (WGS84), in degrees * 1E7*/
int32_t lonraw; /*< Longitude (WGS84), in degrees * 1E7*/
int32_t altraw; /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
uint16_t eph; /*< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
uint16_t epv; /*< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
uint16_t grndvelraw; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
uint8_t gpsfix; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
uint8_t numSats; /*< Number of satellites visible. If unknown, set to 255*/

// Telemetry variables
unsigned long PrevTelemTime = 0;        // will store last time atmospheric data packet was sent
const long TelemPeriod = 500;           // interval at which to send new atmospheric data packet (milliseconds) >250 for continuous reliable op.

// SD Card variables
int chipSelect = 4; //chipSelect pin for the SD card Reader
File mySensorData; //Data object you will write your sesnor data to
unsigned long lognum = 0;
bool SD_Connected = 0;

// Generic Variables
int PinLED = 13; // LED pin to allow flashing


void setup() {
	// The setup code will run once
	pinMode(PinLED, OUTPUT); // set LED pin mode
	digitalWrite(13, HIGH); // Light LED during setup

	TELEMSERIAL.begin(57600);
	APSERIAL.begin(57600);

	pinMode(10, OUTPUT); //Must declare pin10 as an output and reserve it
	SD_Connected = SD.begin(BUILTIN_SDCARD); //Initialize the SD card reader
	lognum = millis();// can add a lastlog file on sd card to keep track if needed

	// ADC Setup

	// ADC0
	pinMode(OP1pin, INPUT); // configuring OP1pin as input
	pinMode(OP2pin, INPUT); // configuring OP2pin as input
							// Initializing serial ports
							//Serial.begin(9600);
							///// ADC0 ////
							// reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
							//adc->setReference(ADC_REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2
	adc->setAveraging(10); // set number of samples to be taken and averaged to give a reading
	adc->setResolution(12); // set bits of resolution
	adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS); // change the conversion speed it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
	adc->setSamplingSpeed(ADC_VERY_LOW_SPEED); // change the sampling speed
	adc->setReference(ADC_REF_3V3); // set the external reference pin as the voltage reference
	// ADC1
	adc->setAveraging(10, ADC_1); // set number of averages
	adc->setResolution(12, ADC_1); // set bits of resolution
	adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS, ADC_1); // change the conversion speed
	adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1); // change the sampling speed
	adc->setReference(ADC_REF_3V3, ADC_1);

	adc->startSynchronizedContinuous(OP1pin, OP2pin);
	
	digitalWrite(13, LOW); // turn off LED after setup completes
}



void loop() {
	// put your main code here, to run repeatedly:  
	//Serial.println("Running...");
	//int incomingByte;
	int outgoingByte;
	unsigned long currentMillis = millis();

	mavlink_message_t msg1;


	if (APSERIAL.available() > 0) {
		// Send request for data
		receive_msg(); // receive msg first to identify AP
		mavlink_msg_request_data_stream_pack(TeensySys_id, TeensyComponent_id, &msg1, PixSys_id, PixComponent_id, MAV_DATA_STREAM_ALL, 20, 1); // build data request msg
		send_message(&msg1); // send data request msg
	}

	Read_Sensor(); // read sensor value and return recursive mean since last save
	
	if (currentMillis - PrevTelemTime >= TelemPeriod && PixSys_id>0) {
		digitalWrite(PinLED, HIGH); // LED on
		PrevTelemTime = currentMillis; // save the last time you recorded an atmospheric measurement
		Send_Telem();
		// Save the information sent to GCS in the mavlink packet on the SD card
		if (SD_Connected) { //log the data only if the SD card is connected
		SD_write();
		}
		CO_Count = 0;
		CO_Mean = 0;
		digitalWrite(PinLED, LOW); // LED off
	}


	//Take the GCS inputs from the telemetry module and transfer them to the AP port
	if (TELEMSERIAL.available() > 0) {
		outgoingByte = TELEMSERIAL.read();
		APSERIAL.write(outgoingByte);
	}

}

void send_message(mavlink_message_t* msg) //send data bit by bit
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	for (uint16_t i = 0; i < len; i++)
	{
		APSERIAL.write(buf[i]);
	}
}

void receive_msg()
{ //receive data over serial from AP module
	while (APSERIAL.available() > 0)
	{
		mavlink_message_t msg;
		mavlink_status_t status;
		uint8_t rec = APSERIAL.read();//Show bytes sent from the pixhawk
		mavlink_parse_char(MAVLINK_COMM_0, rec, &msg, &status);
		handleMessage(&msg);

	}
}

void handleMessage(mavlink_message_t* msg) //handle the messages and decode to variables
{

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_HEARTBEAT");
		mavlink_heartbeat_t packet;
		mavlink_msg_heartbeat_decode(msg, &packet);
		uint8_t beat = 1;
		if ((*msg).sysid != 0xff) { // do not process mission planner heartbeats if we have two receiver xbees
			uint8_t PixSys_id = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
			uint8_t PixComponent_id = (*msg).compid;
			uint8_t bmode = packet.base_mode; // unused
			uint8_t cmode = packet.custom_mode; // unused
		}
		break;
	}
	case MAVLINK_MSG_ID_ATTITUDE:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_ATTITUDE");
		// decode
		mavlink_attitude_t packet;
		mavlink_msg_attitude_decode(msg, &packet);
		pitch = toDeg(packet.pitch);
		yaw = toDeg(packet.yaw);
		roll = toDeg(packet.roll);
		rollspeed = packet.rollspeed;
		pitchspeed = packet.pitchspeed;
		yawspeed = packet.yawspeed;
		time_boot_ms = packet.time_boot_ms;
		break;
	}
	case MAVLINK_MSG_ID_GPS_RAW_INT:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_GPS_RAW_INT");
		// decode
		mavlink_gps_raw_int_t packet;
		mavlink_msg_gps_raw_int_decode(msg, &packet);
		gpsfix = packet.fix_type;
		mav_utime = packet.time_usec;
		numSats = packet.satellites_visible;
		cog = packet.cog;
		break;
	}
	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_GLOBAL_POSITION_INT");
		// decode
		mavlink_global_position_int_t packet;
		mavlink_msg_global_position_int_decode(msg, &packet);
		lat = packet.lat;
		lon = packet.lon;
		alt = packet.alt;
		relative_alt = packet.relative_alt;
		vx = packet.vx;
		vy = packet.vy;
		vz = packet.vz;
		time_boot_ms = packet.time_boot_ms;
		if (GCS_UNITS == 0) alt = packet.alt / 1000;
		else if ((GCS_UNITS == 1) || (GCS_UNITS == 2)) alt = (packet.alt / 1000) * 3.28084;
		break;
	}
	case MAVLINK_MSG_ID_GPS_STATUS:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_GPS_STATUS");
		mavlink_gps_status_t packet;
		mavlink_msg_gps_status_decode(msg, &packet);
		break;
	}
	case MAVLINK_MSG_ID_VFR_HUD:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_VFR_HUD");
		mavlink_vfr_hud_t packet;
		mavlink_msg_vfr_hud_decode(msg, &packet);
		heading = packet.heading;
		if (GCS_UNITS == 0) airspeed = packet.airspeed * 3.6;
		else if (GCS_UNITS == 1) airspeed = packet.airspeed * 2.24;
		else if (GCS_UNITS == 2) airspeed = packet.airspeed * 1.94;
		if (GCS_UNITS == 0) groundspeed = packet.groundspeed * 3.6;
		else if (GCS_UNITS == 1) groundspeed = packet.groundspeed * 2.24;
		else if (GCS_UNITS == 2) groundspeed = packet.groundspeed * 1.94;
		if (GCS_UNITS == 0) verticalspeed = packet.climb;
		else if ((GCS_UNITS == 1) || (GCS_UNITS == 2)) verticalspeed = packet.climb * 3.28084;
		break;
	}
	case MAVLINK_MSG_ID_RAW_PRESSURE:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_RAW_PRESSURE");
		// decode
		mavlink_raw_pressure_t packet;
		mavlink_msg_raw_pressure_decode(msg, &packet);
		break;
	}

	case MAVLINK_MSG_ID_SCALED_PRESSURE:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_SCALED_PRESSURE");
		mavlink_scaled_pressure_t packet;
		mavlink_msg_scaled_pressure_decode(msg, &packet);
		press_abs = packet.press_abs; // hectopascals
		press_diff = packet.press_diff; // hectopascals
		temperature = packet.temperature; // temp in 0.01 deg C
		time_boot_ms = packet.time_boot_ms;
		break;
	}

	case MAVLINK_MSG_ID_SYS_STATUS:
	{
		//Serial.println("Received: MAVLINK_MSG_ID_SYS_STATUS");
		//mavlink_sys_status_t packet;
		//mavlink_msg_sys_status_decode(msg, &packet);
		//vbat = packet.voltage_battery;
		break;
	}
	}

}

void SD_write() {
	//Serial.println("Writing to SD");
	mySensorData = SD.open("Log.txt", FILE_WRITE);
	if (mySensorData) {

		mySensorData.println("Time_boot_ms " + String(time_boot_ms) + ", Roll " + String(roll) + ", Pitch " + String(pitch) + ", Yaw " + String(yaw) + ", RollSpeed " + String(rollspeed)
			+ ", PitchSpeed " + String(pitchspeed) + ", YawSpeed" + String(yawspeed) +
			", Lat " + String(lat) + ", Lon " + String(lon) + ", Alt " + String(alt) + ", RelativeAlt " + String(relative_alt) +
			", vx " + String(vx) + ", vy " + String(vy) + ", vz" + String(vz) + ", Heading " + String(heading) +
			", Airspeed " + String(airspeed) + ", GroundSpeed " + String(groundspeed) + ", ClimbSpeed " + String(verticalspeed) +
			", Press_Abs " + String(press_abs) + ", PressDiff " + String(press_diff) + ", Temperature " + String(temperature) +
			", OP1 " + String(OP1) + ", OP2 " + String(OP2) + ", CO " + String(CO_Mean));

		mySensorData.close();                                  //close the file
	}
}

void Read_Sensor() {
	// result = adc->analogSynchronizedRead(OP1pin, OP2pin); // should be this line? why was it changed?
	result = adc->readSynchronizedContinuous();
	// if using 16 bits and single-ended is necessary to typecast to unsigned,
	// otherwise values larger than 3.3/2 will be interpreted as negative
	result.result_adc0 = (uint16_t)result.result_adc0;
	result.result_adc1 = (uint16_t)result.result_adc1;

	//ADC::Sync_result result = adc->analogSynchronizedRead(OP1pin, OP2pin);
	//Modify the calibration value according to the sensor you are using
	
	// calculate the (fraction of the Vin range) times (what that range represents in sensor V) divided by (calibration value)
	OP1 = ((((float)result.result_adc0 / adc->getMaxValue(ADC_0)) * 5) / CalibVal); //Converting the raw bit value into ppb equivalent 
	OP2 = ((((float)result.result_adc1 / adc->getMaxValue(ADC_1)) * 5) / CalibVal); //Converting the raw bit value into ppb equivalent

	CO = OP1 - OP2; // Calculate the CO value in ppb
	if (CO >= 0){ // error check CO value
		++CO_Count;
		CO_Mean = ((CO_Mean * CO_Count - 1) + CO) / CO_Count; // recursive mean of CO
	}
}

void Send_Telem() {
	//Send2Ground();
	// Initialize the required buffers 
	mavlink_message_t atm;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	// Pack the new message with our data attached
	mavlink_msg_uom_atmospheric_data_pack(TeensySys_id, TeensyComponent_id, &atm,
		roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, time_boot_ms, lat, lon,
		alt, relative_alt, vx, vy, vz, airspeed, groundspeed, heading, press_abs,
		press_diff, temperature, OP1, OP2, CO_Mean);
												  // Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &atm);

	// Send the uom_atmospheric_data_pack to the GCS through the telemetry module 
	TELEMSERIAL.write(buf, len);
}