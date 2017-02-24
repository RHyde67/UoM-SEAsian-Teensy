Ver05
Fixed compile error in Arduino.
Hex file included. This will provide the same Teensy system and component ID to every system, which in turn labels all log files the same. Ideally the system ID should be changed for each Teensy.

Ver04
General code tidy, superfluous, historical, remarked code removed.
Changes:
1.
GPS Time & Date acquired.
Log file is named with 3 digit System ID, 'Log' and 2 digit Log File Number, nnnLogmm.csv	This allows easy use of multiple files. I tried using long filenames in the format YYYY-MM-DD_HH-MM-SS_Sys-ID.csv. Worked right up the point I realised we're limited to 8.3 file names!
Log file now includes the date stamp and time stamp in the first two columns.

2.
Further boot error checking added, the system checks for:
a. availability of the AP, v.fast flashes
b. receipt of Mavlink msgs containing GPS data, fast flashes
c. then sanity checks the year of the timestamp, i.e. not 1970, medium flashes.

Ver03
minor code tidying
Proper conversion from sensor Voltage in to ppb using zero offset and multiplier value from the sensor calibration certificate.
Moved Mavlink message request and added error checking for Autopilot during set up. This is now a separate function. To prevent SD card writes before AP is initialized and sending data, during setup the system checks for the AP with the following LED flashes and Serial.println messages:
1. Setup, LED on steady
2. AP setup, searching for AP, very fast blink, serial print 'Searching for Auto Pilot....'
3. serial print 'Auto Pilot found'
3. Sends mavlink message request
4. receives message and checks for Lat not equal to zero, fast blink, serial print 'Auto Pilot intializing....'
5. Serial print 'Auto Pilot online'
6. LED returns to steady on, setup continues
when running, single flash for telemetry, single flash for SD write, looks like dual flash under normal circumstances.



ver02
Various code tidying
SD Card setup moved to function
ADC setup moved to function
Fixed bug with recursive mean.
Fixed bug where sensore mean calcualtion variables are not accessible globally, these are now passed to, and returned from, the function directly.
Data is written to a *.csv (previsouly *.TXT) file.
Data file contans headers for columns, with subsequent writes being data values only without the text descrition for each value.
(Line added to account for reversing of sensor wires. May not be a good idea as it takes the abs value of CO1-CO2 and we have seen -ve values when correctly connected. This may disguise erroneous readings)

ver01:
Set sensor sampling rate to 'very_low_speed'
Set sensor ref voltage to 3.3V (internal)
Read_Sensor now calculates a recursive mean, if the reading >0, until the data is written. This results in a mean value between data writes and transmits.
Data is written at the same time as it's transmitted by radio.
LED flashes on before write/ send, off on completion. Previsoulsy this went on with write, off with next write.
