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
