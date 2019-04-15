#include <unistd.h>				//Needed for I2C port
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>				//Needed for I2C port
//#include <math.h>
#include <sys/ioctl.h>			//Needed for I2C port
#include </usr/include/linux/i2c-dev.h>		//Needed for I2C port
#include "ms5837.h" //FIX INITS STUFF!
#include <wiringPi.h>
#include <wiringPiI2C.h>

// compile with: gcc -o "i2ctmp2" "i2ctps-rjb.c" -lwiringPi -lm
// execute with: sudo ./i2ctmp2

int file_i2c, fd;
char *filename = (char*)"/dev/i2c-1";
int Reset = 0x1E; // reset address
int PROMs[7] = {0xA0,0xA2,0xA4,0xA6,0xA8,0xAA,0xAC}; // calibration addresses
int desired_values[7]={16385,33668,31548,19947,19751,27317,26370}; // desired calibration values for comparison
uint8_t buffer_read[2]={0};
uint8_t res;
int length,i;

void PROM_read();
void Conversion();
void Temp_and_pressure_read();

#define MS5837_RESET_COMMAND										0x1E
#define MS5837_START_PRESSURE_ADC_CONVERSION						0x4A
#define MS5837_START_TEMPERATURE_ADC_CONVERSION						0x5A
#define MS5837_READ_ADC												0x00


#define MS5837_CONVERSION_OSR_MASK									0x0F

#define MS5837_CONVERSION_TIME_OSR_256								1000
#define MS5837_CONVERSION_TIME_OSR_512								2000
#define MS5837_CONVERSION_TIME_OSR_1024								3000
#define MS5837_CONVERSION_TIME_OSR_2048								5000
#define MS5837_CONVERSION_TIME_OSR_4096								9000
#define MS5837_CONVERSION_TIME_OSR_8192								17000

#define MS5837_CRC_INDEX											0
#define MS5837_PRESSURE_SENSITIVITY_INDEX							1
#define MS5837_PRESSURE_OFFSET_INDEX								2
#define MS5837_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX				3
#define MS5837_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX					4
#define MS5837_REFERENCE_TEMPERATURE_INDEX							5
#define MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX						6
#define MS5837_COEFFICIENT_NUMBERS									7

enum ms5837_resolution_osr ms5837_resolution_osr;
static uint16_t eeprom_coeff[MS5837_COEFFICIENT_NUMBERS+1];
static uint32_t conversion_time[6] = {	MS5837_CONVERSION_TIME_OSR_256,
										MS5837_CONVERSION_TIME_OSR_512,
										MS5837_CONVERSION_TIME_OSR_1024,
										MS5837_CONVERSION_TIME_OSR_2048,
										MS5837_CONVERSION_TIME_OSR_4096,
										MS5837_CONVERSION_TIME_OSR_8192};

void ms5837_init(void)
{
	ms5837_resolution_osr = ms5837_resolution_osr_8192;

										    /* Initialize and enable device with config. */
}


void ms5837_set_resolution(enum ms5837_resolution_osr res)
{
	ms5837_resolution_osr = res;
return;
}

int main() {
	//----- OPEN THE I2C BUS -----
	fd = open(filename, O_RDWR );
	if ((file_i2c = fd) < 0) //Initialization
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return 0;
	}

	int addr = 0x76;          //<<<<<The I2C address of the slave
	if (ioctl(fd, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return 0;
	}

    length = 1; // number of bytes to write
	if(write(fd,&Reset,length)!=length)  //write reset address
        fprintf( stderr, "Failed to write to I2C device: %m\n" );

	sleep(0.01); //reset delay 10 ms

	// loop through each calibration value and compare with desired value

  for(i = 0; (i<7); i++)
    {
		length = 1; // number of bytes to write
		if(write(fd,&PROMs[i],length)!=length) // write address to get first calibration value
			fprintf( stderr, "Failed to write %d to I2C device: %m\n", 0xA0);

		length = 2; // number of bytes to read
		if(read(fd,buffer_read,length)!=length) // read 2 bytes (first calibration value)
			printf("Failed to read from I2C\n");

		//printf("Data read, MSB: %d LSB: %d\n",buffer_read[0],buffer_read[1]);

		uint16_t data = buffer_read[0]<<8 | buffer_read[1]; // merge MSB and LSB data
		//Print results
		printf("Calibration Data for Address 0x%x: %d (desired = %d)\n",PROMs[i],data,desired_values[i]);
    }
    //printf("pre-Checkpoint\n");
	while (1)
		Temp_and_pressure_read();
	return 0;
}

void Conversion_and_Read(uint8_t res, uint32_t *adc) {
	uint8_t TempData[3];
	int ConversionAddr;
	//int Conversions[6] = {0x40,0x42,0x44,0x46,0x48,0x4A};
	TempData[0] = 0;
	TempData[1] = 0;
	TempData[2] = 0;

	int conversionMask = 0x0F;

	write(fd, &res, 1);

	delay(conversion_time[ (res & conversionMask)/2 ]/1000);

	int adcread = 0x00;
	write( fd, &adcread, 1);

	read(fd, &TempData, 3);

	*adc = ((uint32_t)TempData[0] << 16) | ((uint32_t)TempData[1]) << 8 | TempData[2];
}

void Temp_and_pressure_read() {
	uint32_t temp, pres, D2, D1;
	int32_t dT, TEMP;
	int64_t OFF, SENS, P, T2, OFF2, SENS2;
	float temperature, pressure;
	uint32_t SENSi, OFFi, Ti;
	int density;

	//Adresses
	int adctempconvert = 0x50;
	int adcpresconvert = 0x40;
	density = 1029;


	res = ms5837_resolution_osr*2;
	res |= adctempconvert;

	Conversion_and_Read(res, &temp);

	res = ms5837_resolution_osr*2;
	res |= adcpresconvert;

	Conversion_and_Read(res, &pres);

	printf("\rRaw Temp: %d\nRaw Pres: %d\n", temp, pres);


	D2 = temp;
	D1 = pres;

	dT=D2-(desired_values[5]*256);
	SENS=desired_values[1]*32768+(desired_values[3]*dT)/256;
	OFF=desired_values[2]*65536+(desired_values[4]*dT)/128;
	P=(D1*SENS/2097152-OFF)/8192;

	TEMP = 2000+(dT*desired_values[6])/8388608;
	if ( (TEMP/100)<20)
	{
	  Ti = 3*dT*dT/8589934592.0;
	  OFFi=(3*(TEMP-2000)*(TEMP-2000))/2;
	  SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
	  if((TEMP/100)<-15)
	  {
	    OFFi=OFFi+7*(TEMP+1500)*(TEMP+1500);
	    SENSi=SENSi+(4*(TEMP+1500)*(TEMP+1500));
	  }
	}
	else if((TEMP/100)>=20)
	{
	  Ti=(2*dT*dT)/137438953472.0;
	   OFFi = ((TEMP-2000)*(TEMP-2000))/16;
	   SENSi = 0;
	}
	OFF2=OFF-OFFi;
	SENS2=SENS-SENSi;
	TEMP=TEMP-Ti;
	P=(((D1*SENS2)/2097152-OFF2)/8192)/10.0;
	TEMP = TEMP/100.0;

	int depth = (P*100-101300)/(density*9.80665);
	int alt = (1-pow(P/1013.25,.190284))*145366.45*.3048;


/*
Deprecated Secondary Algorithm
	dT = (int32_t)temp - ((int32_t)eeprom_coeff[MS5837_REFERENCE_TEMPERATURE_INDEX] << 8);
		// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000 + ((int64_t)dT * (int64_t)eeprom_coeff[MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;
		// Second order temperature compensation
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
		OFF2 = 3 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 2 ;
		SENS2 = 5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 8 ;
		if( TEMP < -1500 )
		{
			OFF2 += 7 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 4 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 2 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 37;
		OFF2 = ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) >> 4;
		SENS2 = 0 ;
	}
		// OFF = OFF_T1 + TCO * dT
	OFF = ( (int64_t)(eeprom_coeff[MS5837_PRESSURE_OFFSET_INDEX]) << 16 ) + ( ( (int64_t)(eeprom_coeff[MS5837_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 7 ) ;
	OFF -= OFF2 ;
		// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	SENS = ( (int64_t)eeprom_coeff[MS5837_PRESSURE_SENSITIVITY_INDEX] << 15 ) + ( ((int64_t)eeprom_coeff[MS5837_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 8 ) ;
	SENS -= SENS2 ;
		// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( (pres * SENS) >> 21 ) - OFF ) >> 13 ;
	temperature = ( (float)TEMP - T2 ) / 100;
	pressure = (float)P / 100;
*/

	printf("\r\nTemp: %d\nPres: %d\n", TEMP, P);
}
