
#ifndef MS5837_H_INCLUDED
#define MS5837_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

enum ms5837_resolution_osr {
	ms5837_resolution_osr_256 = 0,
	ms5837_resolution_osr_512,
	ms5837_resolution_osr_1024,
	ms5837_resolution_osr_2048,
	ms5837_resolution_osr_4096,
	ms5837_resolution_osr_8192
};

enum ms5837_status {
	ms5837_status_ok,
	ms5837_status_no_i2c_acknowledge,
	ms5837_status_i2c_transfer_error,
	ms5837_status_crc_error
};


void ms5837_init(void);


bool ms5837_is_connected(void);


enum ms5837_status ms5837_reset(void);


void ms5837_set_resolution(enum ms5837_resolution_osr );


enum ms5837_status ms5837_read_temperature_and_pressure(float *, float *);

#endif /* MS5837_H_INCLUDED */
