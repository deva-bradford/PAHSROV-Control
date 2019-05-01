#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <math.h>
#include <limits.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "bno055.h"

typedef unsigned char byte;
char *filename = (char*)"/dev/i2c-1";
int fd = open(filename, O_RDWR );

BNO055::BNO055(int32_t sensorID, uint8_t address){
	_sensorID = sensorID;
	_address = address;
}

int BNO055::write_bytes(int file, uint8_t address, uint8_t *data, uint8_t count){
	struct i2c_rdwr_ioctl_data packets;  // ioctl に渡す I2C_RDWR 用の変数の構造体
	struct i2c_msg messages[1];          // ioctl で送り出すデータ変数の型の配列

	messages[0].addr = address;  // スレーブアドレス
	messages[0].flags = 0;       // 書込みを表す
	messages[0].len = count;     // data に含まれるバイト数
	messages[0].buf = data;      // 書込みたい byte data の配列

	packets.msgs = messages;
	packets.nmsgs = 1;

	return ioctl(file, I2C_RDWR, &packets) >= 0;  // ioctl で書込み処理をしてから return
}

int BNO055::write_byte(int file, uint8_t address, uint8_t reg, uint8_t data){
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = data;
	return write_bytes(file, address, buf, 2);
}

int BNO055::read_bytes(int file, uint8_t address, uint8_t reg, uint8_t *dest, uint8_t count){
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages[2];

					   /* write the register we want to read from */
	messages[0].addr = address;   // スレーブアドレス
	messages[0].flags = 0;        // 書込みを表す
	messages[0].len = 1;          // データ配列の長さ
	messages[0].buf = &reg;       // 内部レジスタをデータとして書き込む

								  /* read */
	messages[1].addr = address;   // もう一度スレーブアドレス
	messages[1].flags = I2C_M_RD; // 今度は読込み
	messages[1].len = count;      // 読込みバイト数
	messages[1].buf = dest;       // 読み込んだデータの書込み先

	packets.msgs = messages;
	packets.nmsgs = 2;            // 今回は内部レジスタの書込みとデータの読込みで message は2個

	return ioctl(file, I2C_RDWR, &packets) >= 0;
}

int BNO055::read_byte(int file, uint8_t address, uint8_t reg, uint8_t *dest){
	return read_bytes(file, address, reg, dest, 1);
}

bool BNO055::readLen(adafruit_bno055_reg_t reg, byte * buffer, uint8_t len){ //TODO:data[0]にする{
	read_bytes(fd, _address, reg, &buffer[0], len);

	//あやしい
	/* ToDo: Check for errors! */
	return true;
}

bool BNO055::write8(adafruit_bno055_reg_t reg, byte value){
	write_byte(fd, _address, (uint8_t)reg, (uint8_t)value);
	/* ToDo: Check for error! */
	return true;
}

byte BNO055::read8(adafruit_bno055_reg_t reg){
	byte val = 0;
	read_byte(fd, _address, reg, &val);
	return val;
}

void BNO055::setMode(adafruit_bno055_opmode_t mode){
	_mode = mode;
	write8(BNO055_OPR_MODE_ADDR, _mode);
	delay(30);
}

bool BNO055::begin(adafruit_bno055_opmode_t mode){
	if ((fd = open ("/dev/i2c-1", O_RDWR)) < 0)
		return -1;
	if (ioctl (fd, I2C_SLAVE, _address) < 0)
		return -1;

	uint8_t id = read8(BNO055_CHIP_ID_ADDR);
	if (id != BNO055_ID)
	{
		delay(1000); // hold on for boot
		id = read8(BNO055_CHIP_ID_ADDR);
		if (id != BNO055_ID) {
			return false;  // still not? ok bail
		}
	}
	setMode(OPERATION_MODE_CONFIG);
	write8(BNO055_SYS_TRIGGER_ADDR,0x20);
	delay(1000);
	
	while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID){delay(10);} // hold on for boot
	
	write8(BNO055_PWR_MODE_ADDR,POWER_MODE_NORMAL);
	write8(BNO055_PAGE_ID_ADDR,0);
	write8(BNO055_SYS_TRIGGER_ADDR,0);
	delay(10);
	setMode(OPERATION_MODE_NDOF);
	return true;
}

void BNO055::setExternalCrystalUse(bool enable){
	adafruit_bno055_opmode_t now = _mode;
	setMode(OPERATION_MODE_CONFIG);
	delay(20);
	write8(BNO055_PAGE_ID_ADDR,0);
	if(enable)
		write8(BNO055_SYS_TRIGGER_ADDR,0x80);
	else
		write8(BNO055_SYS_TRIGGER_ADDR,0);
	delay(10);
	setMode(now);
	delay(20);
}

int8_t BNO055::readTemp(){
	int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
	return temp;
}

void BNO055::readVector(adafruit_vector_type_t type,double data[3]){
	int16_t x, y, z;
	x = y = z = 0;

	/* Read vector data (6 bytes) */
	//readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);
	uint8_t buffer[6] = {0};
	memset(buffer, 0, 6);
	read_bytes (fd, _address, (adafruit_bno055_reg_t)type, &buffer[0], 6);
	printf("0:%x 1:%x 2:%x 3:%x 4:%x 5:%x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);
	x = ((uint8_t)buffer[0]) | (buffer[1] << 8);
	y = ((uint8_t)buffer[2]) | (buffer[3] << 8);
	z = ((uint8_t)buffer[4]) | (buffer[5] << 8);
	switch (type)
	{
	case VECTOR_MAGNETOMETER:
		/* 1uT = 16 LSB */
		data[0] = ((double)x) / 16.0;
		data[1] = ((double)y) / 16.0;
		data[2] = ((double)z) / 16.0;
		break;
	case VECTOR_GYROSCOPE:
		/* 1rps = 900 LSB */
		data[0] = ((double)x) / 900.0;
		data[1] = ((double)y) / 900.0;
		data[2] = ((double)z) / 900.0;
		break;
	case VECTOR_EULER:
		/* 1 degree = 16 LSB */
		data[0] = ((double)x) / 16.0;
		data[1] = ((double)y) / 16.0;
		data[2] = ((double)z) / 16.0;
		break;
	case VECTOR_ACCELEROMETER:
	case VECTOR_LINEARACCEL:
	case VECTOR_GRAVITY:
		/* 1m/s^2 = 100 LSB */
		data[0] = ((double)x) / 100.0;
		data[1] = ((double)y) / 100.0;
		data[2] = ((double)z) / 100.0;
		break;
	}
}

void BNO055::readEuler(double dat[3]){
	readVector(VECTOR_EULER,dat);
}
