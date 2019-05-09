#include "joy.h"
#include "updates.h"
#include "pca9685.h"
#include <iostream>
#include <cstdlib>
#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using namespace std;

#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50

int file_12c, fd, AJ, J;
char *filename = (char*)"/dev/i2c-1";

//g++ -o control joy.cpp updates.cpp test.cpp -g -O3 -Wall -Wextra -Wpedantic -std=c++11 -lwiringPi -lwiringPiPca9685

struct dual {
	int LFM;
	int RFM;
};

dual pack;

int calcTick(float impulseMs, int hertz) {
	float offset = 0.5f;
	float cycleMs = 1000.0f / hertz;
	return (int)(MAX_PWM * impulseMs / cycleMs + offset);
}

dual xresolver(int trigstate, int joystate) { //add function to neutralize the motor before FWB-BWD switching
	int antijoystate;

	if (trigstate && ((1000 > joystate) || (joystate > -1000))) { //full forward
		antijoystate = joystate = trigstate;
	}
	else if (trigstate && ((1000 < joystate) || (joystate < -1000))) { //leaning
		if (joystate<0) {
			antijoystate = 0.75 * joystate;
		}
		else {
			antijoystate = trigstate;
			joystate = 0.75 * trigstate;
		}
	}
	else { //doubleRot
		antijoystate = - joystate;
	}
	//if ((pack.LFM > 0) && (antijoystate > 0))
	//{
		pack = {antijoystate, joystate};
//	}
//	else {
//		pack = {0, 0};
//	}
	return pack;
}

double cubic (double n) {
	return (n*n*n);
}

int Rshifter(int ref, int x) { //negative x for other motor
	int n = ref;
	if ((ref > 0) && (x < 0)) {
		if ((x * -1) > ref) {
			n = (x * -1);
		}
		else{
			n = ref;
		}
	}
	else if ((ref < 0) && (x > 0)) {
		if ((x * -1) < ref) {
			n = (x * -1);
		}
		else{
			n = ref;
		}
		
	}
	return n;
}

int Lshifter(int ref, int x) { //negative x for other motor
	int n = ref;
	if ((ref > 0) && (x > 0)) {
		if (x > ref) {
			n = x;
		}
		else{
			n = ref;
		}
	}
	else if ((ref < 0) && (x < 0)) {
		if (x < ref) {
			n = x ;
		}
		else{
			n = ref;
		}
		
	}
	return n;
}

int converter(int spec) {
	return int(cubic(double(spec) / 32767.0) * 102.0) + 311;
}

int main(int argc, char const *argv[])
{
	mapped coords;
	dual resolver;

	wiringPiSetup();

	int statechk = pca9685Setup(PIN_BASE, 0x41, HERTZ);
	if (statechk < 0) {
		printf("Error on setup!");
	}

	pca9685PWMReset(statechk);

    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <device>\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/input/js0\n", argv[0]);
        exit(0);
    }

    Joystick* joy = new Joystick(argv[1]);
	printf("Initializing...");
    while (true)
    {
        usleep(1000);
        joy->Update();

		coords = silmu(joy);
		int ZAxis = ((coords.RT + 32767) / 2) - ((coords.LT + 32767) / 2);
		int RotX = coords.RJX;
		
		int YAxis = coords.LJY;
		int XAxis = coords.LJX;
		float SC  = float(coords.RJY / 32767.0f * 0.5f + 1.5f);
		
		int Motor1;
		int Motor2;
		int Motor3;
		int Motor4;
		
		// ZAxis + RotX
		if (ZAxis >= 0 && RotX >= 0)
		{
			Motor1 = ZAxis;
			Motor4 = ZAxis - RotX;
		}
		else if (ZAxis >= 0 && RotX < 0)
		{
			Motor1 = ZAxis + RotX;
			Motor4 = ZAxis;
		}
		else if (ZAxis < 0 && RotX >= 0)
		{
			Motor1 = ZAxis;
			Motor4 = ZAxis + RotX;
		}
		else if (ZAxis < 0 && RotX < 0)
		{
			Motor1 = ZAxis - RotX;
			Motor4 = ZAxis;
		}
		
		// YAxis + XAxis
		if (YAxis >= 0 && XAxis >= 0)
		{
			Motor2 = YAxis;
			Motor3 = YAxis - XAxis;
		}
		else if (YAxis >= 0 && XAxis < 0)
		{
			Motor2 = YAxis + XAxis;
			Motor3 = YAxis;
		}
		else if (YAxis < 0 && XAxis >= 0)
		{
			Motor2 = YAxis;
			Motor3 = YAxis + XAxis;
		}
		else if (YAxis < 0 && XAxis < 0)
		{
			Motor2 = YAxis - XAxis;
			Motor3 = YAxis;
		}
		
		Motor1 = converter(Motor1);
		Motor2 = converter(Motor2);
		Motor3 = converter(Motor3);
		Motor4 = converter(Motor4);
	   
		pwmWrite(PIN_BASE + 5, Motor1);
		pwmWrite(PIN_BASE + 6, Motor2);
		pwmWrite(PIN_BASE + 1, Motor3);
		pwmWrite(PIN_BASE,     Motor4);
		pwmWrite(PIN_BASE + 7,  calcTick(SC, HERTZ));
		
		cout<<"Left:        " << Motor1 <<endl;
		cout<<"Upper Left:  " << Motor2 <<endl;
		cout<<"Upper Right: " << Motor3 <<endl;
		cout<<"Right:       " << Motor4 <<endl;
		cout<<"Camera:      " << SC <<endl;
		//	system("clear");
    }

    return 0;
}
