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

int calcTick(float impulseMs, int hertz) {
	float cycleMs = 1000.0f / hertz;
	return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

dual xresolver(int trigstate, int joystate) { //add function to neutralize the motor before FWB-BWD switching
	int antijoystate;
	dual pack;
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
		antijoystate = (0 - joystate);
	}
	if ((pack.LFM > 0) && (antijoystate > 0))
	{
		pack = {antijoystate, joystate};
	}
	else {
		pack = {311, 311}
	}
	return pack;
}

double cubic (double n) {
	return (n*n*n);
}

int shifter(int ref, int x) { //negative x for other motor
	int n = 0;
	if (ref > 0)
		n = abs(x);
	if (ref < 0)
		n = (0 - abs(x));
	return n;
}

int converter(int spec) {
	return (cubic((spec / 32767)) * 102) + 311;
}

int main(int argc, char const *argv[])
{
	mapped coords;
	dual resolver;

	wiringPiSetup();

	int statechk = pca9685Setup(PIN_BASE, 0x40, HERTZ);
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
		pwmWrite(PIN_BASE, converter(shifter(coords.LJY, coords.LJX)));
		pwmWrite(PIN_BASE + 1 , converter(shifter(coords.LJY, (0 - coords.LJX))));
		resolver = xresolver((coords.LT - coords.RT), coords.RJX);
		pwmWrite(PIN_BASE + 2 , converter(resolver.LFM));
		pwmWrite(PIN_BASE + 3 , converter(resolver.RFM));
    }

    return 0;
}
