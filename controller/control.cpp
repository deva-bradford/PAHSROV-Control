#include "joy.h"
#include "updates.h"
#include "pca9685.h"
#include <iostream>
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

int file_12c, fd;
char *filename = (char*)"/dev/i2c-1";

int calcTick(float impulseMs, int hertz) {
	float cycleMs = 1000.0f / hertz;
	return (int)(MAX_PWM * impulseMs / cycleMs + 4.0f);
}

int shifter(int ref, int x) {
	int n;
	if (ref > 0)
		n = |x|;
	if (ref < 0)
		n = -|x|;
	return n;
}

int converter(int spec) {
	return ((spec / 32000) * 102) + 311;
}

int main(int argc, char const *argv[])
{
    mapped coords;

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
	printf("Left joystick test on pin 0 initializing...");
    while (true)
    {
        usleep(1000);
        joy->Update();

		coords = silmu(joy);
		pwmWrite(PIN_BASE + 1, (((int) coords.LJY / 32000) * 102) + 311); //Setup if function to filter 1.5ms
    }

    return 0;
}
