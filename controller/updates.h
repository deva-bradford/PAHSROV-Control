#pragma once

#include "joy.h"
#include <iostream>
#include <climits>

struct mapped {
	int16_t BTA,BTB,BTX,BTY,BLB,BRB,BB,BS,BX,BLC,BRC,LJX,LJY,RJX,RJY,LT,RT;
};


mapped update_buttons(Joystick* joy);
mapped update_axes(Joystick* joy);
mapped silmu(Joystick* joy);

short bound_short(int num);
short map(long x, long in_min, long in_max, long out_min, long out_max);
short short_map(long x);
