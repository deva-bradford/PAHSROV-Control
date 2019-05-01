#include "updates.h"

// This bounds any input to within the range of a short
short bound_short(int num)
{
    if (num > SHRT_MAX)
        return SHRT_MAX;
    else if (num < SHRT_MIN)
        return SHRT_MIN;
    else
        return num;
}

// Maps a value on an input range to an output range. Looks similar to the Arduino code, yeah?
short map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Specifically maps to the range of a short, like above. Not *really* necessary but nice to have.
short short_map(long x)
{
    return (x - SHRT_MIN) * (2*SHRT_MAX - 2*SHRT_MIN) / (SHRT_MAX - SHRT_MIN) + 2*SHRT_MIN;
}
/*
mapped update_buttons(Joystick* joy)
{

	mapped coords;


    switch (update)
    {

    default:
		coords = {
			joy->getButtonState(BUTTON_A),
			joy->getButtonState(BUTTON_B),
			joy->getButtonState(BUTTON_X),
			joy->getButtonState(BUTTON_Y),
			joy->getButtonState(BUTTON_LEFT_BUMPER),
			joy->getButtonState(BUTTON_RIGHT_BUMPER),
			joy->getButtonState(BUTTON_BACK),
			joy->getButtonState(BUTTON_START),
			joy->getButtonState(BUTTON_XBOX),
			joy->getButtonState(BUTTON_LEFT_CLICK),
			joy->getButtonState(BUTTON_RIGHT_CLICK)
		};
        break;
    }
}

mapped update_axes(Joystick* joy)
{
    mapped coords;

    switch (update)
    {

    case AXIS_LEFT_STICK_HORIZONTAL:
    case AXIS_LEFT_STICK_VERTICAL:
    case AXIS_RIGHT_STICK_HORIZONTAL:
    case AXIS_RIGHT_STICK_VERTICAL:

    case AXIS_LEFT_TRIGGER:
    case AXIS_RIGHT_TRIGGER:

    default:
		coords = {
			joy->getAxisState(AXIS_LEFT_STICK_HORIZONTAL),
			joy->getAxisState(AXIS_LEFT_STICK_VERTICAL),
			joy->getAxisState(AXIS_RIGHT_STICK_HORIZONTAL),
			joy->getAxisState(AXIS_RIGHT_STICK_VERTICAL),
			joy->getAxisState(AXIS_LEFT_TRIGGER),
			joy->getAxisState(AXIS_RIGHT_TRIGGER)
		};
        break;
    }
	return coords;
}
*/

mapped silmu(Joystick* joy) {

	mapped coords;


	coords = {
	joy->getButtonState(BUTTON_A),
	joy->getButtonState(BUTTON_B),
	joy->getButtonState(BUTTON_X),
	joy->getButtonState(BUTTON_Y),
	joy->getButtonState(BUTTON_LEFT_BUMPER),
	joy->getButtonState(BUTTON_RIGHT_BUMPER),
	joy->getButtonState(BUTTON_BACK),
	joy->getButtonState(BUTTON_START),
	joy->getButtonState(BUTTON_XBOX),
	joy->getButtonState(BUTTON_LEFT_CLICK),
	joy->getButtonState(BUTTON_RIGHT_CLICK),
	joy->getAxisState(AXIS_LEFT_STICK_HORIZONTAL),
	joy->getAxisState(AXIS_LEFT_STICK_VERTICAL),
	joy->getAxisState(AXIS_RIGHT_STICK_HORIZONTAL),
	joy->getAxisState(AXIS_RIGHT_STICK_VERTICAL),
	joy->getAxisState(AXIS_LEFT_TRIGGER),
	joy->getAxisState(AXIS_RIGHT_TRIGGER)
	};
	return coords;
}