// Navigation waypoint

#include "waypoint.h"

Waypoint::Waypoint(float x, float y, float angle)
{
    _x = x;
    _y = y;
    _angle = angle;
    _unset_angle = false;
}

Waypoint::Waypoint(float x, float y)
{
    _x = x;
    _y = y;
    _unset_angle = true;
}

float getX()
{
    return _x;
}

float getY()
{
    return _y;
}

float getAngle()
{
    return _angle;
}

