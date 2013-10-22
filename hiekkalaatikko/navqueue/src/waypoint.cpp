// Navigation waypoint

#include "waypoint.h"

Waypoint::Waypoint(float x, float y, float angle)
{
    _x = x;
    _y = y;
    _angle = angle;
}

Waypoint::Waypoint(float x, float y)
{
    _x = x;
    _y = y;
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

