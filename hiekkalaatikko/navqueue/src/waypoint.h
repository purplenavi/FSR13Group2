// Navigation waypoint

#ifndef _waypoint_h
#define _waypoint_h

class Waypoint {
    
    // Pose
    float _x;
    float _y;
    float _angle;
    
    // Angle may be automatically defined
    bool  _unset_angle;
    
public:
    
    // Position may be set only once
    Waypoint(float x, float y, float angle);
    Waypoint(float x, float y);
    
    
};

#endif
