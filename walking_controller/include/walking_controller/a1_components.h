#ifndef A1_COMPONENTS_H
#define A1_COMPONENTS_H

class Linear
{
    public:
        float x;
        float y;
        float z;
        Linear():
            x(0.0f),
            y(0.0f),
            z(0.0f)
        {}
};

class Angular
{
    public:
        float x;
        float y;
        float z;
        Angular():
            x(0.0f),
            y(0.0f),
            z(0.0f)
        {}
};

class Velocities
{
    public:
        Linear linear;
        Angular angular;
};

class Point
{
    public:
        float x;
        float y;
        float z;
        Point():
            x(0.0f),
            y(0.0f),
            z(0.0f)
        {}
};

class Euler
{
    public:
        Euler():
            roll(0.0f), 
            pitch(0.0f), 
            yaw(0.0f)
        {}
        float roll;
        float pitch;
        float yaw;
};

class Pose
{
    public:
        Point position;
        Euler orientation;
};

#endif