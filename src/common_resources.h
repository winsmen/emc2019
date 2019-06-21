#ifndef COMMON_RESOURCES_H
#define COMMON_RESOURCES_H

#include <sstream>
#include <string.h>
#include <fstream>
#include <cmath>
#include <iostream>
#include <vector>
#include <emc/io.h>

#define RIGHT   0
#define FRONT   1
#define LEFT    2

#define WINDOW_SIZE         500
#define PPM                 75      // pixels per meter
#define MAX_X               6.7     // map max x
#define MAX_Y               6.4     // map max y
#define MAP_RES             0.05   // in meters
#define MAP_X               int(MAX_X/MAP_RES + 1)
#define MAP_Y               int(MAX_Y/MAP_RES + 1)

using namespace std;

struct LRFpoint;
struct CartPoint;
struct Line;
struct Cabinet;
struct World;
struct Exit;
struct Preformance;
template<typename T>
string to_string(const T &n);
void polar2cart(double r, double theta, double &x, double &y, double x_off, double y_off);
//double distance(LRFpoint p1, LRFpoint p2);
inline double distance(double x1, double y1, double x2, double y2);


enum sys_state
{
    STARTUP,
    FIRST_LOCALISATION,
    GET_NEXT_CABINET,
    GO_TO_DESTINATION,
    AT_CABINET,
    STOP
};


struct LRFpoint
{
    double d;   //Distance
//    double a;   //Angle
    int i;      //Index
    LRFpoint(double d, int i)
    {
//cout << 1 <<endl;
        LRFpoint::d = d;
        LRFpoint::i = i;
//        a = (i-center)*ang_inc;
    }
    LRFpoint()
    {
        d = -1;
//        a = -1;
        i = -1;
    }
    void assignPoint(double d, int i)
    {
        LRFpoint::d = d;
        LRFpoint::i = i;
//        a = (i-center)*ang_inc;
    }
};


struct CartPoint {
    double x;
    double y;
    CartPoint(double x_, double y_ ): x(x_), y(y_)
    {
    }
    bool operator == (const CartPoint p1) const
    {
        return (x == p1.x && y == p1.y);
    }
};


struct Line {
    CartPoint p1,p2;
    Line(CartPoint p1_, CartPoint p2_ ): p1(p1_), p2(p2_)
    {
    }
};

struct Cabinet {
    typedef vector<Line> Lines;
    Lines sides;
    CartPoint front;
    double dir;
    Cabinet(Lines sides_, CartPoint front_, double dir_) : sides(sides_), front(front_), dir(dir_)
    {}
};


struct World
{
    //Environment Variables
    emc::LaserData scan;
    LRFpoint center, right, left;
    LRFpoint farthest, nearest;
    bool front_clear,right_clear,left_clear;
    vector<Exit> exits;
    vector<LRFpoint> convex_corners;
    vector<LRFpoint> concave_corners;
    vector<LRFpoint> dist_smooth;
    vector<CartPoint> points;
    vector<Line> walls;
    vector<Cabinet> cabinets;
    int global_gridmap[MAP_Y][MAP_X];
    int local_gridmap[MAP_Y][MAP_X];
    vector<int> path_x;
    vector<int> path_y;


    //Robot Variables
    emc::OdometryData odom;
    double des_vx, des_vy, des_vtheta;
    double vx, vy, vtheta;
    double des_x,des_y,des_theta;
    double x,y,theta;
    double x_off,y_off,theta_off;

    World();
    ~World();
};


struct Exit
{
    LRFpoint leftEdge,rightEdge,center;
    Exit(LRFpoint l, LRFpoint r) : leftEdge(l), rightEdge(r)
    {
        center.assignPoint((l.d+r.d)/2,(l.i+r.i)/2);
    }
};


World::World()
{
//cout << 1 << endl;
    center.assignPoint(-1,-1);
    right.assignPoint(-1,-1);
    left.assignPoint(-1,-1);
//cout << 2 << endl;
    theta = -1;
    farthest.assignPoint(-1,-1);
    nearest.assignPoint(-1,-1);
//cout << 3<< endl;
    front_clear = false;
    right_clear = false;
    left_clear = false;

    des_vtheta = des_vx = des_vy = 0;
    vtheta = vx = vy = 0;
    x = y = -1;
    theta = 0;
    theta_off = x_off = y_off = 0;
}

World::~World()
{
    //World Destructor
}


struct Performance
{
    const double min_permit_dist;
    const double dist_compare_tol;
    const double corner_compare_tol;
    const double angle_compare_tol;
    const int padding;
    const int av_range;
    const int side_range;
    const double min_range;
    const int heartbeat;
    const double max_rot,max_trans,min_rot,min_trans;

    Performance(float mpd, float dct, float cct, float act, int p, int avr, int sr, float mr,
                int hb, double maxT, double maxR, double minT, double minR)
        : min_permit_dist(mpd), dist_compare_tol(dct), corner_compare_tol(cct),
          angle_compare_tol(act), padding(p), av_range(avr), side_range(sr),
          min_range(mr), heartbeat(hb), max_rot(maxR), max_trans(maxT), min_rot(minR), min_trans(minT)
    {}
};


// define to_string function
template<typename T>
string to_string(const T& n)
{
    ostringstream ss;
    ss << n;
    return ss.str();
}

void polar2cart(double r,double theta, double &x,double &y, double x_off = 0, double y_off = 0)
{
    x = x_off + r*sin(theta);
    y = y_off + r*cos(theta);
}

inline double distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

#endif // COMMON_RESOURCES_H

