#ifndef MAPPING_H
#define MAPPING_H

#include <emc/io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <ctime>
#include <fstream>
#include "json.hpp"
#include "common_resources.h"
#include <chrono>

#include <iostream>
using namespace std;

#ifndef MAPPING_LOG_FLAG
#define MAPPING_LOG_FLAG    1
#endif

#include "bin_map.h"

using namespace cv;
using namespace std;

class Mapping
{
    //Operational Variables
    emc::LaserData &scan;
    emc::OdometryData &odom;
    const int padding;
    const int av_range;
    const int side_range;
    int scan_span;
    double ang_inc;
    const double corner_compare_tol;
    const double min_range;
    const double min_permit_dist;
    double min_angle;
    double max_angle;
    Mat frame,global_map,map_template;
    ofstream mapping_log;
    static const int display_scale = 80;
    std::ifstream map;
    vector<int> av;
    int map_diff[MAP_Y][MAP_X];

    //Mapped Variables
    World &world;
    vector<CartPoint> cart_av;

    double findClosest(int x, int y);
public:
    Mapping(emc::LaserData *scan, emc::OdometryData *odom, World *world, const Performance specs);
    ~Mapping();
    void identify();
    void localise();
    void simplifyClusters();
    void displayMap();
    void readMap();
    void drawGlobalMap();
    void log(string text);
    void makeLocalGridmap(double dx, double dy, double dtheta, int step);
    void captureImage(int num);
    void genWeightedMap();
};


Mapping::Mapping(emc::LaserData *scan, emc::OdometryData *odom, World *world, const Performance specs)
    : scan(*scan), odom(*odom), world(*world), map("../src/finalmap.json"), min_range(specs.min_range),
      padding(specs.padding), av_range(specs.av_range), corner_compare_tol(specs.corner_compare_tol),
      side_range(specs.side_range), min_permit_dist(specs.min_permit_dist)
{
    ang_inc = scan->angle_increment;
    scan_span = scan->ranges.size();
    min_angle = scan->angle_min;
    max_angle = scan->angle_max;
    mapping_log.open("../logs/mapping_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Mapping Log: " + string(ctime(&now)));
    log("max_angle: " + to_string(max_angle) + " min angle: " + to_string(min_angle));
    fill_n(&map_diff[0][0],sizeof(map_diff)/sizeof(**map_diff),0);
    readMap();
}


Mapping::~Mapping()
{
    log("End of Log");
    mapping_log.close();
    destroyWindow("Visualization");
}

void Mapping::identify()
{
    //cout << 1;
    int mult = 0;

    // Corner Detection
    world.concave_corners.clear();
    world.convex_corners.clear();
    world.exits.clear();
    world.dist_smooth.clear();
    av.clear();
    cart_av.clear();
    for (int i = padding+av_range; i < scan_span-padding-av_range; ++i)
    {
        double dist_av = pow((av_range+1),mult)*scan.ranges[i];
        double av_x = 0, av_y = 0;
        double x,y;
        int den = pow(av_range+1,mult);

        // Compute Averaged distance data
        for (int j = 1; j <= av_range; ++j)
        {
            dist_av += pow(j,mult)*scan.ranges[i+j] + pow(j,mult)*scan.ranges[i-j];
            polar2cart(scan.ranges[i+j],(i+j)*-ang_inc+max_angle,x,y);
            av_x += pow(j,mult)*x;
            av_y += pow(j,mult)*y;
            cart_av.push_back(CartPoint(x,y));
            av.push_back(0);
            polar2cart(scan.ranges[i-j],(i-j)*-ang_inc+max_angle,x,y);
            av_x += pow(j,mult)*x;
            av_y += pow(j,mult)*y;
            cart_av.push_back(CartPoint(x,y));
            av.push_back(0);
            den += 2*pow(j,mult);
        }
        polar2cart(scan.ranges[i],i*-ang_inc,x,y);
        av_x += pow(av_range+1,mult)*x;
        av_y += pow(av_range+1,mult)*y;
        av_x /= den;
        av_y /= den;
        dist_av /= den;
        cart_av.push_back(CartPoint(av_x,av_y));
        av.push_back(1);
        world.dist_smooth.push_back(LRFpoint(dist_av,i));

        // Pick out corners
        if (distance(av_x,av_y,x,y) > 20*corner_compare_tol)
        {
            if ((dist_av + corner_compare_tol < scan.ranges[i]) && scan.ranges[i] > min_range)
            {
                world.concave_corners.push_back(LRFpoint(scan.ranges[i],i));
            }
            else if ((dist_av - 5*corner_compare_tol > scan.ranges[i]) && scan.ranges[i] > min_range)
            {
                world.convex_corners.push_back(LRFpoint(scan.ranges[i],i));
            }
        }
    }

    // Reduced clusters of (corner) points to single points
    simplifyClusters();
    log("Number of concave corners: " + to_string(world.concave_corners.size()));
    log("Number of convex corners: " + to_string(world.convex_corners.size()));

    // Exit Detection
    world.exits.clear();
    if (world.convex_corners.size() > 0)
    {
        for (int i = 0; i < world.convex_corners.size()-1; ++i)
        {
            double x1, x2, y1, y2;
            polar2cart(world.convex_corners[i].d,world.convex_corners[i].i*-ang_inc,x1,y1);
            polar2cart(world.convex_corners[i+1].d,world.convex_corners[i+1].i*-ang_inc,x2,y2);
            double midpoint_dist = (world.convex_corners[i].d + world.convex_corners[i+1].d)/2;
            int midpoint_ind = (world.convex_corners[i].i + world.convex_corners[i+1].i)/2;

            // Exits will have at least 0.4m and at most 1.3 separation between corners
            if (distance(x1,y1,x2,y2) > 0.4 && distance(x1,y1,x2,y2) < 1.3 && scan.ranges[midpoint_ind] - midpoint_dist > 0.5)
            {
                world.exits.push_back(Exit(world.convex_corners[i+1],world.convex_corners[i]));
                ++i;
            }
            log("Distance between concave corners " + to_string(i) + ": " + to_string(distance(x1,y1,x2,y2)));
            log("r1: " + to_string(world.convex_corners[i].d));
            log("i1: " + to_string(world.convex_corners[i].i));
            log("r2: " + to_string(world.convex_corners[i+1].d));
            log("i2: " + to_string(world.convex_corners[i+1].i));
            log("midpoint_dist: " + to_string(midpoint_dist));
            log("scan.ranges[midpoint_ind]:" + to_string(scan.ranges[midpoint_ind]));
        }
        log("Number of Exits: " + to_string(world.exits.size()));
    }

    displayMap();
    //cout << 2 <<endl;
}


void Mapping::localise()
{
    log("Odometry values - x: " + to_string(odom.x) + " y: " + to_string(odom.y) + " theta: "+ to_string(odom.a));
    double tempx,tempy,temptheta;
    double min_cost = MAP_X*MAP_Y*10;
    double cur_cost;
    double min_x,min_y,min_theta;
    min_x = world.x;
    min_y = world.y;
    min_theta = world.theta;
//    fill_n(&world.global_gridmap[0][0],sizeof(world.global_gridmap)/sizeof(**world.global_gridmap),0);
    for (double dx = -0.1; dx <= 0.1; dx+=MAP_RES)
    {
        tempx = world.x + dx;
        for (double dy = -0.1; dy <= 0.1; dy+=MAP_RES)
        {
            tempy = world.y + dy;
            for (double dtheta = -10*ang_inc; dtheta <= 10*ang_inc; dtheta+=ang_inc)
            {
                temptheta = world.theta + dtheta;
                makeLocalGridmap(tempx,tempy,temptheta,20);
                cur_cost = 0;
                for (int i = 0; i < MAP_X; ++i)
                {
                    for (int j = 0; j < MAP_Y; ++j)
                    {
                        if (world.local_gridmap[j][i] == 1)
                        {
                            cur_cost += findClosest(i,j);
                        }
                    }
                }
                if (cur_cost < min_cost)
                {
                    min_x = tempx;
                    min_y = tempy;
                    min_theta = temptheta;
                    min_cost = cur_cost;
                }
            }
        }
    }
    world.x_off = min_x - odom.x;
    world.y_off = min_y - odom.y;
    world.theta_off = min_theta - odom.a;
    world.x = min_x;
    world.y = min_y;
    world.theta = min_theta;
    if (world.theta < -M_PI)
        world.theta += 2*M_PI;
    if (world.theta > M_PI)
        world.theta -= 2*M_PI;
    log("Best fit - x: " + to_string(world.x) + " y: " + to_string(world.y) + " theta: " + to_string(world.theta));
    log("Offset - x: " + to_string(world.x_off) + " y: " + to_string(world.y_off) + " theta: " + to_string(world.theta_off));
    makeLocalGridmap(world.x,world.y,world.theta,15);
    drawGlobalMap();
    genWeightedMap();
}


double Mapping::findClosest(int x, int y)
{
    // Obtain the closest obstacle point from x,y and return the distance
    float scaling = 2;
    for (int i = 0; i <=10; ++i)
    {
        for (int j = -i; j <= i; ++j)
        {
            if (x+i < MAP_X && y+j > -1 && y+j < MAP_Y)
                if (super_gridmap[y+j][x+i] == 1)
                {
                    return pow(distance(x,y,x+i,y+j),scaling);
                }
            if (x-i > -1 && y+j > -1 && y+j < MAP_Y)
                if (super_gridmap[y+j][x-i] == 1)
                {
                    return pow(distance(x,y,x+i,y+j),scaling);
                }
            if (y+i < MAP_Y && x+j > -1 && x+j < MAP_X)
                if (super_gridmap[y+i][x+j] == 1)
                {
                    return pow(distance(x,y,x+i,y+j),scaling);
                }
            if (y-i > -1 && x+j > -1 && x+j < MAP_X)
                if (super_gridmap[y-i][x+j] == 1)
                {
                    return pow(distance(x,y,x+i,y+j),scaling);
                }
        }
    }
    return pow(10,scaling);
}


void Mapping::simplifyClusters()
{
    vector<LRFpoint> cluster, temp;

    // Cluster Concave Corner Points
    if (!world.concave_corners.empty())
    {
        cluster.push_back(world.concave_corners[0]);
        for (int i = 1; i < world.concave_corners.size(); ++i)
        {
            // Add to cluster if index values are within 4 counts of each other, else register
            // midpoint of current cluster as a corner point and start with a new cluster
            if (abs(cluster.back().i - world.concave_corners[i].i) < 4)
            {
                cluster.push_back(world.concave_corners[i]);
            }
            else
            {
                temp.push_back(cluster[cluster.size()/2]);
                cluster.clear();
                cluster.push_back(world.concave_corners[i]);
            }
        }

        // Copy the last running cluster, if any
        if (!cluster.empty())
        {
            temp.push_back(cluster[cluster.size()/2]);
            cluster.clear();
        }
    }
    // Overwrite concave_corners with new simplified corner list
    world.concave_corners = temp;

    temp.clear();
    if(!world.convex_corners.empty())
    {
        cluster.push_back(world.convex_corners[0]);
        for (int i = 1; i < world.convex_corners.size(); ++i)
        {
            // Add to cluster if index values are within 4 counts of each other, else register
            // midpoint of current cluster as a corner point and start with a new cluster
            if (abs(cluster.back().i - world.convex_corners[i].i) < 4)
            {
                cluster.push_back(world.convex_corners[i]);
            }
            else
            {
                temp.push_back(cluster[cluster.size()/2]);
                cluster.clear();
                cluster.push_back(world.convex_corners[i]);
            }
        }

        // Copy the last running cluster, if any
        if (!cluster.empty())
        {
            temp.push_back(cluster[cluster.size()/2]);
            cluster.clear();
        }
    }
    // Overwrite convex_corners with new simplified corner list
    world.convex_corners = temp;
}


void Mapping::displayMap()
{
    frame = Mat::zeros(WINDOW_SIZE,WINDOW_SIZE,CV_8UC3);
    double x_c = WINDOW_SIZE/2.0;
    double y_c = WINDOW_SIZE/2.0;
    double x,y;
    int n_points;

    // Averaged LRF values
    for (int i = 0; i < cart_av.size(); ++i)
    {
        if (av[i] == 1)
            circle(frame,Point(cart_av[i].x*display_scale+x_c,cart_av[i].y*display_scale+y_c),1,Scalar(255,0,100),1,8);
        else
            circle(frame,Point(cart_av[i].x*display_scale+x_c,cart_av[i].y*display_scale+y_c),1,Scalar(0,255,0),1,8);
    }

    // Simplified Concave Corners
    n_points = world.concave_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.concave_corners[i].d*display_scale,(world.concave_corners[i].i*-ang_inc)+max_angle,x,y,x_c,y_c);
        circle(frame,Point(x,y),5,Scalar(255,0,0),2,8);
    }

    // Simplified Convex Corners
    n_points = world.convex_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.convex_corners[i].d*display_scale,(world.convex_corners[i].i*-ang_inc)+max_angle,x,y,x_c,y_c);
        circle(frame,Point(x,y),5,Scalar(0,0,255),2,8);
    }

    // Left/Right Alignment check line
    const double compare_length = 0.3; //Check for 0.3m length
    int range_right = atan(compare_length/scan.ranges[world.right.i])/ang_inc;
    int range_left = atan(compare_length/scan.ranges[world.left.i])/ang_inc;
    for (int i = 0; i < range_right; ++i)
    {
        polar2cart(scan.ranges[world.right.i]/cos(i*ang_inc)*display_scale,(world.right.i+i)*-ang_inc+max_angle,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
    }
    for (int i = 0; i < range_left; ++i)
    {
        polar2cart(scan.ranges[world.left.i]/cos(i*ang_inc)*display_scale,(world.left.i-i)*-ang_inc+max_angle,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
    }

    // Identified Exits
    for (int i = 0; i < world.exits.size(); ++i)
    {
        polar2cart(world.exits[i].center.d*display_scale,(world.exits[i].center.i*-ang_inc)+max_angle,x,y,x_c,y_c);
        circle(frame,Point(x,y),5,Scalar(255,0,255),2,8);
    }
    flip(frame,frame,0);
    imshow("Visualization",frame);
    moveWindow("Visualization",0,600);
    waitKey(25);
}

void Mapping::readMap()
{
    nlohmann::json doc = nlohmann::json::parse(map);
    for (const auto& p : doc.at("points") )
    {
        assert(p.is_object());
        assert(p.find("x") != p.end());  // check for key x
        assert(p.find("y") != p.end());  // check for key y
        world.points.push_back(CartPoint(p["x"], p["y"]));
    }

    for (const auto& l : doc.at("walls") )
    {
        CartPoint p1(world.points[l[0]].x,world.points[l[0]].y);
        CartPoint p2(world.points[l[1]].x,world.points[l[1]].y);
        world.walls.push_back(Line(p1, p2));
    }

    for (const auto& cab : doc.at("cabinets") )
    {
        double max_x = 0;
        double max_y = 0;
        double min_x = 100;
        double min_y = 100;
        vector<Line> sides;
        for (const auto& l : cab){
            CartPoint p1(world.points[l[0]].x,world.points[l[0]].y);
            CartPoint p2(world.points[l[1]].x,world.points[l[1]].y);
            sides.push_back(Line(p1, p2));
            max_x = max(max_x,max(p1.x,p2.x));
            max_y = max(max_y,max(p1.y,p2.y));
            min_x = min(min_x,min(p1.x,p2.x));
            min_y = min(min_y,min(p1.y,p2.y));
        }
        CartPoint front((sides[0].p1.x + sides[0].p2.x)/2.0,
                (sides[0].p1.y + sides[0].p2.y)/2.0);
        double dir;
        if (front.x == max_x)
        {
            front.x += min_permit_dist;
            dir = M_PI/2;
        }
        else if (front.x == min_x)
        {
            front.x -= min_permit_dist;
            dir = -M_PI/2;
        }
        else if (front.y == max_y)
        {
            front.y += min_permit_dist;
            dir = M_PI;
        }
        else if (front.y == min_y)
        {
            front.y -= min_permit_dist;
            dir = 0;
        }

        world.cabinets.push_back(Cabinet(sides,front,dir));
    }

    // Initialize map_template
    map_template = Mat::zeros(MAX_Y*PPM,MAX_X*PPM,CV_8UC3);
    for (int i = 0; i < world.walls.size(); ++i)
    {
        Point p1(world.walls[i].p1.x*PPM,world.walls[i].p1.y*PPM);
        Point p2(world.walls[i].p2.x*PPM,world.walls[i].p2.y*PPM);
        line(map_template,p1,p2,Scalar(255,255,255),2,8);
    }
    for (int i = 0; i < world.cabinets.size(); ++i)
    {
        for (int j = 0; j < world.cabinets[i].sides.size(); ++j)
        {
            Point p1(world.cabinets[i].sides[j].p1.x*PPM,
                     world.cabinets[i].sides[j].p1.y*PPM);
            Point p2(world.cabinets[i].sides[j].p2.x*PPM,
                     world.cabinets[i].sides[j].p2.y*PPM);
            line(map_template,p1,p2,Scalar(255,255,255),2,8);
        }
        circle(map_template,Point(world.cabinets[i].front.x*PPM,world.cabinets[i].front.y*PPM),2,Scalar(0,0,255),1,8);
    }
}

void Mapping::drawGlobalMap()
{
    double x,y;
    map_template.copyTo(global_map);

    // Raw LRF values within padding range
    for (int i = padding; i < scan_span-padding; ++i)
    {
        polar2cart(scan.ranges[i],i*-ang_inc+max_angle-world.theta,x,y,world.x,world.y);
        circle(global_map,Point(x*PPM,y*PPM),1,Scalar(0,255,0),1,8);
    }

    // First point marker
    polar2cart(scan.ranges[padding],padding*-ang_inc+max_angle-world.theta,x,y,world.x,world.y);
    circle(global_map,Point(x*PPM,y*PPM),5,Scalar(0,255,0),1,8);

    if (!world.path_x.empty())
    {
        for (int i = 0; i < world.path_x.size(); ++i)
        {
            circle(global_map,Point(world.path_x[i]*MAP_RES*PPM,world.path_y[i]*MAP_RES*PPM),1,Scalar(0,255,255),2,8);
        }
    }

    // Robot Position
    circle(global_map,Point(world.x*PPM,world.y*PPM),10,Scalar(0,255,0),2,8);
    line(global_map,Point(world.x*PPM,world.y*PPM),Point((world.x+0.2*sin(-world.theta))*PPM,(world.y+0.2*cos(-world.theta))*PPM),Scalar(0,0,255),1,8);

    flip(global_map,global_map,0);
    imshow("Visualization",global_map);
    moveWindow("Visualization",0,600);
    waitKey(25);
}

void Mapping::makeLocalGridmap(double dx, double dy, double dtheta, int step)
{
    vector<CartPoint> points;
    double x,y;

    // Convert all points (padding:step:scan_span-padding) to Cartesian Coordinates
    for (int i = padding; i < scan_span-padding; i += step)
    {
        polar2cart(scan.ranges[i],i*-ang_inc+max_angle-dtheta,x,y,dx,dy);
        points.push_back(CartPoint(x,y));
    }

    // Add points on 2D map (local_gridmap)
    fill_n(&world.local_gridmap[0][0],sizeof(world.local_gridmap)/sizeof(**world.local_gridmap),0);
    for (int i = 0; i < points.size(); ++i)
    {
        int x_ = int(points[i].x/MAP_RES);
        int y_ = int(points[i].y/MAP_RES);
        if (x_ < MAP_X && y_ < MAP_Y)
        {
            world.local_gridmap[y_][x_] = 1;
        }
    }
}


void Mapping::captureImage(int num)
{
    // Save current LRF readings to file
//    Mat frame = Mat::zeros(WINDOW_SIZE,WINDOW_SIZE,CV_8UC3);
//    double x_c = WINDOW_SIZE/2.0;
//    double y_c = WINDOW_SIZE/2.0;
//    double x,y;
//    for (int i = 0; i < cart_av.size(); ++i)
//    {
//        if (av[i] == 1)
//            circle(frame,Point(cart_av[i].x*display_scale+x_c,cart_av[i].y*display_scale+y_c),1,Scalar(255,0,100),1,8);
//        else
//            circle(frame,Point(cart_av[i].x*display_scale+x_c,cart_av[i].y*display_scale+y_c),1,Scalar(0,255,0),1,8);
//    }
//    circle(frame,Point(x_c,y_c),4,Scalar(255,255,255),2,8);
//    flip(frame,frame,0);
    imwrite("../captures/cab"+to_string(num)+".png",global_map);
}


void Mapping::genWeightedMap()
{
    // Generate weighted gridmap for path planning
    int layers = 8;
    int power = 1;
    fill_n(&world.global_gridmap[0][0],sizeof(world.global_gridmap)/sizeof(**world.global_gridmap),0);
    for (int i = 0; i < MAP_X; ++i)
    {
        for (int j = 0; j < MAP_Y; ++j)
        {
            if (super_gridmap[j][i] == 1 || world.local_gridmap[j][i] == 1)
            {
                for (int k = 0; k <= layers; ++k)
                {
                    for (int l = -k; l <= k; ++l)
                    {
                        if (j+k < MAP_Y && i+l > -1 && i+l < MAP_X && world.global_gridmap[j+k][i+l] < layers-k)
                            world.global_gridmap[j+k][i+l] = pow(layers-k,power);
                        if (j-k > -1 && i+l > -1 && i+l < MAP_X && world.global_gridmap[j-k][i+l] < layers-k)
                            world.global_gridmap[j-k][i+l] = pow(layers-k,power);
                        if (i+k < MAP_X && j+l > -1 && j+l < MAP_Y && world.global_gridmap[j+l][i+k] < layers-k)
                            world.global_gridmap[j+l][i+k] = pow(layers-k,power);
                        if (i-k > -1 && j+l > -1 && j+l < MAP_Y && world.global_gridmap[j+l][i-k] < layers-k)
                            world.global_gridmap[j+l][i-k] = pow(layers-k,power);
                    }
                }
                map_diff[j][i] += world.local_gridmap[j][i] - super_gridmap[j][i];
                if (map_diff[j][i] > 20)
                    super_gridmap[j][i] = 1;
            }
        }
    }
    layers = 4;
    for (int n = 0; n < world.cabinets.size(); ++n)
    {
        int i = world.cabinets[n].front.x/MAP_RES;
        int j = world.cabinets[n].front.y/MAP_RES;
        for (int k = 0; k <= layers; ++k)
        {
            for (int l = -k; l <= k; ++l)
            {
                if (j+k < MAP_Y && i+l > -1 && i+l < MAP_X && world.global_gridmap[j+k][i+l] < layers-k)
                    world.global_gridmap[j+k][i+l] = 0;
                if (j-k > -1 && i+l > -1 && i+l < MAP_X && world.global_gridmap[j-k][i+l] < layers-k)
                    world.global_gridmap[j-k][i+l] = 0;
                if (i+k < MAP_X && j+l > -1 && j+l < MAP_Y && world.global_gridmap[j+l][i+k] < layers-k)
                    world.global_gridmap[j+l][i+k] = 0;
                if (i-k > -1 && j+l > -1 && j+l < MAP_Y && world.global_gridmap[j+l][i-k] < layers-k)
                    world.global_gridmap[j+l][i-k] = 0;
            }
        }
    }
}


void Mapping::log(string text)
{
#if MAPPING_LOG_FLAG == 1 || MAPPING_LOG_FLAG == 3
    mapping_log << text << endl;
#endif
#if MAPPING_LOG_FLAG == 2 || MAPPING_LOG_FLAG == 3
    cout << "mapper: " << text << endl;
#endif
}

#endif // MAPPING_H
