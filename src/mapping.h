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

#define WINDOW_SIZE         300
#define PPM                 70      // pixels per meter
#define MAX_X               6.7     // map max x
#define MAX_Y               6.4     // map max y
#define MAP_RES             0.05    // in meters
#define MAP_X               int(MAX_X/MAP_RES + 1)
#define MAP_Y               int(MAX_Y/MAP_RES + 1)

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
    Mat frame,global_map;
    ofstream mapping_log;
    static const int display_scale = 80;
    std::ifstream map;
    vector<int> av;

    //Mapped Variables
    World &world;
    vector<CartPoint> cart_av;
public:
    int global_gridmap[MAP_Y][MAP_X];
    int local_gridmap[MAP_Y][MAP_X];
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
    readMap();
}


Mapping::~Mapping()
{
    log("End of Log");
    mapping_log.close();
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
        for (int j = 1; j <= av_range; ++j)
        {
            dist_av += pow(j,mult)*scan.ranges[i+j] + pow(j,mult)*scan.ranges[i-j];
            polar2cart(scan.ranges[i+j],(i+j)*-ang_inc+2,x,y);
            av_x += pow(j,mult)*x;
            av_y += pow(j,mult)*y;
            cart_av.push_back(CartPoint(x,y));
            av.push_back(0);
            polar2cart(scan.ranges[i-j],(i-j)*-ang_inc+2,x,y);
            av_x += pow(j,mult)*x;
            av_y += pow(j,mult)*y;
            cart_av.push_back(CartPoint(x,y));
            av.push_back(0);
            den += 2*pow(j,mult);
            //        polar2cart(dist_av,i*-ang_inc,av_x,av_y);
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
        if (distance(av_x,av_y,x,y) > 20*corner_compare_tol)
        {
            if ((dist_av + corner_compare_tol < scan.ranges[i]) && scan.ranges[i] > min_range)
            {
                world.concave_corners.push_back(LRFpoint(scan.ranges[i],i));
                //log(to_string(i) + ":" + to_string(distance(x1,y1,x2,y2)));
            }
            else if ((dist_av - 5*corner_compare_tol > scan.ranges[i]) && scan.ranges[i] > min_range)
                world.convex_corners.push_back(LRFpoint(scan.ranges[i],i));
        }
    }
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

    // Cabinet Detection

    displayMap();
    //cout << 2 <<endl;
}


void Mapping::localise()
{
    cout << "In Localise" << endl;
    double tempx,tempy,temptheta;
    tempx = world.x;
    tempy = world.y;
    temptheta = world.theta;
    int min_cost = MAP_X*MAP_Y;
    int cur_cost;
    double min_dx,min_dy,min_dtheta;
    min_dx = 0;
    min_dy = 0;
    min_dtheta = 0;
//    auto start = chrono::steady_clock::now();
    for (double dx = -0.15; dx <= 0.15; dx+=MAP_RES)
    {
        world.x = tempx+dx;
        for (double dy = -0.15; dy < 0.15; dy+=MAP_RES)
        {
            world.y = tempy+dy;
            for (double dtheta = -15*ang_inc; dtheta <= 15*ang_inc; dtheta+=ang_inc)
            {
                world.theta = temptheta-dtheta;
                makeLocalGridmap(world.x,world.y,world.theta,5);
                cur_cost = 0;
                for (int i = 0; i < MAP_X; ++i)
                {
                    for (int j = 0; j < MAP_Y; ++j)
                    {
                        cur_cost += (local_gridmap[j][i]-super_gridmap[j][i])*(local_gridmap[j][i]-super_gridmap[j][i]);
                    }
                }
                if (cur_cost < min_cost)
                {
                    min_dx = dx;
                    min_dy = dy;
                    min_dtheta = dtheta;
                    min_cost = cur_cost;
                }
//                cout << "dx: " << dx << " dy: " << dy << " dtheta: " << dtheta << endl;
//                sleep(1);
            }
        }
    }
//    auto end = chrono::steady_clock::now();
//    cout << "Elapsed time in nanoseconds : "
//                    << chrono::duration_cast<chrono::nanoseconds>(end - start).count()
//                    << " ns" << endl;

//            cout << "Elapsed time in microseconds : "
//                    << chrono::duration_cast<chrono::microseconds>(end - start).count()
//                    << " µs" << endl;

//            cout << "Elapsed time in milliseconds : "
//                    << chrono::duration_cast<chrono::milliseconds>(end - start).count()
//                    << " ms" << endl;

//            cout << "Elapsed time in seconds : "
//                    << chrono::duration_cast<chrono::seconds>(end - start).count()
//                    << " sec" << endl;
    world.x = tempx + min_dx;
    world.y = tempy + min_dy;
    world.theta = temptheta + min_dtheta;
//    cout << "min_dx: " << min_dx << " min_dy: " << min_dy << " min_dtheta: " << min_dtheta << endl;
//    cout << "world.x: " << world.x << " world.y: " << world.y << " world.theta: " << world.theta << endl;
    drawGlobalMap();
}


void Mapping::simplifyClusters()
{
    vector<LRFpoint> cluster, temp;

    // Cluster Concave Corner Points
    if (!world.concave_corners.empty())
    {
        cluster.push_back(world.concave_corners[0]);
        //cout << "Starting with " << world.concave_corners[0].i << endl;
        for (int i = 1; i < world.concave_corners.size(); ++i)
        {
            if (abs(cluster.back().i - world.concave_corners[i].i) < 4)
            {
                cluster.push_back(world.concave_corners[i]);
                //cout << "Adding " << world.concave_corners[i].i << " to cluster" << endl;
            }
            else
            {
                //cout << "Cluster complete" << endl;
                temp.push_back(cluster[cluster.size()/2]);
                //cout << "CLuster center: " << world.clustered_concave_corners.back().i << endl;
                cluster.clear();
                //cout << "Starting with " << world.concave_corners[i].i << endl;
                cluster.push_back(world.concave_corners[i]);
            }
        }
        if (!cluster.empty())
        {
            //cout << "Cluster complete" << endl;
            temp.push_back(cluster[cluster.size()/2]);
            //cout << "CLuster center: " << world.clustered_concave_corners.back().i << endl;
            cluster.clear();
        }
    }
    world.concave_corners = temp;

    // Cluster Convex Corner Points
    temp.clear();
    if(!world.convex_corners.empty())
    {
        cluster.push_back(world.convex_corners[0]);
        //cout << "Starting with " << world.concave_corners[0].i << endl;
        for (int i = 1; i < world.convex_corners.size(); ++i)
        {
            if (abs(cluster.back().i - world.convex_corners[i].i) < 4)
            {
                cluster.push_back(world.convex_corners[i]);
                //cout << "Adding " << world.concave_corners[i].i << " to cluster" << endl;
            }
            else
            {
                //cout << "Cluster complete" << endl;
                temp.push_back(cluster[cluster.size()/2]);
                //cout << "CLuster center: " << world.clustered_concave_corners.back().i << endl;
                cluster.clear();
                //cout << "Starting with " << world.concave_corners[i].i << endl;
                cluster.push_back(world.convex_corners[i]);
            }
        }
        if (!cluster.empty())
        {
            //TODO add end corner condition

            //cout << "Cluster complete" << endl;
            temp.push_back(cluster[cluster.size()/2]);
            //cout << "CLuster center: " << world.clustered_concave_corners.back().i << endl;
            cluster.clear();
        }
    }
    world.convex_corners = temp;
}


void Mapping::displayMap()
{
    frame = Mat::zeros(WINDOW_SIZE,WINDOW_SIZE,CV_8UC3);
    double x_c = WINDOW_SIZE/2.0;
    double y_c = WINDOW_SIZE/2.0;
    double x,y;
    int n_points;
//    for (int i = padding+av_range; i < scan_span-padding-av_range; ++i)
//    {
//        polar2cart(scan.ranges[i]*display_scale,(i*-ang_inc)+2,x,y,x_c,y_c);
//        circle(frame,Point(x,y),1,Scalar(180,180,180),1,8);
//    }
    for (int i = 0; i < cart_av.size(); ++i)
    {
        if (av[i] == 1)
            circle(frame,Point(cart_av[i].x*display_scale+x_c,cart_av[i].y*display_scale+y_c),1,Scalar(255,0,100),1,8);
        else
            circle(frame,Point(cart_av[i].x*display_scale+x_c,cart_av[i].y*display_scale+y_c),1,Scalar(0,255,0),1,8);
    }
//    n_points = world.dist_smooth.size();
//    for (int i = 0; i < n_points; ++i)
//    {
//        polar2cart(world.dist_smooth[i].d*display_scale,(world.dist_smooth[i].i*-ang_inc)+2,x,y,x_c,y_c);
//        circle(frame,Point(x,y),1,Scalar(255,0,100),1,8);
//    }
    n_points = world.concave_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.concave_corners[i].d*display_scale,(world.concave_corners[i].i*-ang_inc)+max_angle,x,y,x_c,y_c);
        circle(frame,Point(x,y),5,Scalar(255,0,0),2,8);
    }
    n_points = world.convex_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.convex_corners[i].d*display_scale,(world.convex_corners[i].i*-ang_inc)+max_angle,x,y,x_c,y_c);
        circle(frame,Point(x,y),5,Scalar(0,0,255),2,8);
    }
    const double compare_length = 0.3; //Check for 0.5m length
    int range_right = atan(compare_length/scan.ranges[world.right.i])/ang_inc;
    int range_left = atan(compare_length/scan.ranges[world.left.i])/ang_inc;
    for (int i = 0; i < range_right; ++i)
    {
        polar2cart(scan.ranges[world.right.i]/cos(i*ang_inc)*display_scale,(world.right.i+i)*-ang_inc+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
    }
    for (int i = 0; i < range_left; ++i)
    {
        polar2cart(scan.ranges[world.left.i]/cos(i*ang_inc)*display_scale,(world.left.i-i)*-ang_inc+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
    }
    flip(frame,frame,0);
    imshow("Visualization",frame);
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
            max_x = max(max_x,p1.x);
            max_x = max(max_x,p2.x);
            max_y = max(max_y,p1.y);
            max_y = max(max_y,p2.y);
            min_x = min(min_x,p1.x);
            min_x = min(min_x,p2.x);
            min_y = min(min_y,p1.y);
            min_y = min(min_y,p2.y);
        }
        CartPoint front((sides[0].p1.x + sides[0].p2.x)/2.0,
                (sides[0].p1.y + sides[0].p2.y)/2.0);
        if (front.x == max_x)
            front.x += min_permit_dist;
        else if (front.x == min_x)
            front.x -= min_permit_dist;
        else if (front.y == max_y)
            front.y += min_permit_dist;
        else if (front.y == min_y)
            front.y -= min_permit_dist;

        world.cabinets.push_back(Cabinet(sides,front));
    }
}

void Mapping::drawGlobalMap()
{
    double x,y;
    global_map= Mat::zeros(MAX_Y*PPM,MAX_X*PPM,CV_8UC3);

//    fill_n(&global_gridmap[0][0],sizeof(global_gridmap)/sizeof(**global_gridmap),0);
    for (int i = 0; i < world.walls.size(); ++i)
    {
        Point p1(world.walls[i].p1.x*PPM,world.walls[i].p1.y*PPM);
        Point p2(world.walls[i].p2.x*PPM,world.walls[i].p2.y*PPM);
        line(global_map,p1,p2,Scalar(255,255,255),2,8);
//        global_gridmap[int(world.walls[i].p1.y/MAP_RES)][int(world.walls[i].p1.x/MAP_RES)] = 1;
//        global_gridmap[int(world.walls[i].p2.y/MAP_RES)][int(world.walls[i].p2.x/MAP_RES)] = 1;
    }
    for (int i = 0; i < world.cabinets.size(); ++i)
    {
        for (int j = 0; j < world.cabinets[i].sides.size(); ++j)
        {
            Point p1(world.cabinets[i].sides[j].p1.x*PPM,
                     world.cabinets[i].sides[j].p1.y*PPM);
            Point p2(world.cabinets[i].sides[j].p2.x*PPM,
                     world.cabinets[i].sides[j].p2.y*PPM);
            line(global_map,p1,p2,Scalar(255,255,255),2,8);
//            global_gridmap[int(world.cabinets[i].sides[j].p1.y/MAP_RES)][int(world.cabinets[i].sides[j].p1.x/MAP_RES)] = 1;
//            global_gridmap[int(world.cabinets[i].sides[j].p2.y/MAP_RES)][int(world.cabinets[i].sides[j].p2.x/MAP_RES)] = 1;
        }
        circle(global_map,Point(world.cabinets[i].front.x*PPM,world.cabinets[i].front.y*PPM),2,Scalar(0,0,255),1,8);
    }
    circle(global_map,Point(world.x*PPM,world.y*PPM),10,Scalar(0,255,0),2,8);

    for (int i = padding; i < scan_span-padding; ++i)
    {
        polar2cart(scan.ranges[i],i*-ang_inc+2+world.theta,x,y,world.x,world.y);
        circle(global_map,Point(x*PPM,y*PPM),1,Scalar(0,255,0),1,8);
    }

    Mat flip_im;
    flip(global_map,flip_im,0);
    imshow("Global Map",flip_im);
    waitKey(25);
    moveWindow("Global Map",0,WINDOW_SIZE+100);
}

void Mapping::makeLocalGridmap(double dx, double dy, double dtheta, int step)
{
//    cv::Mat temp = Mat(MAP_Y,MAP_X,CV_8UC3,local_gridmap);
    vector<CartPoint> points;
    double x,y;
    // Convert all points (padding:step:scan_span-padding) to Cartesian Coordinates
    for (int i = padding; i < scan_span-padding; i += step)
    {
        polar2cart(scan.ranges[i],i*-ang_inc+2+dtheta,x,y,dx,dy);
        points.push_back(CartPoint(x,y));
    }
    // Add points on 2D map
    fill_n(&local_gridmap[0][0],sizeof(local_gridmap)/sizeof(**local_gridmap),0);
    for (int i = 0; i < points.size(); ++i)
    {
        int x_ = points[i].x/MAP_RES;
        int y_ = points[i].y/MAP_RES;
        if (x_ > MAP_X-1)
        {
            x_ = MAP_X-1;
        }
        if (y_ > MAP_Y-1)
        {
            y_ = MAP_Y-1;
        }
        local_gridmap[y_][x_] = 1;
//        circle(temp,Point(x_,y_),1,Scalar(255,255,255),1,8);
    }
//    for (int i = 0; i < MAP_X; ++i)
//    {
//        for (int j = 0; j < MAP_Y; ++j)
//        {
//            if (super_gridmap[j][i] == 1)
//            {
//                circle(temp,Point(i,j),1,Scalar(0,255,0),1,8);
//            }
//        }
//    }
//    flip(temp,temp,0);
//    imshow("temp",temp);
//    waitKey(25);
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
/*
cv::Mat temp = Mat(MAP_X,MAP_Y,CV_8U);
fill_n(&grid_map[0][0],sizeof(grid_map)/sizeof(**grid_map),0);
for (int i = 0; i < world.cabinets.size(); ++i)
{
    for (int j = 0; j < world.cabinets[i].sides.size(); ++j)
    {
        double x,y,m;
        CartPoint p1 = world.cabinets[i].sides[j].p1;
        CartPoint p2 = world.cabinets[i].sides[j].p2;
        if (p1.y == p2.y)
        {
            m = max(-MAP_RES,min(MAP_RES,p2.x-p1.x));
            x = p1.x;
            y = p1.y;
            while(fabs(x - p2.x) > 0.01)
            {
                grid_map[int(y/MAP_RES)][int(x/MAP_RES)] = 1;
                circle(temp,Point(int(x/MAP_RES),int(y/MAP_RES)),1,Scalar(255,255,255),1,8);
                x += m;
            }
        }
        else
        {
            m = max(-MAP_RES,min(MAP_RES,p2.y-p1.y));
            x = p1.x;
            y = p1.y;
            while(fabs(y - p2.y) > 0.01)
            {
                grid_map[int(y/MAP_RES)][int(x/MAP_RES)] = 1;
                circle(temp,Point(int(x/MAP_RES),int(y/MAP_RES)),1,Scalar(255,255,255),1,8);
                y += m;
            }
        }
    }
}
for (int i = 0; i < world.walls.size(); ++i)
{
    double x,y,m;
    CartPoint p1 = world.walls[i].p1;
    CartPoint p2 = world.walls[i].p2;
    if (p1.y == p2.y)
    {
        m = max(-MAP_RES,min(MAP_RES,p2.x-p1.x));
        x = p1.x;
        y = p1.y;
        while(fabs(x - p2.x) > 0.01)
        {
            grid_map[int(y/MAP_RES)][int(x/MAP_RES)] = 1;
            circle(temp,Point(int(x/MAP_RES),int(y/MAP_RES)),1,Scalar(255,255,255),1,8);
            x += m;
        }
    }
    else
    {
        m = max(-MAP_RES,min(MAP_RES,p2.y-p1.y));
        x = p1.x;
        y = p1.y;
        while(fabs(y - p2.y) > 0.01)
        {
            grid_map[int(y/MAP_RES)][int(x/MAP_RES)] = 1;
            circle(temp,Point(int(x/MAP_RES),int(y/MAP_RES)),1,Scalar(255,255,255),1,8);
            y += m;
        }
    }
}
imshow("tempmap",temp);
for (int i = 0; i < MAP_Y; ++i)
{
    for (int j = 0; j < MAP_X; ++j)
    {
        cout << grid_map[j][i];
    }
    cout << endl;
}
waitKey(25);

*/
