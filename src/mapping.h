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

#include <iostream>
using namespace std;

#ifndef MAPPING_LOG_FLAG
#define MAPPING_LOG_FLAG    1
#endif

using namespace cv;
using namespace std;

class Mapping
{
    //Operational Variables
    emc::LaserData &scan;
    emc::OdometryData &odom;
    const int padding;
    const int av_range;
    int scan_span;
    double ang_inc;
    const double corner_compare_tol;
    const double min_range;
    Mat frame;
    ofstream mapping_log;
    static const int display_scale = 80;
    std::ifstream map;
    vector<int> av;

    //Mapped Variables
    World &world;
    vector<double> cart_av_x;
    vector<double> cart_av_y;
public:
    Mapping(emc::LaserData *scan, emc::OdometryData *odom, World *world, const Performance specs);
    ~Mapping();
    int identify();
    void simplifyClusters();
    void displayMap();
    void readMap();
    void log(string text);
};


Mapping::Mapping(emc::LaserData *scan, emc::OdometryData *odom, World *world, const Performance specs)
    : scan(*scan), odom(*odom), world(*world), map("../finalmap.json"), min_range(specs.min_range),
      padding(specs.padding), av_range(specs.av_range), corner_compare_tol(specs.corner_compare_tol)
{
    ang_inc = scan->angle_increment;
    scan_span = scan->ranges.size();
    mapping_log.open("../mapping_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Mapping Log: " + string(ctime(&now)));
    readMap();
}


Mapping::~Mapping()
{
    log("End of Log");
    mapping_log.close();
}

int Mapping::identify()
{
    //cout << 1;
    int mult = 0;

    // Corner Detection
    world.concave_corners.clear();
    world.convex_corners.clear();
    world.exits.clear();
    world.dist_smooth.clear();
    av.clear();
    cart_av_x.clear();
    cart_av_y.clear();
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
            cart_av_x.push_back(x);
            cart_av_y.push_back(y);
            av.push_back(0);
            polar2cart(scan.ranges[i-j],(i-j)*-ang_inc+2,x,y);
            av_x += pow(j,mult)*x;
            av_y += pow(j,mult)*y;
            cart_av_x.push_back(x);
            cart_av_y.push_back(y);
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
        cart_av_x.push_back(av_x);
        cart_av_y.push_back(av_y);
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
                world.exits.push_back(Exit(world.concave_corners[i],world.concave_corners[i+1]));
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
    return 0;
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
    int frame_dim = 600;
    frame = Mat::zeros(frame_dim,frame_dim,CV_8UC3);
    double x_c = frame_dim/2.0;
    double y_c = frame_dim/2.0;
    double x,y;
//    for (int i = padding+av_range; i < scan_span-padding-av_range; ++i)
//    {
//        polar2cart(scan.ranges[i]*display_scale,(i*-ang_inc)+2,x,y,x_c,y_c);
//        circle(frame,Point(x,y),1,Scalar(180,180,180),1,8);
//    }
    for (int i = 0; i < cart_av_x.size(); ++i)
    {
        if (av[i] == 1)
            ;//circle(frame,Point(cart_av_x[i]*display_scale+x_c,cart_av_y[i]*display_scale+y_c),1,Scalar(255,0,0),1,8);
        else
            circle(frame,Point(cart_av_x[i]*display_scale+x_c,cart_av_y[i]*display_scale+y_c),1,Scalar(0,255,0),1,8);
    }
    int n_points = world.dist_smooth.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.dist_smooth[i].d*display_scale,(world.dist_smooth[i].i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(255,0,100),1,8);
    }
    n_points = world.concave_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.concave_corners[i].d*display_scale,(world.concave_corners[i].i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),5,Scalar(255,0,0),2,8);
    }
    n_points = world.convex_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.convex_corners[i].d*display_scale,(world.convex_corners[i].i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),5,Scalar(0,0,255),2,8);
    }

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
        world.walls.push_back(Line(l[0], l[1]));
    }

    for (const auto& cab : doc.at("cabinets") )
    {
        typedef vector<Line> Cabinet;
        Cabinet cabinet;
        for (const auto& l : cab){
            cabinet.push_back(Line(l[0], l[1]));
        }
        world.cabinets.push_back(cabinet);
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

