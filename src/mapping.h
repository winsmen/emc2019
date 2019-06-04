#ifndef MAPPING_H
#define MAPPING_H

#include <emc/io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "common_resources.h"

#include <iostream>
using namespace std;

using namespace cv;

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
    Mat frame;

    //Mapped Variables
    World &world;
public:
    Mapping(emc::LaserData *scan, emc::OdometryData *odom, World *world, const Performance specs);
    ~Mapping();
    int identify();
    void displayMap();
};


Mapping::Mapping(emc::LaserData *scan, emc::OdometryData *odom, World *world, const Performance specs)
    : scan(*scan), odom(*odom), world(*world),
      padding(specs.padding), av_range(specs.av_range), corner_compare_tol(specs.corner_compare_tol)
{
    ang_inc = scan->angle_increment;
    scan_span = scan->ranges.size();
}

int Mapping::identify()
{
    //cout << 1;
    int mult = 2;
    world.concave_corners.clear();
    world.convex_corners.clear();
    world.exits.clear();
    world.dist_smooth.clear();
    world.clustered_concave_corners.clear();
    for (int i = padding+av_range; i < scan_span-padding-av_range; ++i)
    {
        float dist_av = pow((av_range+1),mult)*scan.ranges[i];
        int den = pow((av_range+1),mult);
        for (int j = 1; j <= av_range; ++j)
        {
            dist_av += pow(j,mult)*scan.ranges[i+j] + pow(j,mult)*scan.ranges[i-j];
            den += 2*pow(j,mult);
        }
        dist_av /= den;
        world.dist_smooth.push_back(LRFpoint(dist_av,i));
        if ((dist_av - scan.ranges[i] > corner_compare_tol) && scan.ranges[i] > 0.1)
            world.convex_corners.push_back(LRFpoint(scan.ranges[i],i));
        else if ((dist_av - scan.ranges[i] < -corner_compare_tol) && scan.ranges[i] > 0.1)
            world.concave_corners.push_back(LRFpoint(scan.ranges[i],i));
    }
    if (world.concave_corners.size() < 1)
    {
        displayMap();
        return 0;
    }
    vector<LRFpoint> cluster;
    cluster.push_back(world.concave_corners[0]);
    //cout << "Starting with " << world.concave_corners[0].i << endl;
    for (int i = 1; i < world.concave_corners.size(); ++i)
    {
        if (abs(cluster.back().i - world.concave_corners[i].i) < 2)
        {
            cluster.push_back(world.concave_corners[i]);
            //cout << "Adding " << world.concave_corners[i].i << " to cluster" << endl;
        }
        else
        {
            //cout << "Cluster complete" << endl;
            world.clustered_concave_corners.push_back(cluster[cluster.size()/2]);
            //cout << "CLuster center: " << world.clustered_concave_corners.back().i << endl;
            cluster.clear();
            //cout << "Starting with " << world.concave_corners[i].i << endl;
            cluster.push_back(world.concave_corners[i]);
        }
    }
    if (!cluster.empty())
    {
        //cout << "Cluster complete" << endl;
        world.clustered_concave_corners.push_back(cluster[cluster.size()/2]);
        //cout << "CLuster center: " << world.clustered_concave_corners.back().i << endl;
        cluster.clear();
    }
    cout << "Number of concave corners: " << world.clustered_concave_corners.size() << endl;
    cout << "Number of convex corners: " << world.convex_corners.size() << endl;

    displayMap();
    //cout << 2 <<endl;
    return 0;
}


void Mapping::displayMap()
{
    int frame_dim = 600;
    frame = Mat::zeros(frame_dim,frame_dim,CV_8UC3);
    double x_c = frame_dim/2.0;
    double y_c = frame_dim/2.0;
    double x,y;
    for (int i = padding+av_range; i < scan_span-padding-av_range; ++i)
    {
        polar2cart(scan.ranges[i],(i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(180,180,180),1,8);
    }
    int n_points = world.concave_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.concave_corners[i].d,(world.concave_corners[i].i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(255,255,0),1,8);
    }
    n_points = world.convex_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.convex_corners[i].d,(world.convex_corners[i].i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(0,255,255),1,8);
    }
    n_points = world.clustered_concave_corners.size();
    for (int i = 0; i < n_points; ++i)
    {
        polar2cart(world.clustered_concave_corners[i].d,(world.clustered_concave_corners[i].i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(0,0,255),3,8);
    }
/*
    switch(state)
    {
    case FOLLOW_CORRIDOR:
        for (int i = 0; i < 40; ++i)
        {
            polar2cart((scan.ranges[right]/cos(i*ang_inc))+dist_compare_tol,((right+i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[right]/cos(i*ang_inc))-dist_compare_tol,((right+i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[left]/cos(i*ang_inc))+dist_compare_tol,((left-i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[left]/cos(i*ang_inc))-dist_compare_tol,((left-i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
        }
        break;
    default:
        for (int i = av_range+padding; i < scan_span-padding-av_range; ++i)
        {
            polar2cart(distance[i-padding-av_range]+corner_compare_tol,(i*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart(distance[i-padding-av_range]-corner_compare_tol,(i*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
        }
        if (n_corners > 1)
        {
            for (int i = 0; i < n_corners; ++i)
            {
                polar2cart(corner_dist[i],(corner_angle[i]*-ang_inc)+2,x,y,x_c,y_c);
                circle(frame,Point(x,y),1,Scalar(255,255,0),1,8);
            }
        }
        if (found_corridor == 10)
        {
            polar2cart(corridor_center_dist,(corridor_center*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),3,Scalar(0,255,0),2,8);
            polar2cart(corner_dist[0],(corner_angle[0]*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(0,0,255),1,8);
            polar2cart(corner_dist[n_corners-1],(corner_angle[n_corners-1]*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(0,0,255),1,8);
        }
    }
*/
    imshow("Visualization",frame);
    waitKey(25);
}

#endif // MAPPING_H

