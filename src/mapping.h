#ifndef MAPPING_H
#define MAPPING_H

#include <emc/io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "common_resources.h"

class Mapping
{
    //Operational Variables
    emc::LaserData scan;
    emc::OdometryData odom;
    int &padding;
    int &av_range;
    int scan_span;
    double ang_inc;

    //Mapped Variables
    double *distance;
    World world;
public:
    Mapping(emc::LaserData &scan, emc::OdometryData &odom, World &world, const Performance &specs);
    ~Mapping();
    map();
    displayMap();
};


Mapping::Mapping(emc::LaserData &scan, emc::OdometryData &odom, World &world, const Performance &specs)
    : scan(scan), odom(odom), world(world),
      padding(specs.padding), av_range(specs.av_range)
{
    int max_iter = 20;
    while(!io.readLaserData(scan) || !io.readOdometryData(odom) && max_iter > 0)
        --max_iter;
    ang_inc = scan.angle_increment;
    scan_span = scan.ranges.size();
    distance = new double[scan_span-2*(av_range+scan_span)];
}

int Mapping::map()
{
    int mult = 2;
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
        distance[i-padding-av_range] = dist_av;
        if ((dist_av - scan.ranges[i] > corner_compare_tol /*|| fabs(scan.ranges[i]-scan.ranges[i-1]) > 0.01*/)
                && scan.ranges[i] > 0.1)
        {
            corner_dist[n_corners] = scan.ranges[i];
            corner_angle[n_corners] = i;
            ++n_corners;
        }
    }
    text =  "Number of corners: " + to_string(n_corners);
    log(text);
//    cout << "Number of corners: " << n_corners << endl;
    if (n_corners > 0)
    {
//        cout << "Number of corners: " << n_corners << endl;
//        cout << "Corner Start: " << corner_angle[0] << " Corner End: " << corner_angle[n_corners-1] << endl;
        corridor_center = (corner_angle[0]+corner_angle[n_corners-1])/2;
//        cout << corner_dist[0] << " " << corner_dist[n_corners-1] << endl;
        corridor_center_dist = (corner_dist[0]+corner_dist[n_corners-1])/2;
        if (fabs((scan.ranges[corner_angle[n_corners-1]] + scan.ranges[corner_angle[0]])/2 - scan.ranges[corridor_center]) < corner_compare_tol)
        {
            corridor_center = -1;
            found_corridor -= 1;
            if (found_corridor < 0)
                found_corridor = 0;
        }
        else
        {
            found_corridor += 1;
            if (found_corridor > 10)
                found_corridor = 10;
        }
//        else
//            cout << "Found Exit at: " << corridor_center << endl;
//        cout << "Corner start and end: " << corner_angle[0] << " " << corner_angle[n_corners-1] << endl;
    }
    else
    {
        corridor_center = -1;
        corridor_center = -1;
        found_corridor -= 1;
        if (found_corridor < 0)
            found_corridor = 0;
    }
    displayMap();
    return 0;
}


void Mapping::displayMap()
{
    int frame_dim = 600;
    frame = Mat::zeros(frame_dim,frame_dim,CV_8UC3);
    double x_c = frame_dim/2.0;
    double y_c = frame_dim/2.0;
    double x,y;
    for (int i = av_range+padding; i < scan_span-padding-av_range; ++i)
    {
        polar2cart(scan.ranges[i],(i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(255,0,0),1,8);
//        polar2cart(distance[i-padding-av_range],(i*-ang_inc)+2,x,y,x_c,y_c);
//        circle(frame,Point(x,y),1,Scalar(0,255,255),1,8);
    }
//    polar2cart(0,0,x,y,x_c,y_c);
//    circle(frame,Point(x,y),3,Scalar(255,0,0),1,8);

    switch(state)
    {
    case FOLLOW_CORRIDOR:
//        double right_av = 0;
//        double left_av = 0;
        for (int i = 0; i < 40; ++i)
        {
//            right_av += scan.ranges[right+i] - (scan.ranges[right]/cos(i*ang_inc));
//            left_av += scan.ranges[left-i] - (scan.ranges[left]/cos(i*ang_inc));
            polar2cart((scan.ranges[right]/cos(i*ang_inc))+dist_compare_tol,((right+i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[right]/cos(i*ang_inc))-dist_compare_tol,((right+i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[left]/cos(i*ang_inc))+dist_compare_tol,((left-i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[left]/cos(i*ang_inc))-dist_compare_tol,((left-i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
        }
//        right_av /= 20;
//        left_av /= 20;
//        cout << left_av << " " << right_av << endl;
//        if (right_av > dist_compare_tol && left_av < -dist_compare_tol)
//            vtheta = -maxRot;
//        else if (left_av > dist_compare_tol && right_av < -dist_compare_tol)
//            vtheta = maxRot;
//        else
//            vtheta = 0;
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

    imshow("Visualization",frame);
    waitKey(25);
}

#endif // MAPPING_H

