#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include"common_resources.h"
#include"main.h"

#include<cmath>
#include<math.h>

struct Localization{
double Dist_tollerance=0.05;
double corner_x1;  // From the Json file
double corner_x2;  //From the Json file
double corner_x3;  //From the Json file
double corner_x4;  //From the Json file
double corner_y1;  //From the Json file
double corner_y2;  //From the Json file
double corner_y3;  //From the Json file
double corner_y4;  //From the Json file
double corner_x1_measured; //From the laser data
double corner_x2_measured; //From the laser data
double corner_x3_measured; //From the laser data
double corner_x4_measured; //From the laser data
double corner_y1_measured; //From the laser data
double corner_y2_measured; //From the laser data
double corner_y3_measured; //From the laser data
double corner_y4_measured; //From the laser data
double x_robot; //The robot initial x position
double y_robot; //The robot initial y position
localization();
};

void localization()
{
    double angle1=World.convex_corners.0.i*scan.angle_increment;
    double angle2=World.exits.leftEdge.i*scan.angle_increment;
    double angle3=World.exits.rightEdge.i*scan.angle_increment;
    double angle4=World.convex_corners.size().i*scan.angle_increment;
    polar2cart(World.convex_corners.0.d,angle1,corner_x1_measured,corner_y1_measured);
    polar2cart(World.exits.leftEdge.d,angle2,corner_x2_measured,corner_y2_measured);
    polar2cart(World.exits.rightEdge.d,angle3,corner_x3_measured,corner_y3_measured);
    polar2cart(World.convex_corners.size().d,angle4,corner_x4_measured,corner_y4_measured);
    int j=0; 
    double distance_corner12_measured=distance(corner_x1_measured,corner_y1_measured,corner_x2_measured,corner_y2_measured);
    double distance_corner34_measured=distance(corner_x3_measured,corner_y3_measured,corner_x4_measured,corner_y4_measured);
    double distance_corner12=distance(corner_x1,corner_y1,corner_x2,corner_y2);
    double distance_corner34=distance(corner_x3,corner_y3,corner_x4,corner_y4);

    while (abs(distance_corner12-distance_corner12_measured)>Dist_tollerance) {
        j++;
        polar2cart(World.convex_corners.j.d,angle1,corner_x1_measured,corner_y1_measured);
        double distance_corner12_measured=distance(corner_x1_measured,corner_y1_measured,corner_x2_measured,corner_y2_measured);
        int R=j;
        if (j=>World.convex_corners.size().d){
            cout<<"Corner 1 cannot be found"<<endl;
            break;
        }
    }
    
    j=World.convex_corners.size().d;
    
    while (abs(distance_corner34-distance_corner34_measured)>Dist_tollerance) {
        j--;
        polar2cart(World.convex_corners.j.d,angle1,corner_x4_measured,corner_y4_measured);
        double distance_corner34_measured=distance(corner_x3_measured,corner_y3_measured,corner_x4_measured,corner_y4_measured);
        int L=j;
        if (j<=0){
            cout<<"Corner 4 cannot be found"<<endl;
            break;
        }
    }
    if(R<World.convex_corners.size().d && L>0){
    x_robot=corner_x1-sin(angle1)*corner.R.d;
    y_robot=corner_y1-cos(angle1)*corner.R.d;
    }
    else {
        cout << "PICO cannot localize" << endl;
    }
        
}

#endif // LOCALIZATION_H

// Als muur 1 muur 1 is, dan kun je de afstand tot de hoek gebruiken om te bepalen waar je bent in de ruimte. Als de muur gematched is, weet je dat je x en y afstand vanaf dit punt, dat gegeven is, je actuale x en y positie zijn.
