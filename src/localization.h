#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include"common_resources.h"
#include"main.h"

#include<cmath>
#include<math.h>

double tollarance=0.05;
double corner_x1=;  // From the Json file
double corner_x2=;  //From the Json file
double corner_x3=;  //From the Json file
double corner_x4=;  //From the Json file
double corner_y1=;  //From the Json file
double corner_y2=;  //From the Json file
double corner_y3=;  //From the Json file
double corner_y4=;  //From the Json file
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

double angle1=corner.0.i*angle_increment;
double angle2=corner.1.i*angle_increment;
double angle3=corner.2.i*angle_increment;
double angle4=corner.3.i*angle_increment;

polar2cart(corner.0.d,angle1,corner_x1_measured,corner_y1_measured);
polar2cart(corner.1.d,angle2,corner_x2_measured,corner_y2_measured);
polar2cart(corner.2.d,angle3,corner_x3_measured,corner_y3_measured);
polar2cart(corner.3.d,angle4,corner_x4_measured,corner_y1_measured);

double distance_corner12_measured=distance(corner_x1_measured,corner_y1_measured,corner_x2_measured,corner_y2_measured);
double distance_corner34_measured=distance(corner_x3_measured,corner_y3_measured,corner_x4_measured,corner_y4_measured);
double distance_corner12=distance(corner_x1,corner_y1,corner_x2,corner_y2);
double distance_corner34=distance(corner_x3,corner_y3,corner_x4,corner_y4);

void localization(){
if (distance_corner12-distance_corner12_measured <= tollarance && distance_corner34 - distance_corner34_measured <= tollarance)
{
    cout << "Exit is located" << endl;
    x_robot=corner_x1-sin(angle1)*corner.0.d;
    y_robot=corner_y1-cos(angle1)*corner.0.d;
}
else
{
    cout <<"Exit is not located" << endl;
}
}
#endif // LOCALIZATION_H

// Als muur 1 muur 1 is, dan kun je de afstand tot de hoek gebruiken om te bepalen waar je bent in de ruimte. Als de muur gematched is, weet je dat je x en y afstand vanaf dit punt, dat gegeven is, je actuale x en y positie zijn.
