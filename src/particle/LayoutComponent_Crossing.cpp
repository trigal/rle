#include "LayoutComponent_Crossing.h"

float LayoutComponent_Crossing::getCenter_x() const
{
    return center_x;
}

float LayoutComponent_Crossing::getCenter_y() const
{
    return center_y;
}

int LayoutComponent_Crossing::getNum_ways() const
{
    return num_ways;
}

void LayoutComponent_Crossing::setCenter_x(float value)
{
    center_x=value;
}

void LayoutComponent_Crossing::setCenter_y(float value)
{
    center_y=value;
}

/*void LayoutComponent_Crossing::setNum_ways(int value)
{
    num_ways=value;
}*/

void LayoutComponent_Crossing::addRoad(float width, double rotation)
{
    road new_road;
    new_road.width = width;
    new_road.rotation = rotation;
    intersection_roads.push_back(new_road);
    num_ways++;
}
