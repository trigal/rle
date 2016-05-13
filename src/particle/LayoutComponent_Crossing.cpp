#include "LayoutComponent_Crossing.h"

bool LayoutComponent_Crossing::flag = true;

void LayoutComponent_Crossing::calculateComponentScore()
{
    Mat res;
    //res.create(1,1,CV_32FC1);
    //Mat prova1(10,10,CV_32FC1,Scalar(0.0));
    //Mat prova2(10,10,CV_32FC1,Scalar(0.8));
    computeOccupancyGrid();
    matchTemplate(sensorOG, occupancyMap2, res, TM_CCOEFF_NORMED);
    setComponentWeight(res.at<float>(0, 0));

    ROS_DEBUG_STREAM("CROSSING SCORE: " << component_weight);
}

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
    center_x = value;
}

void LayoutComponent_Crossing::setCenter_y(float value)
{
    center_y = value;
}

/*void LayoutComponent_Crossing::setNum_ways(int value)
{
    num_ways=value;
}*/

void LayoutComponent_Crossing::addRoad(float width, double rotation)
{
    road new_road;
    new_road.width = width;
    new_road.rotation = M_PI - rotation;
    intersection_roads.push_back(new_road);
    num_ways++;
}

void LayoutComponent_Crossing::setCrossingState(ira_open_street_map::get_closest_crossing crossing)
{
    for (int i = 0; i < crossing.response.n_ways; i++)
    {
        addRoad(crossing.response.road_list.at(i).width, crossing.response.road_list.at(i).rotation);
    }

}


void LayoutComponent_Crossing::calculateDistanceCenter(double x, double y)
{
    this->center_y = sqrt((x - global_x) * (x - global_x) + (y - global_y) * (y - global_y));
}

void LayoutComponent_Crossing::computeOccupancyGrid()
{

    int n_col = max_x / gridCellSize;
    int n_row = (max_y - min_y) / gridCellSize;
    //occupancyMap = Mat::zeros(n_col, n_row, CV_8UC1);
    occupancyMap2 = Mat::zeros(n_col, n_row, CV_32FC3);

    Point polygonCV[num_ways][4];

    for (int i = 0; i < num_ways; i++)
    {

        float width = intersection_roads.at(i).width;
        Point p1(center_x - (width / 2), - max_x);
        Point p2(center_x - (width / 2), max_x - center_y + width / 2);
        Point p3(center_x + (width / 2), max_x - center_y + width / 2);
        Point p4(center_x + (width / 2), - max_x);


        if (intersection_roads.at(i).rotation != 0)
        {
            p1.x = center_x - (width / 2);
            p4.x = center_x + (width / 2);
            p1 = rotatePoint(p1, intersection_roads.at(i).rotation);
            p2 = rotatePoint(p2, intersection_roads.at(i).rotation);
            p3 = rotatePoint(p3, intersection_roads.at(i).rotation);
            p4 = rotatePoint(p4, intersection_roads.at(i).rotation);
        }


        p1.x = p1.x / gridCellSize;
        p1.y = p1.y / gridCellSize;
        p2.x = p2.x / gridCellSize;
        p2.y = p2.y / gridCellSize;
        p3.x = p3.x / gridCellSize;
        p3.y = p3.y / gridCellSize;
        p4.x = p4.x / gridCellSize;
        p4.y = p4.y / gridCellSize;


        polygonCV[i][0] = p1;
        polygonCV[i][1] = p2;
        polygonCV[i][2] = p3;
        polygonCV[i][3] = p4;

    }

    int npt[] = {4};
    for (int i = 0; i < num_ways; i++)
    {
        const Point* ppt[1] = {polygonCV[i]};

        //fillPoly(occupancyMap, ppt, npt, 1, Scalar(255), 8);
        fillPoly(occupancyMap2, ppt, npt, 1, Scalar(1., 1., 1.), 8);
    }

}

Point LayoutComponent_Crossing::rotatePoint(Point p, double angle)
{
    float s = sin(angle);
    float c = cos(angle);

    p.x = p.x - center_x;
    p.y = p.y - (max_x - center_y);

    Point newPoint(p.x * c - p.y * s, p.x * s + p.y * c);

    newPoint.x = newPoint.x + center_x;
    newPoint.y = newPoint.y + (max_x - center_y);

    return newPoint;

}
