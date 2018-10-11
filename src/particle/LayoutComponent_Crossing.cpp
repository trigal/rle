#include "LayoutComponent_Crossing.h"
#include "Particle.h"

bool LayoutComponent_Crossing::flag = true;

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

void LayoutComponent_Crossing::calculateComponentScore()
{
    Mat res;
    computeOccupancyGrid();
    removeUknownCells();
    matchTemplate(sensorOG, occupancyMap2, res, TM_CCOEFF_NORMED);
    if (res.at<float>(0, 0) < 1)
        setComponentWeight(res.at<float>(0, 0) * alpha);
    else
        setComponentWeight(0.);

    ROS_DEBUG_STREAM("CROSSING SCORE: " << this->getComponentWeight());
}

void LayoutComponent_Crossing::componentPoseEstimation()
{
    cout << "Propagating and estimating CROSSING component pose. ID: " << component_id << " that belongs to particle ID: " << this->getParticlePtr()->getId() << endl;

    ira_open_street_map::get_closest_crossingXY c;
    tf::Stamped<tf::Pose>  tf_global = Utils::toGlobalFrame(particlePtr->getParticleState().getPosition());
    c.request.x = tf_global.getOrigin().getX();
    c.request.y = tf_global.getOrigin().getY();
    c.request.rotation = this->particlePtr->getParticleState().getYaw();
    get_closest_crossingXY_client.call(c);

    ROS_DEBUG_STREAM("CROSSING ID: " << c.response.id << "     ROTATION: " << this->particlePtr->getParticleState().getYaw());
    ROS_DEBUG_STREAM("rotation_diff: " << c.response.rotation_diff);

    setCrossingState(c);

}

/*void LayoutComponent_Crossing::setNum_ways(int value)
{
    num_ways=value;
}*/

void LayoutComponent_Crossing::addRoad(float width, double rotation)
{
    road new_road;
    new_road.width = width;
    new_road.rotation = M_PI - rotation; //NEVER CHANGE THIS LINE!!!! YOU WILL REGRET IT!!!!!
    if (new_road.rotation < 0)
        new_road.rotation += 2 * M_PI;
    intersection_roads.push_back(new_road);
    num_ways++;
}

void LayoutComponent_Crossing::setCrossingState(ira_open_street_map::get_closest_crossingXY crossing)
{

    this->global_x = crossing.response.x;
    this->global_y = crossing.response.y;
    intersection_roads.clear();
    num_ways = 0;

    if (crossing.response.found)
    {

        calculateDistanceCenter(crossing.request.x, crossing.request.y, crossing.response.rotation_diff);

        for (int i = 0; i < crossing.response.n_ways; i++)
        {
            addRoad(crossing.response.road_list.at(i).width, crossing.response.road_list.at(i).rotation);
        }
    }
    else
    {
        this->center_x = 15.;
        this->center_y = max_x;
    }

}


void LayoutComponent_Crossing::calculateDistanceCenter(double x, double y, double rotation_diff)
{

    double distance = sqrt((x - global_x) * (x - global_x) + (y - global_y) * (y - global_y));
    this->center_y = cos(rotation_diff) * distance;
    this->center_x = 15. - sin(rotation_diff) * distance;
    ROS_DEBUG_STREAM("Rotation Diff: " << rotation_diff << ", Distance: " << distance << ", Y: " << this->center_y << " , X: " << this->center_x);
}

void LayoutComponent_Crossing::computeOccupancyGrid()
{

    int n_col = max_x / gridCellSize;
    int n_row = (max_y - min_y) / gridCellSize;

    occupancyMap2 = Mat::zeros(n_col, n_row, CV_32FC3);

    ///Se l'incrocio è vicino (minore di 40m)
    if (this->center_y < 40.)
    {

        Point polygonCV[num_ways][4];

        ///Creo un rettangolo per ogni strada dell'incrocio
        for (int i = 0; i < num_ways; i++)
        {

            //Inizialmente lo creo come se fosse una strada dritta (Qua i punti sono espressi in metri!)
            float width = intersection_roads.at(i).width;
            Point2f p1(center_x - (width / 2), - max_x);
            Point2f p2(center_x - (width / 2), max_x - center_y + width / 2);
            Point2f p3(center_x + (width / 2), max_x - center_y + width / 2);
            Point2f p4(center_x + (width / 2), - max_x);

            //Poi lo ruoto utilizzando come perno il centro dell'incrocio
            if (intersection_roads.at(i).rotation != 0)
            {
                p1.x = center_x - (width / 2);
                p4.x = center_x + (width / 2);
                p1 = rotatePoint(p1, intersection_roads.at(i).rotation);
                p2 = rotatePoint(p2, intersection_roads.at(i).rotation);
                p3 = rotatePoint(p3, intersection_roads.at(i).rotation);
                p4 = rotatePoint(p4, intersection_roads.at(i).rotation);
            }

            //Trasformo i metri in celle!
            p1.x = p1.x / gridCellSize;
            p1.y = p1.y / gridCellSize;
            p2.x = p2.x / gridCellSize;
            p2.y = p2.y / gridCellSize;
            p3.x = p3.x / gridCellSize;
            p3.y = p3.y / gridCellSize;
            p4.x = p4.x / gridCellSize;
            p4.y = p4.y / gridCellSize;


            polygonCV[i][0] = Point(p1.x, p1.y);
            polygonCV[i][1] = Point(p2.x, p2.y);
            polygonCV[i][2] = Point(p3.x, p3.y);
            polygonCV[i][3] = Point(p4.x, p4.y);

        }

        int npt[] = {4};
        for (int i = 0; i < num_ways; i++)
        {
            const Point* ppt[1] = {polygonCV[i]};

            //Qua viene creata la griglia di occupazione! "riempiendo" i rettangoli creati prima.
            cv::fillPoly(occupancyMap2, ppt, npt, 1, Scalar(1., 1., 1.), 8);
        }
    }
    ///Se l'incrocio è lontano (oppure se non è stato trovato nessun incrocio)
    else
    {

        //Prima recupero la particella "snapped"
        ira_open_street_map::snap_particle_xy snapParticle_serviceMessage;
        tf::Stamped<tf::Pose>  tf_global = Utils::toGlobalFrame(particlePtr->getParticleState().getPosition());
        snapParticle_serviceMessage.request.x = tf_global.getOrigin().getX();
        snapParticle_serviceMessage.request.y = tf_global.getOrigin().getY();
        snapParticle_serviceMessage.request.max_distance_radius = 100;
        snap_particle_xy_client.call(snapParticle_serviceMessage);

        //Calcolo la differenza nelle componenti x e y (nel sistema di riferimento "map" - globale)
        //tra la particella "vera" e la particella "snapped"
        double snap_x = snapParticle_serviceMessage.response.snapped_x;
        double snap_y = snapParticle_serviceMessage.response.snapped_y;
        snap_x = snap_x - tf_global.getOrigin().getX();
        snap_y = snap_y - tf_global.getOrigin().getY();

        //Calcolo la distanza nelle componenti x e y NEL SISTEMA DI RIFERIMENTO PARTICELLA!
        double snap_x_rotated, snap_y_rotated;
        double angle = particlePtr->getParticleState().getYaw();
        snap_x_rotated = snap_x * cos(angle) + snap_y * sin(angle);
        snap_y_rotated = -snap_x * sin(angle) + snap_y * cos(angle);

        Point polygonCV[1][4];

        //Larghezza della strada più vicina
        float width = snapParticle_serviceMessage.response.snap_road_width;

        //Creo una strada dritta (parallela alla particella), traslata in base alla distanza
        //della strada più vicina
        this->center_x = 15. - snap_y_rotated;
        Point2f p1(center_x - (width / 2), -50.);
        Point2f p2(center_x - (width / 2), 100.);
        Point2f p3(center_x + (width / 2), 100.);
        Point2f p4(center_x + (width / 2), -50.);

        //Punto attorno a cui deve ruotare la strada, ovvero nella posizione della
        //particella "snappata" riportata nel SISTEMA DI RIFERIMENTO "griglia di occupazione"
        //(che è traslato di 15m a sinistra rispetto al sistema particella)
        Point2f center(15. - snap_y_rotated, snap_x_rotated);

        //Calcolo la rotazione della strada rispetto alla rotazione della particella
        double rotation = angle - snapParticle_serviceMessage.response.way_dir_rad;
        if (rotation > M_PI / 2 || rotation < -M_PI / 2)
            rotation -= M_PI;

        //Ruoto il rettangolo (strada)
        p1 = rotatePointCenter(p1, rotation, center);
        p2 = rotatePointCenter(p2, rotation, center);
        p3 = rotatePointCenter(p3, rotation, center);
        p4 = rotatePointCenter(p4, rotation, center);

        //Trasformo i metri in celle!
        p1.x = p1.x / gridCellSize;
        p1.y = p1.y / gridCellSize;
        p2.x = p2.x / gridCellSize;
        p2.y = p2.y / gridCellSize;
        p3.x = p3.x / gridCellSize;
        p3.y = p3.y / gridCellSize;
        p4.x = p4.x / gridCellSize;
        p4.y = p4.y / gridCellSize;

        polygonCV[0][0] = Point(p1.x, p1.y);
        polygonCV[0][1] = Point(p2.x, p2.y);
        polygonCV[0][2] = Point(p3.x, p3.y);
        polygonCV[0][3] = Point(p4.x, p4.y);

        int npt[] = {4};

        const Point* ppt[1] = {polygonCV[0]};

        //Creo la griglia riempiendo il rettangolo di 1
        fillPoly(occupancyMap2, ppt, npt, 1, Scalar(1., 1., 1.), 8);

    }

}

Point2f LayoutComponent_Crossing::rotatePoint(Point2f p, double angle)
{
    float s = sin(angle);
    float c = cos(angle);

    p.x = p.x - center_x;
    p.y = p.y - (max_x - center_y);

    Point2f newPoint(p.x * c - p.y * s, p.x * s + p.y * c);

    newPoint.x = newPoint.x + center_x;
    newPoint.y = newPoint.y + (max_x - center_y);

    return newPoint;

}

Point2f LayoutComponent_Crossing::rotatePointCenter(Point2f p, double angle, Point2f center)
{
    float s = sin(angle);
    float c = cos(angle);

    p.x = p.x - center.x;
    p.y = p.y - (max_x - center.y);

    Point2f newPoint(p.x * c - p.y * s, p.x * s + p.y * c);

    newPoint.x = newPoint.x + center.x;
    newPoint.y = newPoint.y + (max_x - center.y);

    return newPoint;

}

void LayoutComponent_Crossing::removeUknownCells()
{
    int n_forward_cells = max_x / gridCellSize;
    int n_lateral_cells = (max_y - min_y) / gridCellSize;
    for (int i = 0; i < n_forward_cells; i++)
    {
        for (int j = 0; j < n_lateral_cells; j++)
        {
            if (sensorOG.at<Vec3f>(n_forward_cells - i - 1, n_lateral_cells - j - 1) == Vec3f(1., 0., 0.))
            {
                occupancyMap2.at<Vec3f>(n_forward_cells - i - 1, n_lateral_cells - j - 1) = Vec3f(1., 0., 0.);
            }
        }
    }
}
