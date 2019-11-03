#include <slam/occupancy_grid_utils.hpp>
#include <common/grid_utils.hpp>
#include <cmath>
#include <iostream>

coordinate_convert::coordinate_convert()
{
}

coordinate coordinate_convert::get_end_point_coordinate(const adjusted_ray_t& ray, OccupancyGrid& map_)
{
    Point<float> end_pt = get_end_pt(ray);
    return get_coordinate(end_pt, map_);
}

ray_coordinates coordinate_convert::get_ray_coordinates(const adjusted_ray_t& ray, OccupancyGrid& map_)
{
    ray_coordinates result;

    coordinate start_coor = get_coordinate(ray.origin, map_);
    coordinate end_coor = get_coordinate(get_end_pt(ray), map_);
          
    int x0 = start_coor.x;
    int y0 = start_coor.y;
    int x1 = end_coor.x;
    int y1 = end_coor.y;
    //std::cout << ray.origin.x << " " << ray.origin.y << "  " << get_end_pt(ray).x << " " << get_end_pt(ray).y << std::endl; 
    int dx = end_coor.x - start_coor.x;
    int dy = end_coor.y - start_coor.y;
    int step_x, step_y;

    if(dy < 0)
    {
        dy = -dy;
        step_y = -1;
    }
    else
    {
        step_y = 1;
    }
    if(dx < 0)
    {
        dx = -dx;
        step_x = -1;
    }
    else
    {
        step_x = 1;
    }
    dy <<= 1;
    dx <<= 1;
    result.push_back(coordinate(x0, y0));

    if(dx > dy)
    {
        int fraction = dy - (dx >> 1);
        while(x0 != x1)
        {
            x0 += step_x;
            if(fraction >= 0)
            {
                y0 += step_y;
                fraction -= dx;
            }
            fraction += dy;
            result.push_back(coordinate(x0, y0));
        }
    }
    else
    {
        int fraction = dx - (dy >> 1);
        while(y0 != y1)
        {
            y0 += step_y;
            if(fraction > 0)
            {
                x0 += step_x;
                fraction -= dy;
            }
            fraction += dx;
            result.push_back(coordinate(x0, y0));
        }
    }
    result.pop_back();
    return result;
}

coordinate coordinate_convert::get_end_point_coordinate(const adjusted_ray_t& ray, const OccupancyGrid& map_)
{
    Point<float> end_pt = get_end_pt(ray);
    return get_coordinate(end_pt, map_);
}

ray_coordinates coordinate_convert::get_ray_coordinates(const adjusted_ray_t& ray, const OccupancyGrid& map_)
{
    ray_coordinates result;

    coordinate start_coor = get_coordinate(ray.origin, map_);
    coordinate end_coor = get_coordinate(get_end_pt(ray), map_);
    
    int x0 = start_coor.x;
    int y0 = start_coor.y;
    int x1 = end_coor.x;
    int y1 = end_coor.y;

    int dx = end_coor.x - start_coor.x;
    int dy = end_coor.y - start_coor.y;
    int step_x, step_y;

    if(dy < 0)
    {
        dy = -dy;
        step_y = -1;
    }
    else
    {
        step_y = 1;
    }
    if(dx < 0)
    {
        dx = -dx;
        step_x = -1;
    }
    else
    {
        step_x = 1;
    }
    dy <<= 1;
    dx <<= 1;
    result.push_back(coordinate(x0, y0));

    if(dx > dy)
    {
        int fraction = dy - (dx >> 1);
        while(x0 != x1)
        {
            x0 += step_x;
            if(fraction >= 0)
            {
                y0 += step_y;
                fraction -= dx;
            }
            fraction += dy;
            result.push_back(coordinate(x0, y0));
        }
    }
    else
    {
        int fraction = dx - (dy >> 1);
        while(y0 != y1)
        {
            y0 += step_y;
            if(fraction > 0)
            {
                x0 += step_x;
                fraction -= dy;
            }
            fraction += dx;
            result.push_back(coordinate(x0, y0));
        }
    }
    result.pop_back();
    return result;
}

coordinate coordinate_convert::get_coordinate(const Point<float>& pt, const OccupancyGrid& map_) const
{
    coordinate result;
    result.x = (int)roundf((pt.x - map_.originInGlobalFrame().x) / map_.metersPerCell());
    result.y = (int)roundf((pt.y - map_.originInGlobalFrame().y) / map_.metersPerCell());
    return result;
}

Point<float> coordinate_convert::get_end_pt(const adjusted_ray_t& ray)
{
    Point<float> end_pt;
    end_pt.x = ray.origin.x + ray.range * cos(ray.theta);
    end_pt.y = ray.origin.y + ray.range * sin(ray.theta);
    return end_pt;
}

double dist(coordinate c1, coordinate c2, double cellsize){
    return sqrt(pow((c1.x - c2.x) * cellsize, 2) + pow((c1.y - c2.y) * cellsize, 2));
}

/*
int main(int argc, char** argv)
{
    OccupancyGrid map(10.0f, 10.0f, 0.05f);
    coordinate_convert cc;

    adjusted_ray_t ray;
    Point<float> pt(0.0f, 0.0f);
    ray.origin = pt;
    ray.range = 1.0;
    ray.theta = 4 *  M_PI / 3;
    // ray.theta = 0;


    ray_coordinates rc = cc.get_ray_coordinates(ray, map);
    for (auto c : rc)
        c.print();
}
*/
