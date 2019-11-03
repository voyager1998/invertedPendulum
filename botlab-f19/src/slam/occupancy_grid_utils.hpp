#ifndef SLAM_OCCUPANCY_GRID_UTILS_HPP
#define SLAM_OCCUPANCY_GRID_UTILS_HPP

#include <slam/occupancy_grid.hpp>
#include <slam/moving_laser_scan.hpp>
#include <vector>
#include <iostream>

class coordinate
{
public:
    int x;
    int y;
    coordinate(int x_ = 0, int y_ = 0) : x(x_), y(y_)
    {}
    void print()
    {
        std::cout << "x: " << x << " y: " << y << std::endl;
    }
};

typedef std::vector<coordinate> ray_coordinates;

class coordinate_convert
{

public:
    coordinate_convert();

    coordinate get_end_point_coordinate(const adjusted_ray_t& ray, const OccupancyGrid& map);

    coordinate get_end_point_coordinate(const adjusted_ray_t& ray, OccupancyGrid& map);

    ray_coordinates get_ray_coordinates(const adjusted_ray_t& ray, const OccupancyGrid& map);

    ray_coordinates get_ray_coordinates(const adjusted_ray_t& ray, OccupancyGrid& map);

    
    
private:
    coordinate get_coordinate(const Point<float>& pt, const OccupancyGrid& map) const;
    
    Point<float> get_end_pt(const adjusted_ray_t& ray);

};

double dist(coordinate c1, coordinate c2, double cellsize);

#endif