#include <slam/mapping.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <iostream>

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    if(!initialized_)
    {
        previousPose_ = pose;
	initialized_ = true;
    }
    MovingLaserScan movingScan(scan, previousPose_, pose);
    for(auto i = movingScan.begin(); i < movingScan.end(); i++)
    {
        updateEndpoint(*i, map);
        updateRay(*i, map);
  
    }
    previousPose_ = pose;
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
}


void Mapping::updateEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    //////////////// TODO: Implement this function //////////////////
    coordinate c = coordinate_convert_.get_end_point_coordinate(ray, map);
    increaseCellOdds(c.x, c.y, map);
}


void Mapping::updateRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    //////////////// TODO: Implement this function //////////////////
    ray_coordinates rc = coordinate_convert_.get_ray_coordinates(ray, map);
    for(auto c : rc)
        decreaseCellOdds(c.x, c.y, map);
}


void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map)
{
    //////////////// TODO: Implement this function //////////////////
    int cur_odds = (int)map.logOdds(x, y);
    if(cur_odds - (int)kMissOdds_ < -127)
        map.setLogOdds(x, y, -127);
    else    
        map.setLogOdds(x, y, map.logOdds(x, y) - kMissOdds_);
}


void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map)
{
    //////////////// TODO: Implement this function //////////////////
    int cur_odds = (int)map.logOdds(x, y);
    if(cur_odds + (int)kHitOdds_ > 127)
        map.setLogOdds(x, y, 127);
    else
        map.setLogOdds(x, y, map.logOdds(x, y) + kHitOdds_);
}
