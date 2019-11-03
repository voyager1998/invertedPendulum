#ifndef SLAM_MAPPING_HPP
#define SLAM_MAPPING_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid_utils.hpp>
#include <common/point.hpp>
#include <cstdint>
#include <set>

class OccupancyGrid;
class lidar_t;

/**
* Mapping implements the occupancy grid mapping algorithm. 
*/
class Mapping
{
public:
    
    /**
    * Constructor for Mapping.
    * 
    * \param    maxLaserDistance    Maximum distance for the rays to be traced
    * \param    hitOdds             Increase in occupied odds for cells hit by a laser ray
    * \param    missOdds            Decrease in occupied odds for cells passed through by a laser ray
    */
    Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds);
    
    /**                                                                                     *
    * updateMap incorporates information from a new laser scan into an existing OccupancyGrid.
    * 
    * \param    scan            Laser scan to use for updating the occupancy grid
    * \param    pose            Pose of the robot at the time when the last ray was measured
    * \param    map             OccupancyGrid instance to be updated
    */
    void updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map);

private:
    
    const float  kMaxLaserDistance_;
    const int8_t kHitOdds_;
    const int8_t kMissOdds_;
    bool initialized_;
    pose_xyt_t previousPose_;
    coordinate_convert coordinate_convert_;

    // Update the cell that is on the end point of the ray.
    void updateEndpoint   (const adjusted_ray_t& ray, OccupancyGrid& map);
    // Update the cells on the ray.
    void updateRay        (const adjusted_ray_t& ray, OccupancyGrid& map);
    void decreaseCellOdds(int x, int y, OccupancyGrid& map);
    void increaseCellOdds(int x, int y, OccupancyGrid& map);
    
    //////////////////// TODO: Add any private members needed for your occupancy grid mapping algorithm ///////////////
};

#endif // SLAM_MAPPING_HPP
