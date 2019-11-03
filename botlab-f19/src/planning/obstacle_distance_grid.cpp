#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <limits>
#include <iostream>
#include <planning/KDTree.hpp>
using namespace std;
#define USEKDTREE
ObstacleDistanceGrid::ObstacleDistanceGrid(void)
    : width_(0), height_(0), metersPerCell_(0.05f), cellsPerMeter_(20.0f)
{
}

Point<int> ObstacleDistanceGrid::poseToCoor(pose_xyt_t pos) const
{
    Point<int> coor;
    coor.x = (int)round((pos.x - globalOrigin_.x) / metersPerCell_);
    coor.y = (int)round((pos.y - globalOrigin_.y) / metersPerCell_);
    return coor;
}

pose_xyt_t ObstacleDistanceGrid::coorTopose(Point<int> current) const
{
    pose_xyt_t pos;
    // coor.x = (int)floor((pos.x - globalOrigin_.x) / metersPerCell_);
    // coor.y = (int)floor((pos.y - globalOrigin_.y) / metersPerCell_);
    pos.x = (float)(current.x * metersPerCell_ + globalOrigin_.x);
    pos.y = (float)(current.y * metersPerCell_ + globalOrigin_.y);
    return pos;
}



void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    std::cout << "Map, meters per cell: " << map.metersPerCell() << std::endl;
#ifndef USEKDTREE
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    for (int i = 0; i < width_; i++) {
        for (int j = 0; j < height_; j++) {
            cells_[cellIndex(i, j)] = 999;
        }
    }
    vector<pair<int, int>> occupied;
    for (int i = 0; i < width_; i++) {
        for (int j = 0; j < height_; j++) {
            if (map.logOdds(i, j) > 0) {
                bool ignore = false;
                if (map.logOdds(i - 1, j) > 0 && map.logOdds(i + 1, j) > 0 && map.logOdds(i, j - 1) > 0 && map.logOdds(i, j + 1) > 0) {
                    ignore = true;
                    cells_[cellIndex(i, j)] = 0;
                }
                if (!ignore) occupied.push_back(make_pair(i, j));
            }
        }   
    }
    // for (auto i : occupied) cout << i.first << ' ' << i.second << endl;
    for (int i = 0; i < width_; i++) {
        for (int j = 0; j < height_; j++) {
            float dis = 10000.f;
            for (size_t s = 0; s < occupied.size(); s++) {
                float d = sqrt((i - occupied[s].first) * (i - occupied[s].first) + (j - occupied[s].second) * (j - occupied[s].second));
                if (d < dis) dis = d;
            }
            // cout << dis << endl;
            if (cells_[cellIndex(i, j)] != 0)
                cells_[cellIndex(i, j)] = dis / 10;
        }    
    }
#else
    pointVec points;
    std::cout << "Begin to go through all cells to construct Distance Grid." << std::endl;
    for (int j = 0; j < height_; j++) {
        for (int i = 0; i < width_; i++) {
            cells_[cellIndex(i, j)] = 999;
            if (map.logOdds(i, j) > 0) {
                cells_[cellIndex(i, j)] = 0;
                bool ignore = false;
                if (map.logOdds(i - 1, j) > 0 && map.logOdds(i + 1, j) > 0 && map.logOdds(i, j - 1) > 0 && map.logOdds(i, j + 1) > 0) {
                    ignore = true;
                    // cells_[cellIndex(i, j)] = 0;
                }
                // if(map.logOdds(i, j) == 0)
                //     ignore = true;
                if (!ignore) {
                    point_t pt;
                    pt = {(double)i, (double)j};
                    points.push_back(pt);
                }
            }
        }
    }
    std::cout << "Num of Considered obstables: " << points.size() << std::endl;
    if (points.size() > 0) {
        std::cout << "start KDTree" << std::endl;
        KDTree tree(points);
        for (int j = 0; j < height_; j++) {
            for (int i = 0; i < width_; i++) {
                if (cells_[cellIndex(i, j)] != 0 /*&& map.logOdds(i, j) < 0*/) {
                    point_t pt;
                    pt = {(double)i, (double)j};
                    auto res = tree.nearest_point(pt);
                    // if (res.size() < 2) {
                    //     // std::cout << "!!!!!" << std::endl;
                    //     cells_[cellIndex(i, j)] = 0;
                    // }
                    // else {
                    cells_[cellIndex(i, j)] = (float)sqrt((i - (int)res[0]) * (i - (int)res[0]) + (j - (int)res[1]) * (j - (int)res[1])) * map.metersPerCell();  // / 10.0;
                    // }
                }
            }
        }
        std::cout << "end KDTree" << std::endl;
    }
#endif
}

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}

void ObstacleDistanceGrid::resetGrid(const OccupancyGrid &map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();

    // If the grid is already the correct size, nothing needs to be done
    if ((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }

    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();

    cells_.resize(width_ * height_);
}
