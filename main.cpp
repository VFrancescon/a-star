#include <iostream>
#include <string.h>
#include "source/AStar.hpp"
#include <opencv2/core.hpp>

int main()
{
    
    int worldx = 579, worldy = 768;
    cv::Point CVworld(worldx, worldy);
    cv::Point tip(150,150);
    cv::Point goal(200,200);
    AStar::Vec2i world, src, dst;
    world = CVworld;
    src = tip;
    dst = goal;
    
    AStar::Generator generator;
    generator.setWorldSize(world);
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    std::vector<int> row;
    std::vector<std::vector<int>> grid;

    for(int j = 0; j < world.y; j++) {
     
        for(int i = 0; i < world.x; i++){
            row.push_back(0);
        }
        grid.push_back(row);
        row.clear();
    }
    
    for(int j = 1; j < 5; j++){

        for(int i = 2; i < 4; i++){
            generator.addCollision({j,i});
            grid[j][i] = 1;
        }
    }

    std::cout << "Generating path ... \n";
    std::vector<int> x_speed, y_speed;
    auto path = generator.findPath(src, dst);
    std::vector<cv::Point> cvPath = AStar::Vec2iToCvPointList(path);
    for(auto i: cvPath){
        std::cout << i << "\n";
    }
    
}
