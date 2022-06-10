#include <iostream>
#include <string.h>
#include "source/AStar.hpp"
#include <opencv2/core.hpp>
#include <math.h>
#define PI 3.14159265

// std::vector<std::string> decodePath(std::vector<cv::Point> path_, int pointIncr=2);

namespace DXKDP_LIB{
    enum class Direction{LeftDown, LeftUp, Up, Down, RightUp, RightDown, RightH, LeftH};
}

std::ostream& operator << (std::ostream& os, const DXKDP_LIB::Direction& obj)
{
    using namespace DXKDP_LIB;

   switch(obj){
       case Direction::LeftDown:
            os << "Left Down";
       break;
       case Direction::LeftUp:
            os << "Left Up";
       break;
       case Direction::Up:
            os << "Up";
       break;
       case Direction::Down:
            os << "Down";
       break;
       case Direction::RightUp:
            os << "Right Up";
       break;
       case Direction::RightDown:
            os << "Right Down";
       break;
       case Direction::RightH:
            os << "RightH";
       break;
       case Direction::LeftH:
            os << "LeftH";
       break;
       default:
            os << "Undefined";
       break;
   }
   
   return os;
}

std::vector<DXKDP_LIB::Direction> decodePath(std::vector<cv::Point> path_, int pointIncr=2);

int main()
{
    
    int worldx = 10, worldy = 10;
    cv::Point CVworld(worldx, worldy);
    cv::Point tip(0,0);
    // cv::Point tip(0,9);

    cv::Point goal(0,9);
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

    // std::cout << "---------------------------------------\nCollisions: \n";
    //j controls the rows
    for(int j = 1; j < 6; j++){
        AStar::Vec2i collision;
        //i controls the columns
        for(int i = 0; i < 9; i++){
            collision.x = i;
            collision.y = j;
            generator.addCollision(collision);
            // std::cout << "Collision at: " << collision.x << " , " << collision.y << "\n";
            grid[j][i] = 1;
        }
    }

    //j controls the rows
    for(int j = 7; j < 9; j++){
        AStar::Vec2i collision;
        //i controls the columns
        for(int i = 1; i < 8; i++){
            collision.x = i;
            collision.y = j;
            generator.addCollision(collision);
            // std::cout << "Collision at: " << collision.x << " , " << collision.y << "\n";
            grid[j][i] = 1;
        }
    }

    // std::cout << "---------------------------------------\nPath found: \n";
    auto path = generator.findPath(src, dst);
    std::reverse(path.begin(), path.end());
    // for(auto i: path){
    //     std::cout << i.x << " , " << i.y << "\n";
    // }
    
    std::vector<cv::Point> cvPath = AStar::Vec2iToCvPointList(path);
    
    for(auto i: cvPath){
        grid[i.y][i.x] = 2;
        // std::cout << i << "\n";
    }
    
    

    std::cout << "---------------------------------------\nDecoded path: \n";
    std::vector<DXKDP_LIB::Direction> StrPath = decodePath(cvPath, 1);
    for(auto i: StrPath) std::cout << i << "\n";
    
    
    
    
    std::cout << "---------------------------------------\nGrid: \n";    
    std::cout << "\n  | ";
    for( int i = 0; i < 10; i++) std::cout << i << " ";
    std::cout << "\n-----------------------\n";
    for(int i = 0; i < grid.size(); i++){
        std::cout << i << " | ";
        for (int j = 0; j < grid[i].size(); j++)
        {
            if(grid[i][j] == 1 ) std::cout << "\033[1;31m1\033[0m ";
            else if (grid[i][j] == 2) std::cout << "\033[1;34m2\033[0m ";
            else std::cout << "\033[1;32m0\033[0m ";
        }
        std::cout << "\n";
    }
    std::cout << "\n\n";
}

std::vector<DXKDP_LIB::Direction> decodePath(std::vector<cv::Point> path_, int pointIncr){
    using namespace DXKDP_LIB;
    std::vector<Direction> decodedInstructions;
    int dx, dy, angleD, dxdy;
    bool xPositive, yPositive;
    double angleF;
    for(int i = 0; i < path_.size(); i++){
        if( i + pointIncr >= path_.size()) break;
        dx = path_[i+pointIncr].x - path_[i].x;
        dy = path_[i+pointIncr].y - path_[i].y;

        xPositive = (path_[i+1].x > path_[i].x) ? true : false;
        yPositive = (path_[i+1].y > path_[i].y) ? true : false;
        
        if(dy == 0) {
            angleD = 90;
            
            if(xPositive) decodedInstructions.push_back(Direction::RightH);   
            else decodedInstructions.push_back(Direction::LeftH);
        }
        else {
            dxdy = dx/dy;
            angleF = atan(dxdy) * 180 / PI;
            angleD = (int) angleF;

            /**
             * @todo find a less spaghetti-like implementation of this
             */
            if(angleD > 0 && yPositive) decodedInstructions.push_back(Direction::RightDown);
            else if (angleD > 0 && !yPositive) decodedInstructions.push_back(Direction::LeftUp);
            else if ( angleD < 0 && yPositive) decodedInstructions.push_back(Direction::LeftDown);
            else if ( angleD < 0 && !yPositive) decodedInstructions.push_back(Direction::RightUp);
            else if ( angleD == 0 && yPositive) decodedInstructions.push_back(Direction::Down);
            else decodedInstructions.push_back(Direction::Up);
  
            // std::cout << "tan(" << dx << " , " <<  dy << ") = " <<  angleD << "\n";
        }
        
    }
    return decodedInstructions;    
    
}
