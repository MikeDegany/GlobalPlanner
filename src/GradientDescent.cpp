
/**
 * @file GradientDescent.cpp
 * @author Mohammad Dehghani (m.dehghani94@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2018-12-06
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include "GradientDescent.h"
#include "AstarGlobalPlanner.h"


///@TODO: make the H_value global available in all files 
const float H_value = 1e10; //

namespace astar {

GradientDescent::GradientDescent(){
    xGradient_ = yGradient_ = NULL;
}

GradientDescent::~GradientDescent() {

    if (xGradient_)
        delete[] xGradient_;
    if (yGradient_)
        delete[] yGradient_;
}

void GradientDescent::gradSize(int x, int y) {
    if (xGradient_)
        delete[] xGradient_;
    if (yGradient_)
        delete[] yGradient_;
    xGradient_ = new float[x * y];
    yGradient_ = new float[x * y];
}
   
std::vector<int> GradientDescent::pathFinder(float cellPot[], double start_x, double start_y, double goal_x, double goal_y) {

    
    std::vector<int> path;  // The main path that would be returned providing the path is found correctly
    std::vector<int> emptypath; //Should the pathFinder could not find the path, it will return an empty path


    int startIndex = round(start_x) + round(start_y) * Astar::getWidth(); //used for checking if the path got to the start from the goal
    // int C_current = round(goal_x) + round(goal_y) * nx_;     //current cell
    int C_current = (int)goal_x + (int)goal_y * Astar::getWidth();     //current cell
      
    // double dx = goal_x - (int)goal_x;
    // double dy = goal_y - (int)goal_y;
    float dx = goal_x - (int)goal_x;
    float dy = goal_y - (int)goal_y;

    ///initialize xGradient_ and yGradient_ with 0
    memset(xGradient_, 0, Astar::getMapCellNum() * sizeof(float));
    memset(yGradient_, 0, Astar::getMapCellNum() * sizeof(float));

    float nx;
    float ny;
    int eof; // End-Of-Life (Ctrl + d in Linux)


    ///@TODO: Enable End of Life

    while ( eof ++< Astar::getMapCellNum()) {
    // while ( (eof = cin.get()) =! EOF) {
        
        nx = C_current % Astar::getWidth() + dx;
        ny = C_current / Astar::getWidth() + dy;

        if (fabs(C_current - startIndex) < 2) {
            path.push_back(startIndex);
            //ROS_INFO("[GD] pathOK");
            return path;
        } 

        /**
         *  Check if the current cell is out the bounds 
         */
        if (C_current < Astar::getWidth() || C_current > Astar::getWidth() * Astar::getHeight() - Astar::getWidth()) 
        {
            ROS_WARN("[GD] One of the poses of the path goes out of the bounds");
            return emptypath;
        }

        path.push_back(C_current);

        int npath = path.size();


        int stcnx = C_current + Astar::getWidth();
        int stcpx = C_current - Astar::getWidth();

        /**
         * If any of the cells in 8-connected neighborhood is not calculated or an obstacle, move to the cell with lowest value
         */
        if ((cellPot[C_current] >= H_value) || (cellPot[C_current + 1] >= H_value) || (cellPot[C_current - 1] >= H_value)
                || (cellPot[stcnx] >= H_value) || (cellPot[stcnx + 1] >= H_value) || (cellPot[stcnx - 1] >= H_value)
                || (cellPot[stcpx] >= H_value) || (cellPot[stcpx + 1] >= H_value) || (cellPot[stcpx - 1] >= H_value)
            ) 
        {
            ROS_WARN("[GD] the path is near shelf at %d, Flee! ;)", (int)path.size());
            int Cell_min = C_current;
            int Pot_min = cellPot[C_current];
            int Cell_temp = stcpx - 1;
            if (cellPot[Cell_temp] < Pot_min) {
                Pot_min = cellPot[Cell_temp];
                Cell_min = Cell_temp;
            }
            Cell_temp++;
            if (cellPot[Cell_temp] < Pot_min) {
                Pot_min = cellPot[Cell_temp];
                Cell_min = Cell_temp;
            }
            Cell_temp++;
            if (cellPot[Cell_temp] < Pot_min) {
                Pot_min = cellPot[Cell_temp];
                Cell_min = Cell_temp;
            }
            Cell_temp = C_current - 1;
            if (cellPot[Cell_temp] < Pot_min) {
                Pot_min = cellPot[Cell_temp];
                Cell_min = Cell_temp;
            }
            Cell_temp = C_current + 1;
            if (cellPot[Cell_temp] < Pot_min) {
                Pot_min = cellPot[Cell_temp];
                Cell_min = Cell_temp;
            }
            Cell_temp = stcnx - 1;
            if (cellPot[Cell_temp] < Pot_min) {
                Pot_min = cellPot[Cell_temp];
                Cell_min = Cell_temp;
            }
            Cell_temp++;
            if (cellPot[Cell_temp] < Pot_min) {
                Pot_min = cellPot[Cell_temp];
                Cell_min = Cell_temp;
            }
            Cell_temp++;
            if (cellPot[Cell_temp] < Pot_min) {
                Pot_min = cellPot[Cell_temp];
                Cell_min = Cell_temp;
            }
            C_current = Cell_min;
            dx = 0;
            dy = 0;

            if (cellPot[C_current] >= H_value) {
                ROS_WARN("[GD]  H_value-filled Neighborhood");
                return emptypath;
            }
        }
        else {  // The cell is in a completely Open Neighborhood



            //## bilinear interpolated gradient ##//
            /**
             * 
             * Should you need detailed information on how the interpolated points has been calculated, please take a look at the address below
             * @url: http://www.geocomputation.org/1999/082/gc_082.htm
             * 
             */
            // calculate the gradient of 4 neighbors and immediately interpolate the gradient vectors
            cellGradient(cellPot, C_current);
            cellGradient(cellPot, C_current + 1);
            cellGradient(cellPot, stcnx);
            cellGradient(cellPot, stcnx + 1);

            float x = xGradient_[C_current] + (xGradient_[C_current + 1]-xGradient_[C_current])*dx + (xGradient_[stcnx]-xGradient_[C_current]) * dy + (xGradient_[C_current]-xGradient_[C_current + 1]-xGradient_[stcnx]+xGradient_[stcnx + 1]) * dx * dy ;
            float y = yGradient_[C_current] + (yGradient_[C_current + 1]-yGradient_[C_current])*dx + (yGradient_[stcnx]-yGradient_[C_current]) * dy + (yGradient_[C_current]-yGradient_[C_current + 1]-yGradient_[stcnx]+yGradient_[stcnx + 1]) * dx * dy ;


            if (x == 0.0 && y == 0.0) {
                ROS_WARN("[GD] Zero GD");
                return emptypath;
            }

            // scale the step toward the start(path destination)
            float ss = 0.5 / hypot(x, y);
            dx += x * ss;
            dy += y * ss;

            // Move
            if (dx > 1.0) {
                C_current++;
                dx -= 1.0;
            }
            if (dx < -1.0) {
                C_current--;
                dx += 1.0;
            }
            if (dy > 1.0) {
                C_current += Astar::getWidth();
                dy -= 1.0;
            }
            if (dy < -1.0) {
                C_current -= Astar::getWidth();
                dy += 1.0;
            }

        } //end the else;  The cell is in a completely Open Neighborhood

    } // end while    
    return path;
}// end of pathFinder member function


float GradientDescent::cellGradient(float cellPot[], int n) {


    float cv = cellPot[n];
    float dx = 0.0;
    float dy = 0.0;

    if (n < Astar::getWidth() || n > Astar::getWidth() * Astar::getHeight() - Astar::getWidth())    // the cell is out of the bound 
        return 0.0;

    if (xGradient_[n] + yGradient_[n] > 0.0)    // Gradient has been calculated
        return 1.0;


    // If the cell in non-calculated area(e.g. obstacle), the gradient would be high => 
    //      the slope would be high like continental shelf and continental slope ;))    
    if (cellPot[n] >= H_value) {
        if (cellPot[n - 1] < H_value)
            dx = -highSlope_;
        else if (cellPot[n + 1] < H_value)
            dx = highSlope_;

        if (cellPot[n - Astar::getWidth()] < H_value)
            dy = -highSlope_;
        else if (cellPot[n + Astar::getWidth()] < H_value)
            dy = highSlope_;
    }
    else
    {
        if (cellPot[n - 1] < H_value)
            dx += cellPot[n - 1] - cellPot[n];
        if (cellPot[n + 1] < H_value)
            dx += cellPot[n] - cellPot[n + 1];   // dx = pot(n-1) - pot(n) + pot(n) - pot(n+1) = pot(n-1) - pot(n+1)

        if (cellPot[n - Astar::getWidth()] < H_value)
            dy += cellPot[n - Astar::getWidth()] - cellPot[n];
        if (cellPot[n + Astar::getWidth()] < H_value)
            dy += cellPot[n] - cellPot[n + Astar::getWidth()];   // dy = pot(n-nx) - pot(n) + pot(n) - pot(n+nx) = pot(n-nx) - pot(n+nx)
    }

    // normalize the gradient values 
    float norm = hypot(dx, dy);
    if (norm > 0) {
        norm = 1.0 / norm;
        xGradient_[n] = norm * dx;
        yGradient_[n] = norm * dy;
    }
    return norm;
}// end GradientDescent member function

} //end namespace global_planner

