
#ifndef _GRADIENT_PATH_H
#define _GRADIENT_PATH_H
/**
 * @file GradientDescent.h
 * @author Mohammad Dehghani (m.dehghani94@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2018-12-06
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#include <algorithm>
#include <vector>
#include <math.h>
#include "AstarGlobalPlanner.h"


namespace astar {

    class GradientDescent{
        friend class Astar;
        public:
            /**
             * @brief Construct a new Gradient Descent object
             * 
             */
            GradientDescent();
            
            /**
             * @brief Destroy the Gradient Descent object
             * 
             */
            ~GradientDescent();
            
            /**
             * @brief Set the Size of the gradient vectors xGradient_ and yGradient_
             * 
             * @param x
             * @param y 
             */
            void gradSize(int x, int y);


            /**
             * @brief Find the path using gradient method 
             * 
             * @param cellPot Potentials(calculated values) of the cells
             * @param start_x x coordinate of start cell
             * @param start_y y coordinate of start cell
             * @param end_x x coordinate of end cell
             * @param end_y y coordinate of end cell

             * @return std::vector<int> path: the final plan to be considered
             */
            std::vector<int> pathFinder(const float cellPot[], double start_x, double start_y, double end_x, double end_y);

        private:
            /**
             * @brief calculate the gradient at each cell
             * 
             * @param cellPot Potentials of the cells 
             * @param n the index of the desired cell
             * @return float the valure of the gradient of the desired cell
             */
            float cellGradient(const float cellPot[], int n);

            float *xGradient_, *yGradient_; // the Gradient vectors 

            static const unsigned char highSlope_ = 200;
    }; // end class GradeintDescent

} //end namespace astar
#endif