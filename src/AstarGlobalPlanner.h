/**
 * @file AstarGlobalPlanner.h
 * @author Mohammad Dehghani (m.dehghani94@gmail.com)
 *                                                     

    _/_/_/              _/                  _/                                      _/   
   _/    _/    _/_/    _/_/_/      _/_/_/  _/_/_/      _/_/_/  _/_/_/    _/_/_/          
  _/    _/  _/_/_/_/  _/    _/  _/    _/  _/    _/  _/    _/  _/    _/  _/    _/  _/     
 _/    _/  _/        _/    _/  _/    _/  _/    _/  _/    _/  _/    _/  _/    _/  _/      
_/_/_/      _/_/_/  _/    _/    _/_/_/  _/    _/    _/_/_/  _/    _/  _/    _/  _/       
                                   _/                                                    
                              _/_/                                                       

 * @brief The Library for Implementation of A* algorithm as a Global planner
 * @version 0.1
 * @date 2018-12-06
 * 
 * @copyright Copyright (c) 2018
 * @TODO  Licence. No license defined. 
 */

#ifndef ASTAR_H
#define ASTAR_H

#include <string.h>
#include <tf/tf.h>
#include <set>

/** for global planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;
using std::string;



/**
 * Cell: a square in the grid, e.g. a pixel (picture element) in a picture
 * @brief Cell(i,cost): ordered pair containing index and the assigned value of the cell
 * data member i_ : The index of the cell
 * data member cost_ : The value of the cell
 * 
 */
class Cell{
  public:
    Cell(){}
    Cell(int index, float cost){
      i_ = index;
      cost_ = cost;
      }
    int i_;
    float cost_; 
};

namespace astar
{

  class Astar : public nav_core::BaseGlobalPlanner
  {
    friend class GradientPath;
    public:

      /**
       * @brief Default constructor 
       * 
       */
      Astar();
      
      Astar(ros::NodeHandle &);

      /**
       * @brief Constructor to initialize the costmap and the name of the planner
       * 
       * @param name  The name of the planner 
       * @param costmap_ros The map that will be used for mapping 
       */
      Astar(string name, costmap_2d::Costmap2DROS* costmap_ros);

      
      /** overriden member functions of the base-class nav_core::BaseGlobalPlanner **/

      /**
       * @brief To initialize the costmap
       * 
       * @param name The name of the planner 
       * @param costmap_ros The map that will be used for planning 
       */
      void initialize(string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief overridden method for planning 
       * 
       * @param start 
       * @param goal 
       * @param plan The final plan will be stored in the parameter std::vector<geometry_msgs::PoseStamped>& plan x 
       * @return true if the planning was successful
       * @return false if the planning was not successful
       */
      bool makePlan(const geometry_msgs::PoseStamped &start,
                    const geometry_msgs::PoseStamped &goal,
                    vector<geometry_msgs::PoseStamped> &plan);



      /** Helper methods**/


      /**
       * @brief Get the Width and Height and the number of pixels(cells) of the map
       * 
       * @return int 
       */
       static int getWidth();

       static int getHeight();

       static int getMapCellNum();

      /**
       * @brief To adjust the coordinates w.r.t the map
       * 
       * @param x 
       * @param y 
       */
      void worldToMap(double &x, double &y);
      
      /**
       * @brief Give the coordinates of a Cell, Get the Cell Index on the map 
       * 
       * @param x 
       * @param y 
       * @return int 
       */
      int getCellIndex(float x, float y);

      /**
       * @brief Give the Index of a Cell, Get the Cell coordinates on the map
       * 
       * @param index The index of a cell
       * @param x 
       * @param y 
       */
      void getCellCoordinates(int index, float &x, float &y);

      /**
       * @brief Check if the cell coordinates are inside the boundries of the map
       * 
       * @param x 
       * @param y 
       * @return true The cell is in map
       * @return false The cell is out of map
       */
      bool isCoordinateInBounds(float x, float y);

      /**
       * @brief Give start and goal cells and implement A* algorithm
       * 
       * @param startGridSquare 
       * @param goalGridSquare 
       * @return vector<int> 
       */
      vector<int> astarAlgorithm(double startx, double starty, double goalx, double goaly);//(int start, int goal);

      /**
       * @brief 
       * 
       * @param start
       * @param goal
       * @param cellPot 
       * @return vector<int> 
       */
      vector<int> findPath(double startx, double starty, double goalx, double goaly, float cellPot[]);
      /**
       * @brief To find the path
       * 
       * @param start 
       * @param goal
       * @param cellPot 
       * @return vector<int> 
       */
      vector<int> pathFinder(int start, int goal, float cellPot[]);

      /**
       * @brief To add the passable neighbor to the queue_
       * 
       * @param OPL 
       * @param neighbor
       * @param goal
       * @param cellPot 
       */
      void addNeighborToQueue(vector<Cell> &OPL, int neighbor, int goal, float cellPot[]);

      /**
       * @brief return the adjacent cells that are open 
       * 
       * @param cellIndex current cell
       * @return vector<int> 
       */
      vector<int> passableNeighbors(int cellIndex);

      /**
       * @brief To validate the start and goal cells 
       * 
       * @param start 
       * @param goal
       * @return true 
       * @return false 
       */
      bool isStartAndGoalValid(int start, int goal);

      /**
       * @brief the cost of traversing from one cell to an adjacent cell given the index of the cells 
       * 
       * @param cellIndex1 
       * @param cellIndex2 
       * @return float 
       */
      float moveCost(int cellIndex1, int cellIndex2);

      /**
       * @brief the cost of traversing from one cell to an adjacent cell given the coordinates of the cells 
       * 
       * @param i1 x coorinate of the current cell 
       * @param j1 y coorinate of the current cell 
       * @param i2 x coorinate of the adjacent cell 
       * @param j2 y coorinate of the adjacent cell
       * @return float The return value would be 1 or 1.4 based on the type of neighborhood
       */
      float moveCost(int , int , int , int );

      /**
       * @brief To calculate the heuristic (Eucleaden or Manhattan Distance )
       * 
       * @param gridSquareIndex 
       * @param goalGridSquare 
       * @return float 
       */
      float calculateHeuristic(int cell, int goal);

      /**
       * @brief Get the Cell x coordinate 
       * 
       * @param index 
       * @return int x coordinate of the cell
       */


      int calculateCellIndex(int i, int j); 



      int getCellX(int index);
      /**
       * @brief Get the Cell y coordinate
       * 
       * @param index 
       * @return int y coordinate of the cell 
       */
      int getCellY(int index);
      /**
       * @brief Check if the cell is open
       * 
       * @param gridSquareIndex 
       * @return true 
       * @return false 
       */
      bool isOpen(int cell); 
      /**
       * @brief Check if the cell is open given the coordinates 
       * 
       * @param i 
       * @param j 
       * @return true 
       * @return false 
       */
      bool isOpen(int i, int j);

      static const float H_value = 1e10; 
      


    private:
      static int nx_, ny_, ns_;
      /**
       * @brief 
       * 
       * @param x 
       * @param y 
       * @return int 
       */
      inline int toIndex(float x, float y) {
            return x + nx_ * y;
      }
      
      ros::NodeHandle ROSNodeHandle;
      bool initialized_;
      vector<Cell> queue_;
      costmap_2d::Costmap2DROS *costmap_ros_;
      costmap_2d::Costmap2D *costmap_;
      float originX;
      float originY;
      float resolution;
      //Assigned value for non-calculated nodes

  };
};
#endif