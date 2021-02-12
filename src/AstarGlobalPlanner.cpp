/**
 * @file AstarGlobalPlanner.cpp
 * @author Mohammad Dehghani (m.dehghani94@gmail.com)
 *                                                     

    _/_/_/              _/                  _/                                      _/   
   _/    _/    _/_/    _/_/_/      _/_/_/  _/_/_/      _/_/_/  _/_/_/    _/_/_/          
  _/    _/  _/_/_/_/  _/    _/  _/    _/  _/    _/  _/    _/  _/    _/  _/    _/  _/     
 _/    _/  _/        _/    _/  _/    _/  _/    _/  _/    _/  _/    _/  _/    _/  _/      
_/_/_/      _/_/_/  _/    _/    _/_/_/  _/    _/    _/_/_/  _/    _/  _/    _/  _/       
                                   _/                                                    
                              _/_/                                                       

 * @brief The Implementation of A* algorithm as a Global planner
 * @version 0.1
 * @date 2018-12-06
 * 
 * @copyright Copyright (c) 2018
 * @TODO  Licence. No license defined. 
 */

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h> // To register the planner as BaseGlobalPlanner plugin
#include "AstarGlobalPlanner.h"
#include "GradientDescent.h"
#include <algorithm>

//Register the planner as BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar::Astar, nav_core::BaseGlobalPlanner)


bool* occupancyGridMap;


namespace astar
{

int Astar::nx_ = 0;
int Astar::ny_ = 0;
int Astar::ns_ = 0;


  Astar::Astar(){
  }

  Astar::Astar(ros::NodeHandle& nh)
  {
    ROSNodeHandle = nh;
  }

  Astar::Astar(string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void Astar::initialize(string name, costmap_2d::Costmap2DROS* costmap_ros)
  {

    if (!initialized_)
    {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      originX = costmap_->getOriginX();
      originY = costmap_->getOriginY();

      nx_ = costmap_->getSizeInCellsX();
      ny_ = costmap_->getSizeInCellsY();
      ns_ = nx_ * ny_;
      resolution = costmap_->getResolution();

      occupancyGridMap = new bool[ns_];
      for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
      {
        for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
        {
          unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

          if (cost == 0)
            occupancyGridMap[iy * nx_ + ix] = true;
          else
            occupancyGridMap[iy * nx_ + ix] = false;
        }
      }

      std::make_heap(queue_.begin(),queue_.end(), Cell());//////////////////////////////////////////////////////////////////
      
      ROS_INFO("Global planner initialized.");
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized.");
  }


  bool Astar::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped>& plan)
  {

    if (!initialized_)
    {
      ROS_ERROR("Initialization has not been done yet. Please call initialize() first");
      return false;
    }
  
    plan.clear();

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    double startX = start.pose.position.x;
    double startY = start.pose.position.y;
    double goalX = goal.pose.position.x;
    double goalY = goal.pose.position.y;

    worldToMap(startX, startY);
    worldToMap(goalX, goalY);

    int startCell;
    int goalCell;

    if (isCoordinateInBounds(startX, startY) && isCoordinateInBounds(goalX, goalY))
    {
      startCell = getCellIndex(startX, startY);

      goalCell = getCellIndex(goalX, goalY);
    }
    else
    {
      ROS_WARN("the start or goal is out of the map");
      return false;
    }

    if (isStartAndGoalValid(startCell, goalCell))
    {
      vector<int> bestPath;
      bestPath.clear();

      ROS_INFO("startCell: %d, gaolCell: %d", startCell, goalCell);


      bestPath = astarAlgorithm(startX, startY, goalX, goalY);

      if (bestPath.size() > 0)  //Global Planner found a path
      {
        for (int i = 0; i < bestPath.size(); i++)
        {

          float x = 0.0;
          float y = 0.0;

          float previous_x = 0.0;
          float previous_y = 0.0;

          int index = bestPath[i];
          int previous_index;
          getCellCoordinates(index, x, y);

          previous_index = (i==0) ? index : bestPath[i - 1];

          getCellCoordinates(previous_index, previous_x, previous_y);

          tf::Vector3 vectorToTarget;
          vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
          float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

          geometry_msgs::PoseStamped pose = goal;
        
          if(i != bestPath.size()-1)
          {
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
          }
          plan.push_back(pose);
        }
        return true;
      }

      else
      {
        ROS_WARN("Global Planner could not find the path!");
        return false;
      }
    }
    else
    {
      ROS_WARN("The start or goal are not valid");
      return false;
    }
  }


  void Astar::worldToMap(double &x, double &y)
  {

    x = x - originX;
    y = y - originY;
  }

  /**
    Function to get index of grid square on map given square coordinates
  **/
  int Astar::getCellIndex(float x, float y)
  {

    int cellIndex;

    float newX = x / (resolution);
    float newY = y / (resolution);

    /// TODO: Why toIndex does not work well? the problem might be the wrong data type conversion
    cellIndex = calculateCellIndex(newY, newX);
    return cellIndex;
  }

  void Astar::getCellCoordinates(int index, float &x, float &y)
  {
    x = getCellY(index) * resolution;
    y = getCellX(index) * resolution;

    x = x + originX;
    y = y + originY;
  }


  bool Astar::isCoordinateInBounds(float x, float y)
  {
    bool valid = true;
    if (x < 0 || x > (nx_ * resolution) || y < 0 || y > (ny_ * resolution))
      valid = false;

    return valid;
  }

  vector<int> Astar::astarAlgorithm(double startx, double starty, double goalx, double goaly)   //(int start, int goal)
  {
    
    vector<int> bestPath;

    float cellPot[ns_];

    for (uint i = 0; i < ns_; i++)
      cellPot[i] = H_value;



    bestPath = findPath( startx,  starty,  goalx,  goaly, cellPot);

    return bestPath;
  }

  vector<int> Astar::findPath(double startx, double starty, double goalx, double goaly, float cellPot[])
  {
    queue_.empty();

    vector<int> bestPath;
    vector<int> emptyPath;

      int startCell = getCellIndex(startx, starty);
      int goalCell = getCellIndex(goalx, goaly);

    Cell cell;

    cellPot[startCell] = 0;
    cell.setIndex(startCell);
    cell.setCost(cellPot[startCell] + calculateHeuristic(startCell, goalCell)); 

    queue_.push_back(cell);
    std::make_heap (queue_.begin(),queue_.end(),Cell());

    int currentCell = startCell;

    while (!queue_.empty() && cellPot[goalCell] == H_value) //queue is not empty or the value of the goalCell has not been calculated.
    {
      currentCell = queue_.begin()->getIndex(); //the cell with the highest cost 
      
      std::pop_heap(queue_.begin(), queue_.end(), Cell()); queue_.pop_back();
      
      vector<int> adjacentCells;
      adjacentCells = passableNeighbors(currentCell);

      for (uint i = 0; i < adjacentCells.size(); i++) 
      {
        if ( (cellPot[currentCell] + moveCost(currentCell, adjacentCells[i])) < cellPot[adjacentCells[i]] ) //The value of the adjacent cell has not been calculated earlier
        {
          cellPot[adjacentCells[i]] = cellPot[currentCell] + moveCost(currentCell, adjacentCells[i]);
          addNeighborToQueue(queue_, adjacentCells[i], goalCell, cellPot);
        }
      }
    }
    if (cellPot[goalCell] != H_value) 
    {
      GradientDescent gPath;
      gPath.gradSize(nx_,ny_);
      bestPath = gPath.pathFinder(cellPot, (startx/resolution), (starty/resolution), (goalx/resolution), (goaly/resolution));  
      return bestPath;
    }
    else
    {
      ROS_WARN("Global planner failed!");
      return emptyPath;
    }
  }

  void Astar::addNeighborToQueue(vector<Cell>& queue_, int neighbor, int goal, float cellPot[])
  {
    Cell cell;
    cell.setIndex(neighbor);
    cell.setCost(cellPot[neighbor] + calculateHeuristic(neighbor, goal));

    queue_.push_back(cell); std::push_heap(queue_.begin(), queue_.end(), Cell());

  }

  vector<int> Astar::passableNeighbors(int cellIndex)
  {
    int rowIndex = getCellX(cellIndex);
    int colIndex = getCellY(cellIndex);
    int neighborIndex;
    vector<int> openNeighbors;

    for (int i = -1; i <= 1; i++)
      for (int j = -1; j <= 1; j++)
      {
        if ((rowIndex + i >= 0) && (rowIndex + i < ny_) && (colIndex + j >= 0) &&  (colIndex + j < nx_) && (!(i == 0 && j == 0)))
        {
          neighborIndex = ((rowIndex + i) * nx_) + (colIndex + j);

          if (isOpen(neighborIndex)){

            openNeighbors.push_back(neighborIndex);
          }
        }
      }
    return openNeighbors;
  }


  bool Astar::isStartAndGoalValid(int start, int goal)
  {
    bool isvalid = true;
    bool isOpenStartCell = isOpen(start);
    bool isOpenGoalCell = isOpen(goal);

      if (!isOpenStartCell || !isOpenGoalCell) 
      {
        ROS_WARN("Start or Goal is not valid!");
        isvalid = false;
      }


    return isvalid;
  }


  float Astar::moveCost(int i1, int j1, int i2, int j2)
  {
    float moveCost = H_value; 

    if ((i1 == i2) || (j1 == j2))
        moveCost = 1;
    else
        moveCost = 1.414;

    return moveCost;
  }


  float Astar::moveCost(int gridSquareIndex1, int gridSquareIndex2)
  {
    int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

    i1 = getCellX(gridSquareIndex1);
    j1 = getCellY(gridSquareIndex1);
    i2 = getCellX(gridSquareIndex2);
    j2 = getCellY(gridSquareIndex2);

    return moveCost(i1, j1, i2, j2);
  }


  float Astar::calculateHeuristic(int gridSquareIndex, int goalGridSquare)
  {
    int x1 = getCellX(goalGridSquare);
    int y1 = getCellY(goalGridSquare);
    int x2 = getCellX(gridSquareIndex);
    int y2 = getCellY(gridSquareIndex);

    // return sqrt(abs(x1 - x2) + abs(y1 - y2)); //Eucleaden Distance
    return abs(x1 - x2) + abs(y1 - y2);     //Manhattan Distance
    // return std::max(abs(x1 - x2) , abs(y1 - y2)); //Diagonal Distance
  }

  int Astar::calculateCellIndex(int i, int j) 
  {
    return (i * nx_) + j;
  }
  /**
   * @brief get the row index
   * 
   * @TODO: change the name of the functions CellX: Column, CellY: Row
   * 
   * @param index 
   * @return int 
   */
  int Astar::getCellX(int index) 
  {
    return index / nx_;
  }

  /**
   * @brief get the column index
   * 
   * @TODO: change the name of the functions CellX: Column, CellY: Row
   * 
   * @param index 
   * @return int 
   */
  int Astar::getCellY(int index) 
  {
    return index % nx_;
  }


  bool Astar::isOpen(int i, int j)
  {
    int cellIndex = (i * nx_) + j;

    return occupancyGridMap[cellIndex];
  }


  bool Astar::isOpen(int cell)
  {
    return occupancyGridMap[cell];
  }


      int Astar::getWidth(){
        return nx_;
      };

      int Astar::getHeight(){
        return ny_;
      };

      int Astar::getMapCellNum(){
        return ns_;
      };




};


