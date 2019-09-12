
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <costmap_2d/costmap_2d.h>

#include "../include/aco_ros/ant_colony_ros.h"

#include <pluginlib/class_list_macros.h>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(Aco_planner::AcoPlanner, nav_core::BaseGlobalPlanner)

//inline vector <int> findFreeNeighborCell (int CellID);

namespace Aco_planner
{
int value;
int mapSize;
// bool *OGM;
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();
float tBreak; // coefficient for breaking ties
//ofstream MyExcelFile ("RA_result.xlsx", ios::trunc);

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

//Default Constructor
AcoPlanner::AcoPlanner()
{
}
AcoPlanner::~AcoPlanner()
{
}
AcoPlanner::AcoPlanner(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;
}

AcoPlanner::AcoPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  OccupancyGridMap OGM = new OccupancyGridMap();
  initialize(name, costmap_ros);
}

void AcoPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{

  if (!initialized_)
  {

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);

    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();

    width = costmap_->getSizeInCellsX();
    cout << "Width of Costmap " << width << endl;
    OGM->setWidth(width);
    cout << "Width of OGM " << OGM->getWidth() << endl;

    height = costmap_->getSizeInCellsY();
    cout << "Height of Costmap " << height << endl;
    OGM->setHeight(height);
    cout << "Height of OGM " << OGM->getHeight() << endl;

    resolution = costmap_->getResolution();
    cout << "Resolution of Costmap " << resolution << endl;
    OGM->setResolution(resolution);
    cout << "Resolution of OGM " << OGM->getResolution() << endl;

    mapSize = width * height;
    tBreak = 1 + 1 / (mapSize);
    value = 0;

    OGM_data = (int **)malloc(sizeof(int *) * height);
    for (int i = 0; i < width; i++)
      OGM_data[i] = (int *)malloc(sizeof(int) * width);

    for (unsigned int iy = 0; iy < height; iy++)
    {
      for (unsigned int ix = 0; ix < width; ix++)
      {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
        //cout<<cost;
        if (cost == 0)
          OGM_data[iy][ix] = 0;
        else if (cost == 255)
          OGM_data[iy][ix] = -1;
        else if (cost == 254)
          OGM_data[iy][ix] = 100;
        else if (cost == 253)
          OGM_data[iy][ix] = 99;
        else
          OGM_data[iy][ix] = char(1 + (97 * (cost - 1)) / 251);
      }
    }

    // OGM = new bool[mapSize];
    // for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    // {
    //   for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
    //   {
    //     unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
    //     //cout<<cost;
    //     if (cost == 0)
    //       OGM[iy * width + ix] = true;
    //     else
    //       OGM[iy * width + ix] = false;
    //   }
    // }
    // OccupancyGridMap *OGM_t = new OccupancyGridMap(width, height, resolution, OGM_data, 1, 1);

    //MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;

    ROS_INFO("Aco planner initialized successfully");
    initialized_ = true;
    // cout << "OGM_data :: " << OGM_data << endl;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
}

bool AcoPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                          std::vector<geometry_msgs::PoseStamped> &plan)
{

  if (!initialized_)
  {
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

  plan.clear();

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }
  ACO *aco = new ACO();
  OccupancyGridMap *map = new OccupancyGridMap(width, height, resolution, OGM_data, 1, 1);
  tf::Stamped<tf::Pose> goal_tf;
  tf::Stamped<tf::Pose> start_tf;

  poseStampedMsgToTF(goal, goal_tf);
  poseStampedMsgToTF(start, start_tf);

  // convert the start and goal positions

  float startX = start.pose.position.x;
  float startY = start.pose.position.y;

  float goalX = goal.pose.position.x;
  float goalY = goal.pose.position.y;

  getCorrdinate(startX, startY);
  getCorrdinate(goalX, goalY);

  int startCell;
  int goalCell;

  if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
  {
    startCell = convertToCellIndex(startX, startY);
    goalCell = convertToCellIndex(goalX, goalY);

    //MyExcelFile << startCell <<"\t"<< start.pose.position.x <<"\t"<< start.pose.position.y <<"\t"<< goalCell <<"\t"<< goal.pose.position.x <<"\t"<< goal.pose.position.y;
  }
  else
  {
    ROS_WARN("the start or goal is out of the map");
    return false;
  }

  /////////////////////////////////////////////////////////

  // call global planner

  if (aco->isStartAndGoalCellsValid(map, startCell, goalCell))
  {

    vector<int> bestPath;
    bestPath.clear();
    bestPath = AcoPlannerROS(startCell, goalCell);

    //if the global planner find a path
    if (bestPath.size() > 0)
    {
      // convert the path
      for (int i = 0; i < bestPath.size(); i++)
      {
        float x = 0.0;
        float y = 0.0;

        int index = bestPath[i];

        convertToCoordinate(index, x, y);

        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
      }

      float path_length = 0.0;

      std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
      geometry_msgs::PoseStamped last_pose;
      last_pose = *it;
      it++;
      for (; it != plan.end(); ++it)
      {
        path_length += hypot((*it).pose.position.x - last_pose.pose.position.x,
                             (*it).pose.position.y - last_pose.pose.position.y);
        last_pose = *it;
      }
      cout << "The global path length: " << path_length << " meters" << endl;
      //MyExcelFile << "\t" <<path_length <<"\t"<< plan.size() <<endl;
      //publish the plan

      return true;
    }
    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }
  }

  else
  {
    ROS_WARN("Not valid start or goal");
    return false;
  }
}
void AcoPlanner::getCorrdinate(float &x, float &y)
{

  x = x - originX;
  y = y - originY;
}

int AcoPlanner::convertToCellIndex(float x, float y)
{

  int cellIndex;

  float newX = x / resolution;
  float newY = y / resolution;

  cellIndex = getCellIndex(newY, newX);

  return cellIndex;
}

void AcoPlanner::convertToCoordinate(int index, float &x, float &y)
{

  x = getCellColID(index) * resolution;

  y = getCellRowID(index) * resolution;

  x = x + originX;
  y = y + originY;
}

bool AcoPlanner::isCellInsideMap(float x, float y)
{
  bool valid = true;

  if (x > (width * resolution) || y > (height * resolution))
    valid = false;

  return valid;
}

void AcoPlanner::mapToWorld(double mx, double my, double &wx, double &wy)
{
  costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  wx = costmap->getOriginX() + mx * resolution;
  wy = costmap->getOriginY() + my * resolution;
}

vector<int> AcoPlanner::AcoPlannerROS(int startCell, int goalCell)
{

  string sourcePath = "/root/catkin_ws/src/rover_ws/src/aco_ros/map_folder/";
  string fileName = "udacity_office_map.pgm";

  // vector<int> bestPath;
  Path *bestPath = new Path();

  long double initPheromoneValue = 50;
  long double alpha = 0.5;
  long double beta = 2.5;
  long double evaporationRate = 0.2;
  int Q = 2;
  unsigned int numberOfAnts = 50;
  int numberOfIterations = 15;

  OccupancyGridMap *map = new OccupancyGridMap(width, height, resolution, OGM_data, 1, 1);
  map->importMapLayout(sourcePath, fileName.c_str());

  ACO *aco = new ACO(initPheromoneValue, alpha, beta, evaporationRate, Q, numberOfAnts, numberOfIterations, map);
  // long double initPheromoneValue ,long double alpha , long double beta , long double evaporationRate ,int Q , unsigned int numberOfAnts,int numberOfIterations):GenericACO(initPheromoneValue,alpha,beta,evaporationRate,Q,numberOfAnts, numberOfIterations

  OccupancyGridMap *map1 = new OccupancyGridMap();
  cout << map1->getHeight() << " ";

  timespec time1, time2;
  /* take current time here */
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

  bestPath = aco->findPathAco(map, startCell, goalCell, true);

  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

  cout << "time to generate best global path by ACO planner = " << (diff(time1, time2).tv_sec) * 1e3 + (diff(time1, time2).tv_nsec) * 1e-6 << " microseconds" << endl;

  // MyExcelFile <<"\t"<< (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 ;

  return bestPath->getPath();
}

/*******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and column of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/

// vector <int> AcoPlanner::findFreeNeighborCell (int CellID){

//   int rowID=getCellRowID(CellID);
//   int colID=getCellColID(CellID);
//   int neighborIndex;
//   vector <int>  freeNeighborCells;

//   for (int i=-1;i<=1;i++)
//     for (int j=-1; j<=1;j++){
//       //check whether the index is valid
//      if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
//   neighborIndex = getCellIndex(rowID+i,colID+j);
//         if(isFree(neighborIndex) )
//       freeNeighborCells.push_back(neighborIndex);
//   }
//     }
//     return  freeNeighborCells;

// }

/*******************************************************************************/
//Function Name: isStartAndGoalCellsValid
//Inputs: the start and Goal cells
//Output: true if the start and the goal cells are valid
//Description: check if the start and goal cells are valid
/*********************************************************************************/
// bool AcoPlanner::isStartAndGoalCellsValid(int startCell,int goalCell)
// {
//  bool isvalid=true;
//  bool isFreeStartCell=isFree(startCell);
//  bool isFreeGoalCell=isFree(goalCell);
//     if (startCell==goalCell)
//     {
//     //cout << "The Start and the Goal cells are the same..." << endl;
//     isvalid = false;
//     }
//    else
//    {
//       if (!isFreeStartCell && !isFreeGoalCell)
//       {
//   //cout << "The start and the goal cells are obstacle positions..." << endl;
//         isvalid = false;
//       }
//       else
//       {
//   if (!isFreeStartCell)
//   {
//     //cout << "The start is an obstacle..." << endl;
//     isvalid = false;
//   }
//   else
//   {
//       if(!isFreeGoalCell)
//       {
//         //cout << "The goal cell is an obstacle..." << endl;
//         isvalid = false;
//       }
//       else
//       {
//         if (findFreeNeighborCell(goalCell).size()==0)
//         {
//     //cout << "The goal cell is encountred by obstacles... "<< endl;
//     isvalid = false;
//         }
//         else
//         {
//     if(findFreeNeighborCell(startCell).size()==0)
//     {
//       //cout << "The start cell is encountred by obstacles... "<< endl;
//       isvalid = false;
//     }
//         }
//       }
//   }
//       }
//   }
//  return isvalid;
// }

//  float  AcoPlanner::getMoveCost(int i1, int j1, int i2, int j2){
//    float moveCost=INFINIT_COST;//start cost with maximum value. Change it to real cost of cells are connected
//    //if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
//    if((j2==j1+1 && i2==i1+1)||(i2==i1-1 && j2==j1+1) ||(i2==i1-1 && j2==j1-1)||(j2==j1-1 && i2==i1+1)){
//      //moveCost = DIAGONAL_MOVE_COST;
//      moveCost = 1.4;
//    }
//     //if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
//    else{
//      if ((j2==j1 && i2==i1-1)||(i2==i1 && j2==j1-1)||(i2==i1+1 && j2==j1) ||(i1==i2 && j2==j1+1)){
//        //moveCost = MOVE_COST;
//        moveCost = 1;
//      }
//    }
//    return moveCost;
//  }

//   float  AcoPlanner::getMoveCost(int CellID1, int CellID2){
//    int i1=0,i2=0,j1=0,j2=0;

//    i1=getCellRowID(CellID1);
//    j1=getCellColID(CellID1);
//    i2=getCellRowID(CellID2);
//    j2=getCellColID(CellID2);

//     return getMoveCost(i1, j1, i2, j2);
//  }

//  //verify if the cell(i,j) is free
//  bool  AcoPlanner::isFree(int i, int j){
//     int CellID = getCellIndex(i, j);
//     return OGM[CellID];

//  }

//   //verify if the cell(i,j) is free
//  bool  AcoPlanner::isFree(int CellID){
//     return OGM[CellID];
//  }
}; // namespace Aco_planner

bool operator<(cells const &c1, cells const &c2) { return c1.fCost < c2.fCost; }