/* iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. 

 * Website: http://www.iroboapp.org/index.php?title=IPath
 * Contact: 
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OCCUPANCYGRIDMAP_H
#define OCCUPANCYGRIDMAP_H

// #include "Map.h"
#include <limits.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>

using namespace std;

/**
* @class OccupancyGridMap
* @brief A class that represents an occupancy grid map 
*/
class OccupancyGridMap //: public Map
{

public:
  //define constants
  static const int FREE = 0;       //!< for free cell
  static const int OBSTACLE = 100; //!< for obstacles
  static const int UNKNOWN = -1;   //!< for unknown cell

  static constexpr float MOVE_COST = 1.0;          //!< horizental or vertical move
  static constexpr float DIAGONAL_MOVE_COST = 1.4; //!< Diagonal move

  static constexpr float INFINIT_COST = INT_MAX; //!< cost of non connected nodes

  /**
   * @brief default constructor to initialize map
   */
  OccupancyGridMap(int w = 0, int h = 0, float res = 0.0);
  /**
   * @brief default destructor
   */
  ~OccupancyGridMap();
  /**
  * @brief four-argument constructor to initialize map
  */
  OccupancyGridMap(int w, int h, float res, int **mapMatrix, int obsSize, float obsRatio);
  /**
  * @brief constructor that loads a map from a file
  */
  OccupancyGridMap(ifstream *mpgMapFile);
  /**
  * @brief Copy constructor for the OccupancyGridMapGridMap
  */
  OccupancyGridMap(OccupancyGridMap *map);

  void setObstacle(int i, int j);
  void setFree(int i, int j);
  void setUnknown(int i, int j);
  void setCell(int i, int j, int occupancy); //give the cell(i,j) the value of occupancy

  /**
  * @brief it is used to get the move cost between 2 cells
  * @param i1
  * @param j1
  * @param i2
  * @param j2
  * @return move cost
  */
  float getMoveCost(int i1, int j1, int i2, int j2);
  /**
  * @brief it is used to get the move cost between 2 cells
  * @param CellID1
  * @param CellID2
  * @return move cost 
  */
  float getMoveCost(int CellID1, int CellID2);

  /**
  * @brief it is used to check if the cell is an obstacle cell
  * @param i
  * @param j
  * @return true if the cell is an Obstacle
  */
  bool isObstacle(int i, int j);
  /**
  * @brief it is used to check if the cell is a free cell
  * @param i
  * @param j
  * @return true if the cell is Free
  */
  bool isFree(int i, int j);
  /**
  * @brief it is used to check if the cell is unknown cell
  * @param i
  * @param j 
  * @return true if the cell is Unknown
  */
  bool isUnknown(int i, int j);

  /**
  * @brief it is used to check if the cell is a free cell
  * @param CellID
  * @return true if the cell is Free
  */
  bool isFree(int CellID); //returns true if the cell is Free
  /**
  * @brief it is used to check if the cell is an obstacle cell
  * @param CellID
  * @return true if the cell is an Obstacle
  */
  bool isObstacle(int CellID); //return true if the cell is an Obstacle
  /**
  * @brief it is used to check if the cell is unknown cell
  * @param CellID
  * @return true if the cell is Unknown
  */
  bool isUnknown(int CellID); //returns true if the cell is Unknown

  /*override the method defined in Map.h by adding ratio of obstacle*/
  /**
  * @brief it is used to write the MapLayout in a file; override the method defined in Map.h by adding ratio of obstacle
  * @param file_name
  * @param mapMatrix 
  */
  void exportMapLayout(const char *file_name, int **mapMatrix);

  // Map header file functions definitions
  //Define Mutators
  void setWidth(int w);
  void setHeight(int h);
  void setResolution(float res);
  void setMapLayout(int **mapMatrix);
  void setObstacleSize(int obsSize);
  void setObstacleRatio(float obsRatio);

  //Define Accessors
  int getWidth();
  int getHeight();
  float getResolution();
  int **getMapLayout();
  int getObstacleSize();
  float getObstacleRatio();

  /**
	* @brief it is used to get the index of the cell to be used in Path.
	* @param i
	* @param j
	* @return cell index
	*/
  int getCellIndex(int i, int j);
  /**
	* @brief get the row ID from cell index
	* @param index
	* @return cell row ID
	*/
  int getCellRowID(int index);
  /**
	* @brief get colunm ID from cell index
	* @param index
	* @return cell col ID
	*/
  int getCellColID(int index);

  /**
	* @brief it is used to read the MapLayout from a file
	* @param path_name
	* @param file_name
	*/
  void importMapLayout(string path_name, const char *file_name);
  /**
	* @brief it is used to write the MapLayout in a file
	* @param file_name
	* @param mapMatrix
	*/
  // void exportMapLayout(const char *file_name, int **mapMatrix);

private:
  int width;           //!<  Map width [cells]
  int height;          //!<  Map height [cells]
  float resolution;    //!< The map resolution [m/cell]
  int **mapLayout;     //!<  the map data structure that contains all cells of the environment
  int obstacleSize;    //!< The size of the obstacles
  float obstacleRatio; //!< The ratio of obstacles in the map
};

#endif
