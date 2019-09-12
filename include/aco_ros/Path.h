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

#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "OccupancyGridMap.h"
using namespace std;

/**
* @class Path
* @brief A class that represents the path as a vector structure for the sequence of cells forming the path 
*/
class Path
{

public:
	/**
	* @brief default constructor to initialize path
	*/
	Path(void);
	/**
	* @brief Destructor.
	*/
	~Path(void);
	/**
	* @brief Constructor
	* @param pname
	*/
	Path(string pname);
	//Copy constructor for the map
	// Path (Path & path);

	/**
	* @brief Constructor
	* @param p
	*/
	Path(vector<int> p);
	/**
	* @brief Constructor
	* @param p
	* @param pname	
	*/
	Path(vector<int> p, string pname);
	//Define Mutators
	void setPath(vector<int> p);
	void setName(string pname);
	void setCost(float c);

	//Define Accessors
	vector<int> getPath();
	string getName();
	float getCost();

	/**
	* @brief it is used to insert cell
	* @param map
	* @param index
	* @param cell
	*/
	void insertCell(OccupancyGridMap *map, int index, int cell);
	/**
	* @brief it is used to remove cell
	* @param map
	* @param index
	*/
	void removeCell(OccupancyGridMap *map, int index);
	/**
	* @brief it is used to exchange 2 cells
	* @param map
	* @param cell	
	* @param index
	*/
	void setCell(OccupancyGridMap *map, int cell, int index);
	/**
	* @brief get the cost of the path
	* @param map
	* @param isRecalculate
	* @return path cost
	*/
	float getPathCost(OccupancyGridMap *map, bool isRecalculate);
	/**
	* @brief calculate distance between two cells in a path
	* @param OGM
	* @param startIndex
	* @param goalIndex
	* @return distance
	*/
	float calculateDistance(OccupancyGridMap *OGM, int startIndex, int goalIndex);
	/**
	* @brief checks wether the path is feasible
	* @param map
	* @return true if the path is feasible, false otherwise.
	*/
	bool isFeasible(OccupancyGridMap *map);
	/**
	* @brief checks wether the cost of the path is valid
	* @param map
	* @return ture if the path is valid, false otherwise.
	*/
	bool isValidCost(OccupancyGridMap *map);
	/**
	* @brief it is used to display a path
	*/
	void printPath();
	/**
	* @brief export path
	* @param map
	* @param file_name
	*/
	void exportPath(OccupancyGridMap *map, const char *file_name);

private:
	vector<int> path; //!< the path data structure
	float cost;		  //!<
	string name;	  //!<
};
#endif
