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

#ifndef ANT_H
#define ANT_H

#include <vector>
#include "Path.h"

using namespace std;
/**
 * @class Ant
 * @brief A class that implements ----
 */
class Ant
{
public:
  /**
	* @brief Constructor.
	*/
  Ant();
  /**
	* @brief Constructor.
	* @param antPosition
	* @param path
	*/
  Ant(int antPosition, Path *path);
  /**
	* @brief Destructor.
	*/
  ~Ant();

  //void setID(int AntId);
  void setPosition(int AntPos);
  void setConstructedPath(Path *path);
  //int  getID();
  int getPosition();
  Path *getConstructedPath(); //return the path constructed by the ant

private:
  int Position;          //!< the position of the ant in the map represented by the cell ID
  Path *constructedPath; //!< the path is constructed by the ant
};

#endif
