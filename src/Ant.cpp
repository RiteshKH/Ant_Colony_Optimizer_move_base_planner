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

#include "../include/aco_ros/Ant.h"

#include <vector>

using namespace std;

Ant::Ant()
{
}

Ant::Ant(int antPosition, Path *path)
{
	Position = antPosition;
	constructedPath = path;
}

Ant::~Ant()
{
}
/**********************************************************/
//Function: Mutators
/**********************************************************/
void Ant::setPosition(int antPosition)
{
	Position = antPosition;
}

void Ant::setConstructedPath(Path *path)
{
	constructedPath = path;
}

/**********************************************************/
//Function: Accessors
/**********************************************************/
int Ant::getPosition()
{
	return Position;
}
Path *Ant::getConstructedPath()
{
	return constructedPath;
}
