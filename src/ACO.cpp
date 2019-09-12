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

#include "../include/aco_ros/ACO.h"
#include <vector>
#include <float.h>

using namespace std;
ofstream file100("traceFile.txt", ios::trunc);

/**********************************************************/
//Constructors and destructors
/**********************************************************/
ACO::ACO()
{
}
ACO::ACO(long double **PM)
{
	setPheromoneMatrix(PM);
}
ACO::ACO(long double initPheromoneValue, long double alpha, long double beta, long double evaporationRate, int Q, unsigned int numberOfAnts, int numberOfIterations, OccupancyGridMap *map) //:GenericACO(initPheromoneValue,alpha,beta,evaporationRate,Q,numberOfAnts, numberOfIterations)
{
	setNumberOfAnts(numberOfAnts);
	setInitialPheromoneValue(initPheromoneValue);
	setAlpha(alpha);
	setBeta(beta);
	setEvaporationRate(evaporationRate);

	setQ(Q);
	setNumberOfIterations(numberOfIterations);
}
ACO::~ACO()
{
}
/**********************************************************/
// Other functions(only required for removing dependency on GenericACO)
void ACO::setNumberOfAnts(unsigned int NumberOfAnts)
{
	numberOfAnts = NumberOfAnts;
}
unsigned int ACO::getNumberOfAnts()
{
	return numberOfAnts;
}
void ACO::setInitialPheromoneValue(long double InitialPheromoneValue)
{
	initialPheromoneValue = InitialPheromoneValue;
}
long double ACO::getInitialPheromoneValue()
{
	return initialPheromoneValue;
}
void ACO::setAlpha(long double Alpha)
{
	alpha = Alpha;
}
void ACO::setBeta(long double Beta)
{
	beta = Beta;
}
long double ACO::getAlpha()
{
	return alpha;
}
long double ACO::getBeta()
{
	return beta;
}
void ACO::setEvaporationRate(long double EvaporationRate)
{
	evaporationRate = EvaporationRate;
}
long double ACO::getEvaporationRate()
{
	return evaporationRate;
}
void ACO::setQ(int q)
{
	Q = q;
}
int ACO::getQ()
{
	return Q;
}
void ACO::setNumberOfIterations(int numIterations)
{
	numberOfIterations = numIterations;
}
int ACO::getNumberOfIterations()
{
	return numberOfIterations;
}
//Function Name: findBestPath
//Inputs: the map, a vector of paths
//Output: index of the best path in the vector
//Description: it is to generate the best path from a set of candidate paths
/*********************************************************************************/
int ACO::findBestPath(OccupancyGridMap *map, vector<Path *> paths)
{
	int indexBestPath = 0;
	float minPathCost = paths[0]->getPathCost(map, false);
	for (uint i = 0; i < paths.size(); i++)
	{
		if (paths[i]->getPathCost(map, false) <= minPathCost)
		{
			minPathCost = paths[i]->getPathCost(map, false);
			indexBestPath = i;
		}
	}
	return indexBestPath;
}
/*******************************************************************************/
//Function Name: findBestAnt
//Inputs: the map, a vector of Ant
//Output: index of the best Ant in the vector
//Description: it is to generate the best ant from a set of candidate ants
/*********************************************************************************/
Path *ACO::findBestAnt(OccupancyGridMap *map, vector<Ant *> Ants)
{
	Path *bestPathOfTheAnt = new Path(); //best path of the ant
	int indexBestAnt = 0;				 // index of the best ant in the vector Ants
	float minPathCost = 0;				 //minimum path cost

	//initialize the minimum path cost
	for (uint i = 0; i < Ants.size(); i++)
	{
		if (Ants[i]->getConstructedPath()->getPathCost(map, false) != 0)
		{
			minPathCost = Ants[i]->getConstructedPath()->getPathCost(map, false);
			break;
		}
	}
	//identify the minimum path cost in the vector ants
	if (minPathCost != 0)
	{
		for (uint i = 0; i < Ants.size(); i++)
		{
			if (Ants[i]->getConstructedPath()->getPathCost(map, false) <= minPathCost && Ants[i]->getConstructedPath()->getPathCost(map, false) != 0)
			{
				minPathCost = Ants[i]->getConstructedPath()->getPathCost(map, false);
				indexBestAnt = i;
			}
		}
		bestPathOfTheAnt = Ants[indexBestAnt]->getConstructedPath();
	}

	return bestPathOfTheAnt;
}
/*******************************************************************************
	* Function Name: findFreeNeighborCell
	* Inputs: the row and columun of the current Cell
	* Output: a vector of free neighbor cells of the current cell
	* Description:it is used to find the free neighbors Cells of a the current Cell in the grid
	* Check Status: Checked by Anis, Imen and Sahar
	*********************************************************************************/
vector<int> ACO::findFreeNeighborCell(OccupancyGridMap *map, int CellID)
{

	int rowID = map->getCellRowID(CellID);
	int colID = map->getCellColID(CellID);

	vector<int> freeNeighborCells;

	for (int i = -1; i <= 1; i++)
		for (int j = -1; j <= 1; j++)
		{
			//check whether the index is valid
			if ((rowID + i >= 0) && (rowID + i < map->getHeight()) && (colID + j >= 0) && (colID + j < map->getWidth()))
				if (map->isFree(rowID + i, colID + j) && (!(i == 0 && j == 0)))
				{
					freeNeighborCells.push_back(map->getCellIndex(rowID + i, colID + j));
				}
		}
	return freeNeighborCells;
}
/*******************************************************************************
	* Function Name: getUnvisitedFreeNeighbors
	* Inputs: the neighbor cells and the path
	* Output: a vector of unvisited neighbor
	* Description:it is to remove the already visited neighbor from the path
	* Check Status: Checked by Anis, Imen and Sahar
	*********************************************************************************/
vector<int> ACO::getUnvisitedFreeNeighbors(vector<int> neighborCells, Path *path)
{
	vector<int> unvisitedNeighborCells;
	for (uint i = 0; i < neighborCells.size(); i++)
		if (!isVisited(neighborCells[i], path))
			unvisitedNeighborCells.push_back(neighborCells[i]);

	return unvisitedNeighborCells;
}
/*******************************************************************************
	* Function Name: isVisited
	* Inputs: the cellID and the path
	* Output: a boolean variable that indicates if the cellID exists in the path or not
	* Description:it is to verify if a cell exists in the path or not
	* Check Status: Checked by Anis, Imen and Sahar
	*********************************************************************************/
bool ACO::isVisited(int CellID, Path *path)
{
	for (uint i = 0; i < path->getPath().size(); i++)
	{
		if (CellID == path->getPath()[i])
			return true;
	}
	return false;
}
/*******************************************************************************/
//Function Name: isStartAndGoalCellsValid
//Inputs: the start and Goal cells
//Output: true if the start and the goal cells are valid
//Description: check if the start and goal cells are valid
/*********************************************************************************/
bool ACO::isStartAndGoalCellsValid(OccupancyGridMap *map, int startCell, int goalCell)
{
	bool isvalid = true;
	bool isFreeStartCell = map->isFree(map->getCellRowID(startCell), map->getCellColID(startCell));
	bool isFreeGoalCell = map->isFree(map->getCellRowID(goalCell), map->getCellColID(goalCell));
	if (startCell == goalCell)
	{
		cout << "The Start and the Goal cells are the same..." << endl;
		isvalid = false;
	}
	else
	{
		if (!isFreeStartCell && !isFreeGoalCell)
		{
			cout << "The start and the goal cells are obstacle positions..." << endl;
			isvalid = false;
		}
		else
		{
			if (!isFreeStartCell)
			{
				cout << "The start is an obstacle..." << endl;
				isvalid = false;
			}
			else
			{
				if (!isFreeGoalCell)
				{
					cout << "The goal cell is an obstacle..." << endl;
					isvalid = false;
				}
				else
				{
					if (findFreeNeighborCell(map, goalCell).size() == 0)
					{
						cout << "The goal cell is encountred by obstacles... " << endl;
						isvalid = false;
					}
					else
					{
						if (findFreeNeighborCell(map, startCell).size() == 0)
						{
							cout << "The start cell is encountred by obstacles... " << endl;
							isvalid = false;
						}
					}
				}
			}
		}
	}
	return isvalid;
}

/**********************************************************/
//Function: Mutators
/**********************************************************/
void ACO::setPheromoneMatrix(long double **PM)
{
	pheromoneMatrix = PM;
}
/**********************************************************/
//Function: Accessors
/**********************************************************/
long double **ACO::getPheromoneMatrix()
{
	return pheromoneMatrix;
}
/**********************************************************/
// Function initializePheromoneMatrix
// it used to initialze the Matrix of pheromone
// Input: the occupancy grid map
/**********************************************************/
void ACO::initializePheromoneMatrix(OccupancyGridMap *map)
{
	pheromoneMatrix = (long double **)malloc(sizeof(long double *) * map->getHeight());
	for (int i = 0; i < map->getWidth(); i++)
		pheromoneMatrix[i] = (long double *)malloc(sizeof(long double) * map->getWidth());

	for (int i = 0; i < map->getWidth(); i++)
	{
		for (int j = 0; j < map->getHeight(); j++)
		{
			if (map->isFree(i, j))
			{
				getPheromoneMatrix()[i][j] = getInitialPheromoneValue();
			}
			else
				getPheromoneMatrix()[i][j] = FLT_MAX;
		}
	}
}
/**********************************************************/
// Function computeTransitionProbabilities
// it calculates the transition probabilities of the ants between the cells
// Input: the map, the current Cell and the Unvisited Neighbor Cells of the current cell.
// Output: the probabilities calculated by the transition rule probability.
/**********************************************************/
vector<long double> ACO::computeTransitionProbabilities(OccupancyGridMap *map, int currentCell, vector<int> unvisitedNeighborCells)
{
	vector<long double> probabilities; //it contains the probabilities of transition
	long double etha;				   // the inverse of euclidian distance between the current cell and the neighbor cell

	//calculate the probabilities using the transition rule probability

	for (uint i = 0; i < unvisitedNeighborCells.size(); i++)
	{
		etha = 1 / map->getMoveCost(currentCell, unvisitedNeighborCells[i]);
		probabilities.push_back(pow(getPheromoneMatrix()[map->getCellRowID(unvisitedNeighborCells[i])][map->getCellColID(unvisitedNeighborCells[i])], getAlpha()) * pow(etha, getBeta()));
	}

	long double sum = 0;
	for (uint i = 0; i < probabilities.size(); i++)
	{
		sum = sum + probabilities[i];
	}
	if (sum != 0)
		for (uint i = 0; i < unvisitedNeighborCells.size(); i++)
		{
			probabilities[i] = probabilities[i] / sum;
		}

	return probabilities;
}
/**********************************************************/
// Function getNextCell
// it is used to select the next cell of the ant.
// input: a vector of neighbor cells and their probabilities.
// output: the next cell of the ant.
//Check Status: Checked by Imen
/**********************************************************/
int ACO::getNextCell(vector<int> NeighborCells, vector<long double> probabilities)
{
	int nextCell;
	vector<long double> cumulativeProbabilities;
	float randomNumber;

	//calculate the cumulative probabilities
	cumulativeProbabilities.push_back(probabilities[0]);
	for (uint i = 1; i < probabilities.size(); i++)
	{
		cumulativeProbabilities.push_back(cumulativeProbabilities[i - 1] + probabilities[i]);
	}

	file100 << "Cumulative Probabilities: ";
	for (uint i = 0; i < cumulativeProbabilities.size(); i++)
		file100 << cumulativeProbabilities[i] << "\t";
	file100 << endl;

	//choose a random number
	randomNumber = 1 - ((float)rand() / (float)RAND_MAX);

	file100 << "Random number= " << randomNumber << endl;

	// choose the next cell
	//case1:
	if (randomNumber >= 0 && randomNumber <= cumulativeProbabilities[0])
	{
		nextCell = NeighborCells[0];
	}
	else
	{
		//case2:
		if (randomNumber >= cumulativeProbabilities[cumulativeProbabilities.size() - 1] && randomNumber <= 1)
		{
			nextCell = NeighborCells[NeighborCells.size() - 1];
		}
		//case3:
		else
		{
			for (int i = 0; i < cumulativeProbabilities.size(); i++)
			{
				if (randomNumber >= cumulativeProbabilities[i] && randomNumber <= cumulativeProbabilities[i + 1])
				{
					long double difference1 = randomNumber - cumulativeProbabilities[i];
					long double difference2 = cumulativeProbabilities[i + 1] - randomNumber;
					if (difference1 < difference2)
						nextCell = NeighborCells[i];
					else
						nextCell = NeighborCells[i + 1];
				}
			}
		}
	}
	return nextCell;
}
/**********************************************************/
// Function updatePheromone
// it is used to update the quantity of pheromone.
// input: vector of Ant that include the ants id, their paths and their paths cost.
// output:
/**********************************************************/
void ACO::updatePheromone(OccupancyGridMap *map, vector<Ant *> ants)
{
	long double delta; // the quantity of pheromone laid on edge (cell1, cell2) joining two cells i and j by all ants
	if (ants.size() != 0)
	{
		for (uint i = 0; i < map->getHeight(); i++)
		{
			for (uint j = 0; j < map->getWidth(); j++)
			{
				if (getPheromoneMatrix()[i][j] != FLT_MAX)
				{
					delta = getDelta(ants, map->getCellIndex(i, j));
					getPheromoneMatrix()[i][j] = getPheromoneMatrix()[i][j] * (1 - getEvaporationRate()) + delta;
				}
			}
		}
	}
}
/**********************************************************/
// Function getDelta
// iIt is used to calculate the delta value used in the update pheromone rule.
// Input: list of ants the index of the cells i and j.
// Output: the delta value.
//Check Status: Checked by Imen
/**********************************************************/
long double ACO::getDelta(vector<Ant *> Ants, int cell)
{
	long double delta = 0;
	long double Lk = 0; // cost of the constructed path by all the ants .
	bool exist = false;
	for (uint k = 0; k < Ants.size(); k++)
	{
		if (Ants[k]->getConstructedPath()->getCost() != 0)
		{
			for (uint l = 0; l < Ants[k]->getConstructedPath()->getPath().size(); l++)
			{
				// if the edge relying two cells cell1 and cell2
				if (Ants[k]->getConstructedPath()->getPath()[l] == cell)
				{
					// if the ant k uses the position cell
					// the length of the path constructed by the ant k is added to Lk value.
					Lk = Lk + Ants[k]->getConstructedPath()->getCost();
					exist = true;
					break;
				}
			}
		}
	}
	if (exist == true)
	{
		// if the edge (cell1,cell2) is used by the ants then the value of delta is different from 0
		//thus the quantity of pheromone will increase else it will evaporate
		delta = getQ() / Lk;
	}

	return delta;
}
/*******************************************************************************/
//Function Name: findPathAco
//Inputs: the map, the start cell and the goal cell
//Output: the best path of the algorithm
//Description: it is used to generate the best path after N iterations
//Check Status: Checked by Imen
/*********************************************************************************/
Path *ACO::findPathAco(OccupancyGridMap *map, int startCell, int goalCell, bool enable_trace)
{
	if (enable_trace == true)
	{
		//    file100 << "Planner Type -ACO-" << endl;
		//    file100 << "/....Map dimensions" << endl;
		//    file100 << "Width= " << map->getWidth() << " " << "Height= " << map->getHeight() <<endl;
		//    file100 << "/....Start and goal positions" << endl;
		//    file100 << "The start position is: " <<startCell << " and the goal position is: " << goalCell << endl;
		//    file100 << "/....ACO parameters" << endl;
		//    file100 << "Alpha= " << getAlpha() << " " << "beta= " << getBeta() << " " << "Evaporation rate= " << getEvaporationRate() << " " << "Q= " << getQ() << " " << "Number of ants= " << getNumberOfAnts() << " " << "Initial pheromone value= " << getInitialPheromoneValue() << endl;
		//    file100 << "/....starting simulation" << endl;
	}
	Path *bestPath = new Path();		   //the best path
	vector<Path *> bestPaths;			   // it contains the best paths generated after each iteration
	vector<int> neighborCell;			   //the vector of neighbor cells
	vector<int> unvisitedFreeNeighborCell; //the unvisited neighbor cells
	int currentCell;					   //the current  position of the ant
	int nextCell;						   //the new position  of the ant
	vector<long double> probabilities;	 // it contains the probabilities of the possible next positions of an ant
	vector<Ant *> Ants(getNumberOfAnts()); // vector of ant
	int indexBestPath;					   // index of the best path of the algorithm

	//create the ants
	for (uint j = 0; j < getNumberOfAnts(); j++)
		Ants[j] = new Ant();

	//intialize the pheromone matrix
	initializePheromoneMatrix(map);
	if (enable_trace == true)
	{
		file100 << "The initial pheromone Matrix: " << endl;
		for (int row = 0; row < map->getHeight(); ++row)
		{
			for (int col = 0; col < map->getWidth(); ++col)
			{
				file100 << getPheromoneMatrix()[row][col] << " ";
			}
			file100 << endl;
		}
	}

	for (int i = 0; i < getNumberOfIterations(); i++) //ACO is repeated "numberOfIterations" times
	{
		if (enable_trace == true)
		{
			file100 << "**********************************" << endl;
			file100 << "iteration " << i << endl;
			file100 << "**********************************" << endl;
		}
		for (uint j = 0; j < getNumberOfAnts(); j++) // for each Ant
		{
			currentCell = startCell;
			Path *pathAnt = new Path();
			Ants[j]->setPosition(currentCell);
			pathAnt->insertCell(map, 0, currentCell);
			Ants[j]->setConstructedPath(pathAnt);
			int index = 1;
			//while the ant has not reached the goal position or not blocked
			while (currentCell != goalCell)
			{
				neighborCell = findFreeNeighborCell(map, currentCell);
				unvisitedFreeNeighborCell = getUnvisitedFreeNeighbors(neighborCell, pathAnt);
				if (enable_trace == true)
				{
					file100 << "The current Cell " << currentCell << endl;
					file100 << "The unvisited neighbor cells: ";
					for (uint i = 0; i < unvisitedFreeNeighborCell.size(); i++)
						file100 << unvisitedFreeNeighborCell[i] << "\t";
					file100 << endl;
				}

				if (unvisitedFreeNeighborCell.size() == 0) // if the ant is blocked and there is no unvisited neighbor cells so the ant will be discarted
				{
					pathAnt->getPath().clear();
					Ants[j]->setConstructedPath(pathAnt);
					Ants[j]->getConstructedPath()->setCost(0);
					if (enable_trace == true)
					{
						file100 << "The ant [ " << j << "] is blocked" << endl;
						file100 << "**********************************************************************************" << endl;
					}
					break;
				}
				else
				{
					probabilities = computeTransitionProbabilities(map, currentCell, unvisitedFreeNeighborCell);
					if (enable_trace == true)
					{
						file100 << "Probabilities: ";
						for (uint i = 0; i < probabilities.size(); i++)
							file100 << probabilities[i] << "\t";
						file100 << endl;
					}
					nextCell = getNextCell(unvisitedFreeNeighborCell, probabilities);
					if (enable_trace == true)
					{
						file100 << "The next cell is: " << nextCell << endl;
						file100 << "------------------------" << endl;
					}
					currentCell = nextCell;
					pathAnt->insertCell(map, index, nextCell);
					index++;

					if (currentCell == goalCell) // if the ant has reached the goal cell
					{
						if (enable_trace == true)
							file100 << "The ant reachs the goal position" << endl;
						Ants[j]->setConstructedPath(pathAnt);
						Ants[j]->getConstructedPath()->setCost(pathAnt->getPathCost(map, false));
						if (enable_trace == true)
						{
							file100 << "The path of the ant [" << j << "] ";
							for (uint i = 0; i < Ants[j]->getConstructedPath()->getPath().size(); i++)
								file100 << Ants[j]->getConstructedPath()->getPath()[i] << "\t";
							file100 << endl;
							//  Ants[j]->getConstructedPath()->printPath();
							file100 << "The path cost is " << Ants[j]->getConstructedPath()->getCost() << endl;
							file100 << "*******************************************************************************" << endl;
						}
					}
				}
			}
			pathAnt->getPath().clear();
		}

		// The best path after one iteration
		Path *bestPathInAnIteration = new Path();
		bestPathInAnIteration = findBestAnt(map, Ants);

		if (bestPathInAnIteration->getCost() != 0)
		{
			cout << "The best path after iteration " << i << endl;
			bestPathInAnIteration->printPath();
			cout << "Its cost is " << bestPathInAnIteration->getPathCost(map, false) << endl;
			bestPaths.push_back(bestPathInAnIteration);
		}
		// update the pheromone list
		updatePheromone(map, Ants);
		if (enable_trace == true)
		{
			file100 << "The updated pheromone Matrix is: " << endl;
			for (int row = 0; row < map->getHeight(); ++row)
			{
				for (int col = 0; col < map->getWidth(); ++col)
				{
					file100 << getPheromoneMatrix()[row][col] << " ";
				}
				file100 << endl;
			}
		}
	}

	// The best path generated by the algorithm
	if (bestPaths.size() != 0)
	{
		indexBestPath = findBestPath(map, bestPaths);
		bestPath->setPath(bestPaths[indexBestPath]->getPath());
		bestPath->setCost(bestPaths[indexBestPath]->getPathCost(map, false));
	}

	return bestPath;
}
