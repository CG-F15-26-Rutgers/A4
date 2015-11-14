//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

// 1 for Manhattan
// 2 for Euclidean
#define HEURISTIC 1

//Set to true for part 3
#define PART_3 false
#define DJDIAG 2


//Set to true for part 4
#define PART_4 false
#define H_1 2
#define H_2 4
#define H_3 8




namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	double AStarPlanner::Manhattan(Util::Point a, Util::Point b)
	{
		// don't need y since it isn't jumping
		double x = abs(a.x - b.x);
		double z = abs(a.z - b.z);

		return x + z;
	}

	double AStarPlanner::Euclidean(Util::Point a, Util::Point b)
	{
		Util::Vector mathStuff = b - a;

		return sqrt(pow(mathStuff.x, 2) + pow(mathStuff.z, 2));
	}

	double AStarPlanner::Heuristic(Util::Point start, Util::Point goal)
	{
		// Manhattan
		if (HEURISTIC == 1) {
			return Manhattan(start, goal);
		}

		// Euclidean
		return Euclidean(start, goal);
	}

	int AStarPlanner::locateSmallestFPos(std::vector<SteerLib::AStarPlannerNode> openset)
	{
		int pos = 0;
		double lowestf = openset.begin()->f;
		double lowestg = openset.begin()->g; //Used to break ties.
		for (int i = 0; i < openset.size(); i++) {
			if (openset[i].f < lowestf) {
				lowestf = openset[i].f;
				lowestg = openset[i].g;
				pos = i;
			}
			else if (openset[i].f == lowestf && openset[i].g < lowestg){
				lowestg = openset[i].g;
				pos = i;
			}
		}
		
		return pos;
	}

	SteerLib::AStarPlannerNode AStarPlanner::createNeighbor(Util::Point point, Util::Point goal, SteerLib::AStarPlannerNode curr, bool dpad)
	{
		float g, f, h;

		if (dpad)
			g = 1 + curr.g;
		else {
			if (HEURISTIC == 1)
				g = 2 + curr.g; // Diagonal movement has length 2 in Manhattan geometry.
			else
				g = sqrt(2) + curr.g;

			// if (PART_3)
			// 	g = DJDIAG + curr.g;
		}

		// Heuristic is point to goal
		//If part 4, increase the heuristic cost by 2, 4, or 8 respectively 
		//if (PART_4)
		//h = H_1 * Heuristic(point, goal); // Change H_1 to either: H_2 or H_3 for inflated` heuristic

		//If not part 4, heurstic as normal
		//else
		h = Heuristic(point, goal);

		f = g + h;
		
		return SteerLib::AStarPlannerNode(point, g, h, f, &curr);
	}

	
	std::vector<SteerLib::AStarPlannerNode> AStarPlanner::getNeighbors(SteerLib::AStarPlannerNode curr, Util::Point start, Util::Point goal){
		
		std::vector<SteerLib::AStarPlannerNode> neighbors;
		std::vector<Util::Point> points;
		int i = curr.point.x;
		int j = curr.point.z;

		neighbors.clear(); // Ensure it does not contain garbage data.

		points.push_back(Util::Point(i - 1, 0, j));			// top			0
		points.push_back(Util::Point(i + 1, 0, j));			// bottom		1
		points.push_back(Util::Point(i, 0, j - 1));			// left			2
		points.push_back(Util::Point(i, 0, j + 1));			// right		3
		points.push_back(Util::Point(i - 1, 0, j - 1));		// top left		4
		points.push_back(Util::Point(i - 1, 0, j + 1));		// top right	5
		points.push_back(Util::Point(i + 1, 0, j - 1));		// bottom left	6
		points.push_back(Util::Point(i + 1, 0, j + 1));		// bottom right	7
		
		for (int i = 0; i < points.size(); i++) {
			if (canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points[i]))){
				if (i < 4)
					neighbors.push_back(createNeighbor(points[i], goal, curr, true));
				else
					neighbors.push_back(createNeighbor(points[i], goal, curr, false));
			}
		}
		
		return neighbors;
	}

	
	std::vector<Util::Point> AStarPlanner::pathReconstruct(std::vector<Util::Point>& cameFrom, int currID, Util::Point start){	
		std::vector<Util::Point> totalPath;
		totalPath.clear(); // Ensure it doesn't contain any garbage data.
		
		for (int i = 0; i < cameFrom.size(); ++i)
			totalPath.push_back(cameFrom[i]);
		
		return totalPath;
	}
	

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		std::vector<SteerLib::AStarPlannerNode> openNodes, closedNodes, neighbors;
		std::vector<Util::Point> cameFrom;
		int pos;
		
		double start_g = 0;
		double start_h = Heuristic(start, goal);
		double start_f = start_g + start_h;
		SteerLib::AStarPlannerNode startnode(start, start_g, start_h, start_f, nullptr);

		openNodes.push_back(startnode);

		while (!openNodes.empty()) {
			pos = locateSmallestFPos(openNodes);

			SteerLib::AStarPlannerNode curr = openNodes[pos];

			if (curr.point == goal) {
				int currID = gSpatialDatabase->getCellIndexFromLocation(curr.point);
				agent_path = pathReconstruct(cameFrom, currID, startnode.point);
				return true;
			}
			
			openNodes.erase(openNodes.begin() + pos); // remove curr from openNodes
			closedNodes.push_back(curr); // add curr to closedNodes

			neighbors = getNeighbors(curr, start, goal);
			
			for (int i = 0; i < neighbors.size(); i++) {
				// check if neighbor in closedNodes
				if (std::find(closedNodes.begin(), closedNodes.end(), neighbors[i]) != closedNodes.end())
					continue;
				
				// might have to use heuristic for distanceBetween
				double tentativeg = curr.g + distanceBetween(curr.point, neighbors[i].point);

				if (tentativeg < neighbors[i].g){
					cameFrom.push_back(curr.point); // NOTE: this is where the problem is.
					neighbors[i].g = tentativeg;
					neighbors[i].f = neighbors[i].g + Heuristic(neighbors[i].point, goal);
					
					if (std::find(openNodes.begin(), openNodes.end(), neighbors[i]) == openNodes.end())
						openNodes.push_back(neighbors[i]);
				}
			}
		}
		
		return false;
	}
}
