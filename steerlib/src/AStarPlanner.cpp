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
#define HEURISTIC 2

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
		for (int i = 0; i < openset.size(); i++) {
			if (openset[i].f < lowestf) {
				lowestf = openset[i].f;
				pos = i;
			}
		}

		printf("Lowest F found (%f) at %d\n", openset[pos].f, pos);

		return pos;
	}

	SteerLib::AStarPlannerNode AStarPlanner::createNeighbor(Util::Point point, Util::Point goal, SteerLib::AStarPlannerNode curr, bool dpad)
	{
		float g, f, h;


		// g = DPAD/DIAGONAL + curr.g
		if (dpad) {
			printf("DPAD curr g: %f\n", curr.g);
			g = 1 + curr.g;
		}
		else {
			printf("DIAGONAL curr g: %f\n", curr.g);
			g = 1 + curr.g;

			if (PART_3)
			{
				g = DJDIAG + curr.g;
			}
		}

		// Heuristic is point to goal
		//If part 4, increase the heuristic cost by 2, 4, or 8 respectively 
		if (PART_4)
		{
			//Change H_1 to either: H_2 or H_3 for inflated` heuristic
			h = H_1 * Heuristic(point, goal);
		}

		//If not part 4, heurstic as normal
		h = Heuristic(point, goal);

		f = g + h;

		//printf("MAKING <%f, %f, %f> the parent\n", curr.point.x, curr.point.y, curr.point.z);

		SteerLib::AStarPlannerNode temp(point, g, h, f, &curr);

		//printf("Making node <%f, %f, %f>\nG: %f, ")

		//printf("temp parent <%f, %f, %f>\n", temp.parent->point.x, temp.parent->point.y, temp.parent->point.z);

		return temp;
		//return SteerLib::AStarPlannerNode (point, g, f, &curr);
	}

	std::vector<SteerLib::AStarPlannerNode> AStarPlanner::getNeighbors(std::vector<SteerLib::AStarPlannerNode> neighbors, SteerLib::AStarPlannerNode curr, Util::Point start, Util::Point goal)
	{
		int i = curr.point.x;
		int j = curr.point.y;

		std::vector<Util::Point> points;

		points.push_back(Util::Point(i - 1, 0, j));			// top			0
		points.push_back(Util::Point(i + 1, 0, j));			// bottom		1
		points.push_back(Util::Point(i, 0, j - 1));			// left			2
		points.push_back(Util::Point(i, 0, j + 1));			// right		3
		points.push_back(Util::Point(i - 1, 0, j - 1));		// top left		4
		points.push_back(Util::Point(i - 1, 0, j + 1));		// top right	5
		points.push_back(Util::Point(i + 1, 0, j - 1));		// bottom left	6
		points.push_back(Util::Point(i + 1, 0, j + 1));		// bottom right	7

		if (canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points[i])))
		{
			for (int i = 0; i < points.size(); i++) {
				//if (!canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points[i])))
					//continue;

				if (i < 4)
					neighbors.push_back(createNeighbor(points[i], goal, curr, true));
				else
					neighbors.push_back(createNeighbor(points[i], goal, curr, false));
			}
		}

		return neighbors;
	}

	

	std::vector<Util::Point> AStarPlanner::pathReconstruct(std::map<int, int>& cameFrom, int currID, Util::Point start)
	{
		std::vector<Util::Point> totalPath;
		int bruh = currID;
		int startIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		while (bruh != startIndex)
		{
			totalPath.insert(totalPath.begin(), getPointFromGridIndex(bruh));
			bruh = cameFrom[bruh];
		}

		//Because it breaks the loop and doesn't add the last point
		//So it adds that last point here
		totalPath.insert(totalPath.begin(), getPointFromGridIndex(bruh));
		return totalPath;
	}
	



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		//std::cout << "\nIn A*";

		std::vector<SteerLib::AStarPlannerNode> openset, closedset, neighbors;
		std::map<int, int> cameFrom;

		// AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
		// g = move cost --> parent g + diagonal/dpad
		// h = distance to goal from node
		// f = g + h
		double start_g = 0;
		double start_h = Heuristic(start, goal);
		double start_f = start_g + start_h;
		SteerLib::AStarPlannerNode startnode(start, start_g, start_h, start_f, nullptr);
		

		printf("\n************\nGoal Node: <%f, %f, %f>\n", goal.x, goal.y, goal.z);
		printf("Start Node: <%f, %f, %f>\n", startnode.point.x, startnode.point.y, startnode.point.z);
		printf("Start G-value: %f\n", startnode.g);
		printf("Start H-value: %f\n", startnode.h);
		printf("Start F-value: %f\n************\n", startnode.f);

		openset.push_back(startnode);
		printf("\n************\n");
		printf("Adding startnode to openset\n");
		printf("************\n");

		int pos = 0;

		while (!openset.empty()) {
			pos = locateSmallestFPos(openset);

			SteerLib::AStarPlannerNode curr = openset[pos];
			printf("\n************\n");
			printf("Set curr to <%f, %f, %f>\n", openset[pos].point.x, openset[pos].point.y, openset[pos].point.z);
			printf("************\n");

			if (curr.point == goal) {
				//TODO
				//RECONSTRUCT PATH HERE

				int currID = gSpatialDatabase->getCellIndexFromLocation(curr.point);
				agent_path = pathReconstruct(cameFrom, currID, startnode.point);
				printf("\n************\n");
				printf("GOOOAL\n");
				printf("************\n");
				return true;
			}
			else {
				printf("\n************\n");
				printf("Not goal, remove curr from openset, find neighbors\n");
				printf("************\n");
			}

			// remove curr from openset
			printf("\n************\n");
			printf("Removing curr from openset\n");
			openset.erase(openset.begin() + pos);
			printf("************\n");

			// add curr to closedset
			printf("\n************\n");
			printf("Adding curr to closedset\n");
			closedset.push_back(curr);
			printf("************\n");

			neighbors = getNeighbors(neighbors, curr, start, goal);
			printf("\n************\n");
			printf("Neighbors:\n");
			for (int i = 0; i < neighbors.size(); i++) {
				printf("<%f, %f, %f>\n", neighbors[i].point.x, neighbors[i].point.y, neighbors[i].point.z);
				printf("G: %f\nH: %f\nF: %f\n", neighbors[i].g, neighbors[i].h, neighbors[i].f);

				// check if neighbor in closedset
				printf("Checking if neighbor's in closed set\n");
				if (std::find(closedset.begin(), closedset.end(), neighbors[i]) != closedset.end()) {
					printf("Neighbor is in closed set\n");
					continue;
				}

				//double tentativeg = curr.g + distanceBetween(curr.point, neighbors[i].point);

				/*
				if (tentativeg < neighbors[i].g)
				{
					printf("Curr in came from\n");
					cameFrom[i] = gSpatialDatabase->getCellIndexFromLocation(curr.point);
					//camefrom.push_back(curr);
					printf("tentative g in neighbors g\n");
					neighbors[i].g = tentativeg;
					printf("new f value in neighbor\n");
					neighbors[i].f = neighbors[i].g + Heuristic(neighbors[i].point, goal);

					if (!(std::find(openset.begin(), openset.end(), neighbors[i]) != openset.end())) {
						printf("Not in openset...new node here\n");
						printf("Adding neighbor to openset\n");
						openset.push_back(neighbors[i]);
					}
				}
				
				*/
				// might have to use heuristic for distanceBetween
				double tentativeg = curr.g + distanceBetween(curr.point, neighbors[i].point);

				printf("Checking if neighbor's in open set, if it is then check tentativeg\n");
				if (!(std::find(openset.begin(), openset.end(), neighbors[i]) != openset.end())) {
					printf("Not in openset...new node here\n");
					printf("Adding neighbor to openset\n");
					openset.push_back(neighbors[i]);
				}
				else if (tentativeg >= neighbors[i].g) {
					printf("Path is not worth traveling\n");
					continue;
				}

				cameFrom[i] = gSpatialDatabase->getCellIndexFromLocation(curr.point);
				neighbors[i].g = tentativeg;
				neighbors[i].f = neighbors[i].g + Heuristic(neighbors[i].point, goal);

				
			}
			printf("************\n");

			//break;
		}





		return false;
	}
}
