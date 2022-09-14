#pragma once
// Program to find Dijkstra's shortest path using
// priority_queue in STL
// #include<bits/stdc++.h>
#include <list>
#include <queue>
#include <iostream>
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

#define INF 1E10

// Dijk_Pair ==> Pair (double weight, int Index)
typedef std::pair<double, int> Dijk_Pair;

// This class represents a directed graph using
// adjacency list representation
class Graph
{
	int V; // No. of vertices

	// In a weighted graph, we need to store vertex
	// and weight pair for every edge
	std::list< std::pair<double, int> >* adj; // u (-> weight-> v )

	QMeshPatch* inputMesh;

public:
	Graph(int V, QMeshPatch* mesh); // Constructor

	// function to add an edge to graph
	void addEdge(int u,  double w, int v);

	// prints shortest path from s
	void shortestPath(int s);
};