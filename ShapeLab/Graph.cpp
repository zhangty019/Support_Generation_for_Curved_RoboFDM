#include "Graph.h"

// Allocates memory for adjacency list
Graph::Graph(int V, QMeshPatch* mesh)
{
	this->V = V;
	adj = new std::list<Dijk_Pair>[V];
	this->inputMesh = mesh;
}

void Graph::addEdge(int u, double w, int v){

	adj[u].push_back(std::make_pair(w, v));
	//adj[v].push_back(std::make_pair(w, u)); 
}

// Prints shortest paths from src to all other vertices
void Graph::shortestPath(int src)
{
	// Create a priority queue to store vertices that
	// are being preprocessed. This is weird syntax in C++.
	// Refer below link for details of this syntax
	// https://www.geeksforgeeks.org/implement-min-heap-using-stl/
	std::priority_queue< Dijk_Pair, std::vector <Dijk_Pair>, std::greater<Dijk_Pair> > pq;

	// Create a vector for distances and initialize all
	// distances as infinite (INF)
	std::vector<double> dist(V, INF);

	// Insert source itself in priority queue and initialize
	// its distance as 0.
	pq.push(std::make_pair(0.0, src));
	dist[src] = 0.0;

	std::vector<bool> f(V, false);

	/* Looping till priority queue becomes empty (or all
	distances are not finalized) */
	while (!pq.empty())
	{
		// The first vertex in pair is the minimum distance
		// vertex, extract it from priority queue.
		// vertex label is stored in second of pair (it
		// has to be done this way to keep the vertices
		// sorted distance (distance must be first item
		// in pair)
		int u = pq.top().second;
		pq.pop();
		f[u] = true;

		// 'i' is used to get all adjacent vertices of a vertex
		std::list< std::pair<double, int> >::iterator i;
		for (i = adj[u].begin(); i != adj[u].end(); ++i)
		{
			// Get vertex label and weight of current adjacent
			// of u.

			double weight = (*i).first;
			int v = (*i).second;

			// If there is shorted path to v through u.
			if (f[v] == false && dist[v] > dist[u] + weight)
			{
				// Updating distance of v
				dist[v] = dist[u] + weight;
				pq.push(std::make_pair(dist[v], v));
			}
		}
	}

	// Print shortest distances stored in dist[]
	//std::printf("Vertex Distance from Source\n");
	//for (int i = 0; i < V; ++i)
	//	printf("%d \t\t %f\n", i, dist[i]);

	for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);

		Node->boundaryValue_temp = dist[Node->GetIndexNo()];
	}

}
