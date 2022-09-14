#pragma once
class QMeshPatch;

struct Graph_Node {
	QMeshNode* attached_QNode = NULL;
	bool isVisited = false;
	double dist = 1.0e5;
	QMeshNode* parent_QNode = NULL;
	double multiSrc_dist = 9999.9;
};

class BoundaryFieldCalculation
{
public:
	BoundaryFieldCalculation(void) {};
	~BoundaryFieldCalculation(void) {};

	double GenerateBndDistMap(QMeshPatch* mesh);	//basic
	double GenerateBndDistMap1(QMeshPatch* mesh);	//priority queue opt
	double GenerateBndDistMap2(QMeshPatch* mesh);	//multi-channel

	void meshRefinement(QMeshPatch* surfaceMesh);

private:
	
	bool _visitedAll(std::vector<Graph_Node> graph);
	double _calQMeshNodeDist(QMeshNode* node1, QMeshNode* node2);

};