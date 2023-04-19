#ifndef TOOLPATHGENERATION_H
#define TOOLPATHGENERATION_H

#include "QMeshPatch.h"
#include "PolygenMesh.h"
#include "QMeshNode.h"
#include "QMeshEdge.h"
#include "QMeshFace.h"
#include "../GLKLib/GLKObList.h"

class toolpathGeneration
{
public:
	toolpathGeneration(PolygenMesh* isoLayerSet, PolygenMesh* toolPathSet,
		double deltaWidth, double deltaDistance);
	~toolpathGeneration();

	
	void generate_all_toolPath();
	

private:

	QMeshPatch* generate_each_bundaryToolPath(QMeshPatch* surfaceMesh);
	void resampleToolpath(QMeshPatch* patch);
	int autoComputeTPathNum(QMeshPatch* surfaceMesh, bool boundaryORzigzag);
	void generateBoundaryIsoNode(QMeshPatch* singlePath, QMeshPatch* surfaceMesh, double isoValue);
	void linkEachIsoNode(QMeshPatch* singlePath, double startIsoValue);
	QMeshNode* findNextBoundaryToolPath(QMeshNode* sNode, QMeshPatch* singlePath);
	QMeshEdge* buildNewEdgetoQMeshPatch(QMeshPatch* patch, QMeshNode* startNode, QMeshNode* endNode);
	QMeshNode* findNextNearestPoint(QMeshNode* startNode, QMeshPatch* boundIsoPatch);
	bool detectAll_isoNode_Processed(QMeshPatch* singlePath);
	QMeshNode* link_OneRing_isoNode(QMeshPatch* singlePath, QMeshNode* sNode, QMeshNode* eNode);


	PolygenMesh* m_isoLayerSet;
	PolygenMesh* m_toolPathSet;
	double toolpath_Width;
	double toolpath_Distance;
};

#endif // TOOLPATHGENERATION_H