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
	toolpathGeneration(QMeshPatch* isoLayer, double deltaWidth, double deltaDistance);
	~toolpathGeneration();

	QMeshPatch* generateBundaryToolPath();
	void resampleToolpath(QMeshPatch* patch);

	double toolpath_Width;
	double toolpath_Distance;

private:
	int autoComputeTPathNum();
	void generateBoundaryIsoNode(QMeshPatch* singlePath, double isoValue);
	void linkEachIsoNode(QMeshPatch* singlePath, double startIsoValue);
	QMeshNode* findNextBoundaryToolPath(QMeshNode* sNode, QMeshPatch* singlePath);
	QMeshEdge* buildNewEdgetoQMeshPatch(QMeshPatch* patch, QMeshNode* startNode, QMeshNode* endNode);
	QMeshNode* findNextNearestPoint(QMeshNode* startNode, QMeshPatch* boundIsoPatch);
	bool detectAll_isoNode_Processed(QMeshPatch* singlePath);
	QMeshNode* link_OneRing_isoNode(QMeshPatch* singlePath, QMeshNode* sNode, QMeshNode* eNode);

	QMeshPatch* surfaceMesh;
};

#endif // TOOLPATHGENERATION_H