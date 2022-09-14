#include "toolpathgeneration.h"
#include <iostream>
#include <fstream>
#include "GLKGeometry.h"

toolpathGeneration::toolpathGeneration(QMeshPatch* isoLayer, double deltaWidth, double deltaDistance)
{
	surfaceMesh = isoLayer;
	toolpath_Width = deltaWidth;
	toolpath_Distance = deltaDistance;
}

toolpathGeneration::~toolpathGeneration() {}

QMeshPatch* toolpathGeneration::generateBundaryToolPath() {

	int curveNum = autoComputeTPathNum();
	if (curveNum > 100) {
		std::cout << "layer = " << surfaceMesh->GetIndexNo() << " high curve num" << std::endl;
	}
	if (curveNum == 0 || curveNum > 100) return NULL;

	double deltaIsoValue = 1.0 / curveNum; // deltaIsoValue in [0-1]
	std::vector<double> isoValue(curveNum);// store all isoValue
	QMeshPatch* singlePath = new QMeshPatch;

	for (int i = 0; i < curveNum; i++) {
		isoValue[i] = (i + 0.5) * deltaIsoValue;

		//build isonode with each isoValue without linkage
		generateBoundaryIsoNode(singlePath, isoValue[i]);

		if (singlePath->GetNodeNumber() == 0) {
			std::cout << "Warning!! the boundary toolpath contains no isonode!\n";  //means no node in singlePath
		}
	}

	//DEBUG
	//for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
	//	std::cout << thisNode->GetIndexNo() << std::endl;
	//}
	//

	double min_isoValue = isoValue[0];
	double max_isoValue = isoValue[curveNum - 1];
	double startIsoValue;
	//choose printing direction (inner -> outside(true)) or inverse(false)
	bool growDirction = true;
	if (growDirction)	startIsoValue = min_isoValue;
	else startIsoValue = max_isoValue;
	linkEachIsoNode(singlePath, startIsoValue);

	//DEBUG
	/*for (GLKPOSITION posEdge = singlePath->GetEdgeList().GetHeadPosition(); posEdge != nullptr;) {
		QMeshEdge* Edge = (QMeshEdge*)singlePath->GetEdgeList().GetNext(posEdge);
		std::cout << "start Node " << Edge->GetStartPoint()->GetIndexNo() << " end Node " << Edge->GetEndPoint()->GetIndexNo() << std::endl;
	}*/
	//END

	//printf("--> Bundary Toolpath Generation Finish\n");

	return singlePath;
}

int toolpathGeneration::autoComputeTPathNum() {

	double distanceTPath = this->toolpath_Width;

	int iter = 10;
	std::vector<Eigen::MatrixXd> isoPoint(10);
	Eigen::VectorXd isoPointNum(iter);

	int maxNodeNum = 10000;

	for (int i = 0; i < iter; i++) {
		isoPoint[i] = Eigen::MatrixXd::Zero(maxNodeNum, 3);

		double isoValue = (0.5 + i) * 1.0 / iter;

		int index = 0;
		for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

			double a = Edge->GetStartPoint()->boundaryValue;
			double b = Edge->GetEndPoint()->boundaryValue;

			if ((isoValue - a) * (isoValue - b) < 0.0) {
				double alpha = (isoValue - a) / (b - a);
				double p1[3], p2[3], pp[3];
				Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
				Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

				for (int j = 0; j < 3; j++) {
					//compute the position for this isonode
					if (index > maxNodeNum) { printf("ERROR, node number too high!\n"); break; }
					isoPoint[i](index, j) = (1.0 - alpha) * p1[j] + alpha * p2[j];
				}
				index++;
			}
		}
		isoPointNum(i) = index;
		//std::cout << "isovalue " << isoValue << " layer has " << index << " points" << std::endl;
	}

	Eigen::VectorXd distance(iter - 1);
	for (int i = 0; i < iter - 1; i++) {
		distance(i) = 1000000.0;

		for (int j = 0; j < isoPointNum(i); j++) {
			for (int k = 0; k < isoPointNum(i + 1); k++) {
				double dis = (isoPoint[i].row(j) - isoPoint[i + 1].row(k)).norm();

				if (dis < distance(i)) distance(i) = dis;
			}
		}
	}
	//std::cout << distance << std::endl; // return the number of cut
	return floor(distance.sum() / distanceTPath); 
}

void toolpathGeneration::generateBoundaryIsoNode(QMeshPatch* singlePath, double isoValue) {

	//----build the node and install back to the "singlePath"
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);

		double a = Edge->GetStartPoint()->boundaryValue;
		double b = Edge->GetEndPoint()->boundaryValue;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++) {
				//compute the position for this isonode
				pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
			}

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedLayerEdge = Edge;
			isoNode->connectTPathProcessed = false;
			isoNode->isoValue = isoValue;

			isoNode->SetMeshPatchPtr(singlePath);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(singlePath->GetNodeList().GetCount()); //index should start from 0

			//cal normal of iso point
			double n1[3], n2[3], n3[3];
			Edge->GetLeftFace()->GetNormal(n1[0], n1[1], n1[2]);
			Edge->GetRightFace()->GetNormal(n2[0], n2[1], n2[2]);
			for (int i = 0; i < 3; i++) n3[i] = (n1[i] + n2[i]) / 2;
			isoNode->SetNormal(n3[0], n3[1], n3[2]);

			//install this isoNode of layer to its Edge
			Edge->installedIsoNode_layerEdge.push_back(isoNode);
			Edge->isLocateIsoNode_layerEdge = true;
			singlePath->GetNodeList().AddTail(isoNode);
		}
	}
}

void toolpathGeneration::linkEachIsoNode(QMeshPatch* singlePath, double startIsoValue) {

	/*DEGUG*/
	//for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
	//	QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
	//	std::cout << "thisNode->connectTPathProcessed = " << thisNode->connectTPathProcessed 
	//		<< " thisNode->isoValue = " << thisNode->isoValue << std::endl;
	//}
	/*END*/

	// get a First Node(boundFirstNode) to start the toolPath
	QMeshNode* boundFirstNode = nullptr;
	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (thisNode->connectTPathProcessed == false && startIsoValue == thisNode->isoValue) {
			thisNode->connectTPathProcessed = true;
			boundFirstNode = thisNode;
			break;
		}
	}

	QMeshNode* sNode = boundFirstNode;
	QMeshNode* eNode = findNextBoundaryToolPath(sNode, singlePath);
	/* Link all of iso-Node in one Layer*/
	do {
		
		// Start linking one ring with same iso-Value
		QMeshNode* unlinked_eNode = link_OneRing_isoNode(singlePath, sNode, eNode);
		if (unlinked_eNode == nullptr) return;
		
		if (detectAll_isoNode_Processed(singlePath)) return;
		
		QMeshNode* link_sNode = unlinked_eNode;
		QMeshNode* link_eNode = findNextNearestPoint(link_sNode, singlePath);
		if (link_eNode == nullptr) std::cout << "Error: Cannot find link_eNode" << std::endl;
		QMeshEdge* link_Edge = buildNewEdgetoQMeshPatch(singlePath, link_sNode, link_eNode);
		link_Edge->isConnectEdge = true;

		sNode = link_eNode;
		eNode = findNextBoundaryToolPath(sNode, singlePath);
		if (eNode == nullptr) std::cout << "Error: Cannot find next toolpath Node" << std::endl;

	} while (detectAll_isoNode_Processed(singlePath) == false);
}

QMeshNode* toolpathGeneration::findNextBoundaryToolPath(QMeshNode* sNode, QMeshPatch* singlePath) {

	QMeshNode* eNode;
	bool nextNodeDetected = false;
	QMeshEdge* thisEdge = sNode->relatedLayerEdge;

	//detect left face
	for (int i = 0; i < 3; i++) {
		QMeshEdge* NeighborEdge = thisEdge->GetLeftFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;

		if (NeighborEdge->isLocateIsoNode_layerEdge) {

			for (int j = 0; j < NeighborEdge->installedIsoNode_layerEdge.size(); j++) {

				QMeshNode* bNode = NeighborEdge->installedIsoNode_layerEdge[j];
				if (bNode->connectTPathProcessed == false && bNode->isoValue == sNode->isoValue) {
					nextNodeDetected = true;
					eNode = bNode;
					break;
				}
			}
		}
		if (nextNodeDetected == true) break;
	}

	if (nextNodeDetected) {
		eNode->connectTPathProcessed = true;
		return eNode;
	}

	//detect right face
	for (int i = 0; i < 3; i++) {
		QMeshEdge* NeighborEdge = thisEdge->GetRightFace()->GetEdgeRecordPtr(i + 1);
		if (NeighborEdge == thisEdge) continue;

		if (NeighborEdge->isLocateIsoNode_layerEdge) {

			for (int j = 0; j < NeighborEdge->installedIsoNode_layerEdge.size(); j++) {

				QMeshNode* bNode = NeighborEdge->installedIsoNode_layerEdge[j];
				if (bNode->connectTPathProcessed == false && bNode->isoValue == sNode->isoValue) {
					nextNodeDetected = true;
					eNode = bNode;
					break;
				}
			}
		}
		if (nextNodeDetected == true) break;
	}

	if (nextNodeDetected) { 
		eNode->connectTPathProcessed = true;
		return eNode; 
	}else {
		//std::cout<<"Error, the next node is not found!"<<std::endl;
		return nullptr;
	}
}

QMeshEdge* toolpathGeneration::buildNewEdgetoQMeshPatch(QMeshPatch* patch, QMeshNode* startNode, QMeshNode* endNode) {

	QMeshEdge* isoEdge = new QMeshEdge;

	//std::cout << "start Node " << startNode->GetIndexNo() << " end Node " << endNode->GetIndexNo() << std::endl;

	isoEdge->SetStartPoint(startNode);
	isoEdge->SetEndPoint(endNode);

	isoEdge->SetMeshPatchPtr(patch);
	//isoEdge->SetIndexNo(patch->GetEdgeList().GetCount() + 1);
	isoEdge->SetIndexNo(patch->GetEdgeList().GetCount());

	(startNode->GetEdgeList()).AddTail(isoEdge);
	(endNode->GetEdgeList()).AddTail(isoEdge);
	patch->GetEdgeList().AddTail(isoEdge);

	return isoEdge;
}

QMeshNode* toolpathGeneration::findNextNearestPoint(QMeshNode* sNode, QMeshPatch* singlePath){

	GLKGeometry geo;
	double pp[3]; sNode->GetCoord3D(pp);
	double p1[3];
	double dist = 1000.0;
	QMeshNode* nextNode = nullptr;

	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (Node->connectTPathProcessed == true) continue;
		Node->GetCoord3D(p1);
		double distancePP = geo.Distance_to_Point(pp, p1);
		if (distancePP < dist) {
			nextNode = Node;
			dist = distancePP;
		}
	}
	if (nextNode == nullptr) { 
		std::cout << "There is no isoNode need to link between different isoValue!" <<std::endl;
		return nullptr;
	}
	else {
		nextNode->connectTPathProcessed = true;
		return nextNode;
	}
}

bool toolpathGeneration::detectAll_isoNode_Processed(QMeshPatch* singlePath) {
	//if all the node being processed return true, else return false.
	for (GLKPOSITION Pos = singlePath->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)singlePath->GetNodeList().GetNext(Pos);
		if (thisNode->connectTPathProcessed == false) return false;
	}
	return true;
}

QMeshNode* toolpathGeneration::link_OneRing_isoNode(QMeshPatch* singlePath, QMeshNode* sNode, QMeshNode* eNode) {

	do {

		if (sNode == nullptr || eNode == nullptr) {
			std::cout << "NULL sNode or eNode" << std::endl;
			return nullptr;
		}

		buildNewEdgetoQMeshPatch(singlePath, sNode, eNode);

		sNode = eNode;
		eNode = findNextBoundaryToolPath(sNode, singlePath);

	} while (eNode != nullptr);

	return sNode;// return the sNode for the link opration between isoNode with different isoValue 
}

void toolpathGeneration::resampleToolpath(QMeshPatch* patch) {

	if (patch == NULL) return;

	double length = this->toolpath_Distance * 0.75;

	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		Node->resampleChecked = false;
	}

	QMeshEdge* sEdge = (QMeshEdge*)patch->GetEdgeList().GetHead();
	QMeshNode* sNode = sEdge->GetStartPoint(); // the first Node of Toolpath
	sNode->resampleChecked = true;

	QMeshNode* sPoint = sNode;	QMeshNode* ePoint;	double lsum = 0.0;// temp distance record
	// mark the resampleChecked nodes
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

		ePoint = Edge->GetStartPoint();
		if (ePoint == sPoint) ePoint = Edge->GetEndPoint();

		// give the ancor points (END point)
		if (Edge == patch->GetEdgeList().GetTail()) {

			ePoint->resampleChecked = true;
			break;
		}
		// give the ancor points (Linkage point)
		if (Edge->isConnectEdge == true) {

			sPoint->resampleChecked = true;	ePoint->resampleChecked = true;

			sPoint = ePoint;
			lsum = 0;
			continue;
		}

		lsum += Edge->CalLength();
		if (lsum > length) {
			
			ePoint->resampleChecked = true;
			sPoint = ePoint;
			lsum = 0;
		}
		else {
			sPoint = ePoint;
		}
	}
	// reorganize the toolpath node order(toolpath_NodeSet)
	std::vector<QMeshNode*> toolpath_NodeSet;
	toolpath_NodeSet.push_back(sNode);
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);

		ePoint = Edge->GetStartPoint();
		if (ePoint == sNode) ePoint = Edge->GetEndPoint();

		if (ePoint->resampleChecked)	toolpath_NodeSet.push_back(ePoint);

	}

	////DEBUG
	//std::cout << "---------------------------------- " << std::endl;
	//for (int i = 0; i < toolpath_NodeSet.size(); i++) {
	//	std::cout << "Node " << toolpath_NodeSet[i]->GetIndexNo() << std::endl;
	//}
	////END

	//rebuild the edge
	patch->GetEdgeList().RemoveAll();
	for (int i = 0; i < ( toolpath_NodeSet.size() - 1); i++)
		buildNewEdgetoQMeshPatch(patch, toolpath_NodeSet[i], toolpath_NodeSet[i + 1]);

	//printf("--> Resample Toolpath Finish\n");
}