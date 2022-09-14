#include "../GLKLib/GLKHeap.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

#include "BoundaryFieldCalculation.h"
#include "Graph.h"

// Program to find Dijkstra's shortest path using vector in STL
double BoundaryFieldCalculation::GenerateBndDistMap(QMeshPatch* mesh) {

	// initial index -----------------------------------------------------
	int index = 0; // initial index of Node
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(index); index++;
	}
	index = 0; // initial index of Face
	for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)mesh->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index); index++;
	}
	index = 0; // initial index of Edge
	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);
		Edge->SetIndexNo(index); index++;
	}

	//--------------------------------------------------------------------
	// initial + build a vector for finding the node pnt* rapidly
	std::vector<Graph_Node> graph;
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		node->SetAttribFlag(0, false);	// initial boundary flag
		Graph_Node graphNode;
		graphNode.attached_QNode = node;
		graph.push_back(graphNode);
	}

	//--------------------------------------------------------------------
	// initial boundary Edge and Node
	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);
		edge->SetAttribFlag(0, false);	// initial boundary flag
		edge->CalLength();

		if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) {
			edge->SetAttribFlag(0, true);
			edge->GetStartPoint()->SetAttribFlag(0, true);
			edge->GetEndPoint()->SetAttribFlag(0, true);
		}
	}
	// collect boundary node pnt into bndryNodePnt
	std::vector<QMeshNode*> bndryNodePnt;
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		if (node->GetAttribFlag(0)) bndryNodePnt.push_back(node);
	}

	//--------------------------------------------------------------------
	for (int i = 0; i < bndryNodePnt.size(); i++) {

		// initialize the graph vector
		for (int i = 0; i < graph.size(); i++) {

			graph[i].isVisited = false;
			graph[i].dist = 1.0e5;
			graph[i].parent_QNode = NULL;
		}

		// pick up a source Node and find the shortest distance to all other nodes
		int graphIndex = bndryNodePnt[i]->GetIndexNo();
		graph[graphIndex].isVisited = true;
		graph[graphIndex].dist = 0.0;

		
		QMeshNode* node = graph[graphIndex].attached_QNode;

		while (_visitedAll(graph) == false) {

			int edgeNum = node->GetEdgeNumber();

			for (int i = 0; i < edgeNum; i++) {

				QMeshEdge* edge = node->GetEdgeRecordPtr(i + 1);
				QMeshNode* linked_Node = edge->GetEndPoint();
				if (linked_Node == node)
					linked_Node = edge->GetStartPoint();

				if (graph[linked_Node->GetIndexNo()].isVisited == false) {

					if (graph[graphIndex].dist + edge->GetLength() < graph[linked_Node->GetIndexNo()].dist) {
						graph[linked_Node->GetIndexNo()].dist = graph[graphIndex].dist + edge->GetLength();
						graph[linked_Node->GetIndexNo()].parent_QNode = node;
					}
				}
			}

			double minDist = 1.0e5;
			int minGraphNodeIndex = -1;
			for (int i = 0; i < graph.size(); i++) {
				if (graph[i].isVisited == false) {
					if (graph[i].dist < minDist) {
						minDist = graph[i].dist;
						minGraphNodeIndex = i;
					}
				}
			}
			graph[minGraphNodeIndex].isVisited = true;
			node = graph[minGraphNodeIndex].attached_QNode;
			graphIndex = minGraphNodeIndex;

		}

		// move min_Dis into multiSrc_dist
		for (int i = 0; i < graph.size(); i++) {
			if (graph[i].multiSrc_dist > graph[i].dist) {
				graph[i].multiSrc_dist = graph[i].dist; 
			}
		}

	}
	//get multiSrc_distance range : [0 - max_multiDist]
	double max_multiDist = -1.0e5;
	for (int i = 0; i < graph.size(); i++) {
		if (graph[i].multiSrc_dist > max_multiDist) {
			max_multiDist = graph[i].multiSrc_dist;
		}
	}
	// give graph[i].dist -> SetBoundaryDis(0.0); normalized
	for (int i = 0; i < graph.size(); i++) {

		graph[i].attached_QNode->boundaryValue = 1.0 - graph[i].multiSrc_dist / max_multiDist;
	}

	return max_multiDist;
}

// Program to find Dijkstra's shortest path using priority_queue in STL
double BoundaryFieldCalculation::GenerateBndDistMap1(QMeshPatch* mesh) {

	//--------------------------------------------------------------------
	// initial index
	int index = 0; // initial index of Node
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(index); index++;
	}
	index = 0; // initial index of Face
	for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)mesh->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index); index++;
	}
	index = 0; // initial index of Edge
	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);
		Edge->SetIndexNo(index); index++;
	}

	//--------------------------------------------------------------------
	// initial Node flag
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		node->SetAttribFlag(0, false);	// initial boundary flag
		node->boundaryValue = INF;		// initial boundary value for minValue collection
	}
	// mark boundary Edge and Node
	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);
		edge->SetAttribFlag(0, false);	// initial boundary flag
		edge->CalLength();

		if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) {
			edge->SetAttribFlag(0, true);
			edge->GetStartPoint()->SetAttribFlag(0, true);
			edge->GetEndPoint()->SetAttribFlag(0, true);
		}
	}

	// collect boundary node pnt into bndryNodePnt
	std::vector<int> bndryNodeInd;
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		if (node->GetAttribFlag(0)) bndryNodeInd.push_back(node->GetIndexNo());
	}

	//--------------------------------------------------------------------
	// MAIN function of Dijkstra shortest path algorithm
	//--------------------------------------------------------------------

	// build a graph for shortest path searching
	Graph g(mesh->GetNodeNumber(), mesh);

	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		
		int edgeNum = node->GetEdgeNumber();

		for (int i = 0; i < edgeNum; i++) {

			QMeshEdge* edge = node->GetEdgeRecordPtr(i + 1);
			QMeshNode* linked_Node = edge->GetEndPoint();
			if (linked_Node == node)
				linked_Node = edge->GetStartPoint();

			g.addEdge(node->GetIndexNo(), edge->GetLength(), linked_Node->GetIndexNo());

		}
	}
	//--------------------------------------------------------------------
	// multi-sorce Dijkstra shortest path algorithm
	for (int i = 0; i < bndryNodeInd.size(); i++) {

		g.shortestPath(bndryNodeInd[i]);

		for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
			
			if (Node->boundaryValue_temp < Node->boundaryValue)  Node->boundaryValue = Node->boundaryValue_temp;
		}
	}

	//get multiSrc_distance range : [0 - max_multiDist]
	double max_multiDist = -1.0e5;
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);

		if (max_multiDist < Node->boundaryValue)  max_multiDist = Node->boundaryValue;
	}

	// give graph[i].dist -> SetBoundaryDis(0.0); normalized
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);

		Node->boundaryValue = 1.0 - Node->boundaryValue / max_multiDist;
	}

	return max_multiDist;
}

// Program to find Dijkstra's shortest path using priority_queue in STL+ multi-channel
double BoundaryFieldCalculation::GenerateBndDistMap2(QMeshPatch* mesh) {

	//--------------------------------------------------------------------
	// initial index
	int index = 0; // initial index of Node
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		Node->SetIndexNo(index); index++;
	}
	index = 0; // initial index of Face
	for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)mesh->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index); index++;
	}
	index = 0; // initial index of Edge
	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);
		Edge->SetIndexNo(index); index++;
	}

	//--------------------------------------------------------------------
	// install midNode on each Edge
	int baseNum = mesh->GetNodeNumber();// the index is added on the meshNode number
	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);

		double startPP[3]; double endPP[3]; double midPP[3];

		Edge->GetStartPoint()->GetCoord3D(startPP);
		Edge->GetEndPoint()->GetCoord3D(endPP);
		for (int i = 0; i < 3; i++) {
			midPP[i] = (startPP[i] + endPP[i]) / 2;
		}

		QMeshNode* node = new QMeshNode;
		node->SetCoord3D(midPP[0], midPP[1], midPP[2]);
		node->SetIndexNo(Edge->GetIndexNo() + baseNum);
		node->SetAttribFlag(0, false);	// initial boundary mid node flag
		node->boundaryValue = INF;

		Edge->midNode = node;
	}

	//--------------------------------------------------------------------
	// initial Node flag
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		node->SetAttribFlag(0, false);	// initial boundary flag
		node->boundaryValue = INF;		// initial boundary value for minValue collection
	}

	// mark boundary Edge and Node
	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);
		edge->SetAttribFlag(0, false);	// initial boundary flag
		edge->CalLength();

		if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) {
			edge->SetAttribFlag(0, true);
			edge->GetStartPoint()->SetAttribFlag(0, true);
			edge->GetEndPoint()->SetAttribFlag(0, true);
			edge->midNode->SetAttribFlag(0, true);
		}
	}

	// collect boundary node pnt into bndryNodePnt
	std::vector<int> bndryNodeInd;
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);
		if (node->GetAttribFlag(0)) bndryNodeInd.push_back(node->GetIndexNo());
	}

	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);

		if (Edge->midNode->GetAttribFlag(0)) bndryNodeInd.push_back(Edge->midNode->GetIndexNo());
	}

	//--------------------------------------------------------------------
	// MAIN function of Dijkstra shortest path algorithm + multi-channel
	//--------------------------------------------------------------------

	// build a graph for shortest path searching
	Graph g(mesh->GetNodeNumber() + mesh->GetEdgeNumber(), mesh);

	// build link of orginal QMeshNode
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);

		int edgeNum = node->GetEdgeNumber();

		for (int i = 0; i < edgeNum; i++) {

			QMeshEdge* edge = node->GetEdgeRecordPtr(i + 1);
			QMeshNode* linked_Node = edge->midNode;

			double ll = _calQMeshNodeDist(node, linked_Node);
			g.addEdge(node->GetIndexNo(), ll, linked_Node->GetIndexNo());
		}
	}

	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);

		// each face linked with Node
		int faceNum = node->GetFaceNumber();
		for (int i = 0; i < faceNum; i++) {

			QMeshFace* face = node->GetFaceRecordPtr(i + 1);

			//each edge linked with Face
			int edgeNum_face = face->GetEdgeNum();
			for (int j = 0; j < edgeNum_face; j++) {
				QMeshEdge* edge_face = face->GetEdgeRecordPtr(j + 1);

				if (edge_face->GetStartPoint() != node || edge_face->GetEndPoint() != node) {

					QMeshNode* linked_Node = edge_face->midNode;
					double ll = _calQMeshNodeDist(node, linked_Node);
					g.addEdge(node->GetIndexNo(), ll, linked_Node->GetIndexNo());
					break;
				}
			}
		}
	}

	// build link of middle QMeshNode
	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);

		double ll = _calQMeshNodeDist(edge->midNode, edge->GetStartPoint());

		g.addEdge(edge->midNode->GetIndexNo(), ll, edge->GetStartPoint()->GetIndexNo());

		ll = _calQMeshNodeDist(edge->midNode, edge->GetEndPoint());

		g.addEdge(edge->midNode->GetIndexNo(), ll, edge->GetEndPoint()->GetIndexNo());
	}

	for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetNext(Pos);

		std::vector<QMeshFace*> faces_ofEdge;
		if (edge->GetLeftFace() != NULL) faces_ofEdge.push_back(edge->GetLeftFace());
		if (edge->GetRightFace() != NULL) faces_ofEdge.push_back(edge->GetRightFace());

		if (std::size(faces_ofEdge) == 0) std::cout << "Error: this edge has no neighbor face." << std::endl;

		for (int i = 0; i < std::size(faces_ofEdge); i++) {

			for (int j = 0; j < 3; j++) {

				QMeshEdge* edge_ofFace = faces_ofEdge[i]->GetEdgeRecordPtr(j + 1);
				if (edge_ofFace == NULL)	std::cout << "Error: this face has no edge." << std::endl;

				if (edge_ofFace != edge) {
					double ll = _calQMeshNodeDist(edge->midNode, edge_ofFace->midNode);
					g.addEdge(edge->midNode->GetIndexNo(), ll, edge_ofFace->midNode->GetIndexNo());
				}
			}

			for (int k = 0; k < 3; k++) {

				QMeshNode* node_ofFace = faces_ofEdge[i]->GetNodeRecordPtr(k);
				if (node_ofFace == NULL) std::cout << "Error: this face has no node." << std::endl;

				if (node_ofFace != edge->GetStartPoint() && node_ofFace != edge->GetEndPoint()) {

					double ll = _calQMeshNodeDist(edge->midNode, node_ofFace);
					g.addEdge(edge->midNode->GetIndexNo(), ll, node_ofFace->GetIndexNo());

					break;
				}
			}
		}
	}

	//--------------------------------------------------------------------
	// multi-sorce Dijkstra shortest path algorithm + multi-channel
	for (int i = 0; i < bndryNodeInd.size(); i++) {

		g.shortestPath(bndryNodeInd[i]);

		for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);

			if (Node->boundaryValue_temp < Node->boundaryValue)  Node->boundaryValue = Node->boundaryValue_temp;
		}
	}

	//get multiSrc_distance range : [0 - max_multiDist]
	double max_multiDist = -1.0e5;
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);

		if (max_multiDist < Node->boundaryValue)  max_multiDist = Node->boundaryValue;
	}

	// give graph[i].dist -> SetBoundaryDis(0.0); normalized
	for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)mesh->GetNodeList().GetNext(Pos);

		Node->boundaryValue = 1.0 - Node->boundaryValue / max_multiDist;
	}

	return max_multiDist;
}

// Subfunction
void BoundaryFieldCalculation::meshRefinement(QMeshPatch* surfaceMesh) {

	int nodeNum = surfaceMesh->GetNodeNumber() + surfaceMesh->GetEdgeNumber();
	int faceNum = surfaceMesh->GetFaceNumber() * 4;

	//Eigen::VectorXd scalarField = Eigen::VectorXd::Zero(nodeNum);

	// build nodeNum and nodeTable - refinement
	float* nodeTable;
	nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
	int index = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
		double pp[3]; Node->GetCoord3D(pp); Node->SetIndexNo(index); //scalarField(index) = Node->zigzagValue;
		for (int i = 0; i < 3; i++) nodeTable[index * 3 + i] = (float)pp[i];
		index++;
	}

	index = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)surfaceMesh->GetEdgeList().GetNext(Pos);
		double p1[3], p2[3];
		Edge->GetStartPoint()->GetCoord3D(p1); Edge->GetEndPoint()->GetCoord3D(p2);
		//scalarField(index + surfaceMesh->GetNodeNumber()) = (Edge->GetStartPoint()->zigzagValue + Edge->GetEndPoint()->zigzagValue) / 2;
		for (int i = 0; i < 3; i++) nodeTable[(index + surfaceMesh->GetNodeNumber()) * 3 + i] = (float)((p1[i] + p2[i]) / 2);
		Edge->refineNodeIndex = index; index++;
	}

	// build faceNum and faceTable - refinement

	unsigned int* faceTable;
	faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);
	index = 0;
	for (GLKPOSITION Pos = surfaceMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* face = (QMeshFace*)surfaceMesh->GetFaceList().GetNext(Pos);
		int nodeIndex[6];
		for (int i = 0; i < 3; i++) {
			nodeIndex[i] = face->GetNodeRecordPtr(i)->GetIndexNo();
			nodeIndex[3 + i] = face->GetEdgeRecordPtr(i + 1)->refineNodeIndex + surfaceMesh->GetNodeNumber();
		}

		int faceNodeIndex[12] = { 1,4,6,4,2,5,4,5,6,6,5,3 };
		for (int i = 0; i < 12; i++)
			faceTable[index * 12 + i] = nodeIndex[faceNodeIndex[i] - 1];
		index++;

	}

	// reconstruct the mesh
	surfaceMesh->ClearAll();
	surfaceMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

	//protect the stress field value
	//index = 0;
	//for (GLKPOSITION Pos = surfaceMesh->GetNodeList().GetHeadPosition(); Pos;) {
	//    QMeshNode* Node = (QMeshNode*)surfaceMesh->GetNodeList().GetNext(Pos);
	//    Node->zigzagValue = scalarField(index); index++;
	//}
	//std::cout << surfaceMesh->GetNodeNumber() << std::endl;

	free(nodeTable);
	free(faceTable);

	//std::cout << "--> Refine Mesh Finish (one time)" << std::endl;
}

double BoundaryFieldCalculation::_calQMeshNodeDist(QMeshNode* node1, QMeshNode* node2) {

	double pp1[3]; double pp2[3];
	node1->GetCoord3D(pp1[0], pp1[1], pp1[2]);
	node2->GetCoord3D(pp2[0], pp2[1], pp2[2]);

	double ll = sqrt((pp1[0] - pp2[0]) * (pp1[0] - pp2[0])
		+ (pp1[1] - pp2[1]) * (pp1[1] - pp2[1])
		+ (pp1[2] - pp2[2]) * (pp1[2] - pp2[2]));
	return ll;
}

bool BoundaryFieldCalculation::_visitedAll(std::vector<Graph_Node> graph) {

	bool allVisited = true;

	for (int i = 0; i < graph.size(); i++) {
		if (graph[i].isVisited == false) {
			allVisited = false;
			break;
		}
	}

	return allVisited;
}