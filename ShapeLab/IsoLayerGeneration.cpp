#include <fstream>

#include "io.h"
#include "IsoLayerGeneration.h"

void IsoLayerGeneration::generateIsoSurface(PolygenMesh* isoSurface, int layerNum) {

	double isoCurveValue = 0.0;
	for (int i = 0; i < layerNum; i++) {

		isoCurveValue = (0.5 + i) * 1 / (double)layerNum;
		QMeshPatch* layer = _generatesingleIsoSurface(isoCurveValue, isoSurface);

		if (layer->GetNodeNumber() == 0) {
			std::cout << "this layer have no node!" << std::endl; continue;
		}
		layer->is_SupportLayer = false;
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount());//index begin from 0
		layer->compatible_layer_Index = layer->GetIndexNo();
		isoSurface->meshList.AddTail(layer);
		std::cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue << ", nodeNum = " << layer->GetNodeNumber() << std::endl;
	}
}

/*Main function for iso-surface substraction from tetrahedral model*/
QMeshPatch* IsoLayerGeneration::_generatesingleIsoSurface(double isoValue, PolygenMesh* isoSurface){

	QMeshPatch* layer = new QMeshPatch;
	layer->isoSurfaceValue = isoValue;

	//when the node iso-value is equal to surface value, add this eps.
	double eps = 1.0e-5;

	for (GLKPOSITION Pos = m_tetMesh->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetMesh->GetNodeList().GetNext(Pos);
		if (fabs(Node->scalarField - isoValue) < eps) {
			if (Node->scalarField > isoValue) Node->scalarField = isoValue + eps;
			else Node->scalarField = isoValue - eps;
		}
	}

	// build node list for isoSurface, this comes from the edge list
	for (GLKPOSITION Pos = m_tetMesh->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)m_tetMesh->GetEdgeList().GetNext(Pos);

		Edge->installedIsoNode = nullptr;
		Edge->isLocateIsoNode = false;

		double a = Edge->GetStartPoint()->scalarField;
		double b = Edge->GetEndPoint()->scalarField;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++)
				pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedTetEdge = Edge;
			isoNode->SetMeshPatchPtr(layer);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(layer->GetNodeList().GetCount() + 1);
			layer->GetNodeList().AddTail(isoNode);

			Edge->installedIsoNode = isoNode;
			Edge->isLocateIsoNode = true;

		}
	}

	// build edge list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = m_tetMesh->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)m_tetMesh->GetFaceList().GetNext(Pos);
		Face->installedIsoEdge = nullptr;
		Face->isLocatedIsoEdge = false;

		int positiveNum = 0;
		for (int i = 0; i < 3; i++) {
			if (Face->GetNodeRecordPtr(i)->scalarField > isoValue) positiveNum++;
		}

		if (positiveNum == 0 || positiveNum == 3) continue;
		else if (positiveNum == 1) {
			QMeshEdge* isoEdge = new QMeshEdge;

			//detect which node is positive
			QMeshNode* PostiveNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				PostiveNode = Face->GetNodeRecordPtr(index);
				if (PostiveNode->scalarField > isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
		else if (positiveNum == 2) {
			QMeshEdge* isoEdge = new QMeshEdge;
			//detect which node is negative
			QMeshNode* NegativeNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				NegativeNode = Face->GetNodeRecordPtr(index);
				if (NegativeNode->scalarField < isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
	}

	// build face list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = m_tetMesh->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetMesh->GetTetraList().GetNext(Pos);

		int isoEdgeNum = 0;
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) isoEdgeNum++;
		}
		if (isoEdgeNum == 0) continue;
		else if (isoEdgeNum == 2 || isoEdgeNum == 1) std::cout << "Error! isoEdgeNum cannot equal to 1 or 2!" << std::endl << std::endl;
		else if (isoEdgeNum == 3) {
			QMeshFace* isoFace = new QMeshFace;
			
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(3);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) {
				if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) {
					FaceList[faceIndex] = Tetra->GetFaceRecordPtr(i + 1);
					faceIndex++;
				}
			}

			//sorting
			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
			if (firstDir == true) {
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}
			else {
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}

			//using the first face and add its edge into the isoFace.
			for (int i = 0; i < 3; i++) {
				isoFace->SetEdgeRecordPtr(i, FaceList[i]->installedIsoEdge);
				isoFace->SetDirectionFlag(i, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])));
				if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
					FaceList[i]->installedIsoEdge->SetLeftFace(isoFace);
				else FaceList[i]->installedIsoEdge->SetRightFace(isoFace);
			}

			//push this isoFace back to layer and compute norm
			isoFace->SetEdgeNum(3);
			isoFace->CalPlaneEquation();
			isoFace->SetMeshPatchPtr(layer);
			isoFace->SetIndexNo(layer->GetFaceList().GetCount() + 1);
			layer->GetFaceList().AddTail(isoFace);
		}

		else if (isoEdgeNum == 4) {
			std::vector<QMeshFace*> isoFace; isoFace.resize(2);
			for (int i = 0; i < 2; i++) {
				isoFace[i] = new QMeshFace;
			}
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(4);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) FaceList[i] = Tetra->GetFaceRecordPtr(i + 1);

			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
																						//sorting edge
			if (firstDir == true) {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);
			}
			else {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);

			}
			////test the sorting
			//cout << Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])) << endl;
			//for (int i = 0; i < 4; i++) {
			//	cout << FaceList[i]->installedIsoEdge->GetStartPoint()->GetIndexNo() << " , " <<
			//		FaceList[i]->installedIsoEdge->GetEndPoint()->GetIndexNo() << endl;	
			//}

			QMeshEdge* midEdge1 = new QMeshEdge;
			midEdge1->isMiddleEdge1 = true;
			if (firstDir == true) {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge1->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge1->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/


			QMeshEdge* midEdge2 = new QMeshEdge;
			midEdge2->isMiddleEdge = true;
			if (firstDir == true) {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());

				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());

				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge2->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge2->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/

			//midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
			/*
			bool dir = false;
			if (FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetStartPoint());
			dir = true;
			}
			else if(FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetEndPoint());
			else cout << "Error!" << endl;*/

			QMeshEdge* midEdge;

			if (midEdge1->CalLength() <= midEdge2->CalLength()) {

				midEdge = midEdge1;
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, FaceList[1]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[0]->SetEdgeRecordPtr(2, midEdge);
				isoFace[0]->SetDirectionFlag(2, false);

				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 1) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetRightFace(isoFace[0]);

				isoFace[1]->SetEdgeRecordPtr(0, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[3]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, true);
				for (int i = 0; i < 4; i++) {
					if (i == 2 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[1]);
			}

			else {
				midEdge = midEdge2;
				//first triangle
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, midEdge);
				isoFace[0]->SetDirectionFlag(1, true);
				isoFace[0]->SetEdgeRecordPtr(2, FaceList[3]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(2, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[0]);

				//first triangle
				isoFace[1]->SetEdgeRecordPtr(0, FaceList[1]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, false);
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 1 || i == 2) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[1]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[1]);
					}
				}
				midEdge->SetRightFace(isoFace[1]);
			}

			//push back midEdge
			midEdge->SetMeshPatchPtr(layer);
			midEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge);


			/*midEdge1->SetMeshPatchPtr(layer);
			midEdge1->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge1);*/

			//midEdge2->SetMeshPatchPtr(layer);
			//midEdge2->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			//layer->GetEdgeList().AddTail(midEdge2);



			//push this isoFace back to layer and compute norm
			for (int i = 0; i < 2; i++) {
				isoFace[i]->SetEdgeNum(3);
				isoFace[i]->CalPlaneEquation();
				isoFace[i]->SetMeshPatchPtr(layer);
				isoFace[i]->SetIndexNo(layer->GetFaceList().GetCount() + 1);
				layer->GetFaceList().AddTail(isoFace[i]);
			}
		}
	}

	//give each node face list
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		for (int i = 0; i < 3; i++) {
			QMeshNode* Node = Face->GetNodeRecordPtr(i);
			Node->GetFaceList().AddTail(Face);
		}
	}

	return layer;
}

void IsoLayerGeneration::generateIsoSurface_support(PolygenMesh* isoSurface, QMeshPatch* patch_supportTet, int layerNum) {

	m_tetMesh_support = patch_supportTet;

	Eigen::VectorXd scalarField_Support(patch_supportTet->GetNodeNumber());

	for (GLKPOSITION Pos = patch_supportTet->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch_supportTet->GetNodeList().GetNext(Pos);
		scalarField_Support(Node->GetIndexNo()) = Node->scalarField;
	}

	double minPhi = scalarField_Support.minCoeff();		double maxPhi = scalarField_Support.maxCoeff();

	double gap = 1.0 / layerNum;
	int minLayerNum = (int)((0.5 * gap - minPhi) / gap);		// layer number on the top
	int maxLayerNum = (int)((maxPhi - (1 - 0.5 * gap)) / gap);	// layer number at the bottom
	int allLayerNum = layerNum + minLayerNum + maxLayerNum;

	std::cout << "[min, max] = " << minPhi << "," << maxPhi << std::endl;
	std::cout << "[minLayerNum, maxLayerNum, allLayerNum] = "
		<< minLayerNum << "," << maxLayerNum << "," << allLayerNum << std::endl;

	//multi-layer
	for (int i = 0; i < allLayerNum; i++) {

		double isoCurveValue = (0.5 - minLayerNum + i) * gap;
		QMeshPatch* layer = _generatesingleIsoSurface_support(isoCurveValue, isoSurface);

		if (layer->GetNodeNumber() == 0) {
			std::cout << "this layer have no node!" << std::endl; continue;
		}

		layer->is_SupportLayer = true;
		layer->SetIndexNo(isoSurface->GetMeshList().GetCount()); //index begin from 0
		isoSurface->meshList.AddTail(layer);

		std::cout << layer->GetIndexNo() << " Layer, isoValue = " << isoCurveValue
			<< ", nodeNum = " << layer->GetNodeNumber() << std::endl;
	}
}

QMeshPatch* IsoLayerGeneration::_generatesingleIsoSurface_support(double isoValue, PolygenMesh* isoSurface) {

	QMeshPatch* layer = new QMeshPatch;
	layer->isoSurfaceValue = isoValue;

	//when the node iso-value is equal to surface value, add this eps.
	double eps = 1.0e-5;

	for (GLKPOSITION Pos = m_tetMesh_support->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetMesh_support->GetNodeList().GetNext(Pos);
		if (fabs(Node->scalarField - isoValue) < eps) {
			if (Node->scalarField > isoValue) Node->scalarField = isoValue + eps;
			else Node->scalarField = isoValue - eps;
		}
	}

	// build node list for isoSurface, this comes from the edge list
	for (GLKPOSITION Pos = m_tetMesh_support->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)m_tetMesh_support->GetEdgeList().GetNext(Pos);

		//new code added by tianyu
		/*if ((!Edge->GetStartPoint()->is_tetSupportNode && !Edge->GetStartPoint()->model_boundary)
			|| (!Edge->GetEndPoint()->is_tetSupportNode && !Edge->GetEndPoint()->model_boundary)) continue;*/

		Edge->installedIsoNode = nullptr;
		Edge->isLocateIsoNode = false;

		double a = Edge->GetStartPoint()->scalarField;
		double b = Edge->GetEndPoint()->scalarField;

		if ((isoValue - a) * (isoValue - b) < 0.0) {
			double alpha = (isoValue - a) / (b - a);
			double p1[3], p2[3], pp[3];
			Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
			Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

			for (int j = 0; j < 3; j++)
				pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];

			QMeshNode* isoNode = new QMeshNode;
			isoNode->relatedTetEdge = Edge;
			isoNode->SetMeshPatchPtr(layer);
			isoNode->SetCoord3D(pp[0], pp[1], pp[2]);
			isoNode->SetIndexNo(layer->GetNodeList().GetCount() + 1);
			layer->GetNodeList().AddTail(isoNode);

			Edge->installedIsoNode = isoNode;
			Edge->isLocateIsoNode = true;

		}
	}

	// build edge list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = m_tetMesh_support->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)m_tetMesh_support->GetFaceList().GetNext(Pos);
		Face->installedIsoEdge = nullptr;
		Face->isLocatedIsoEdge = false;

		int positiveNum = 0; 
		//int innerNum = 0;
		for (int i = 0; i < 3; i++) {
			if (Face->GetNodeRecordPtr(i)->scalarField > isoValue) positiveNum++;
			//new code added by tianyu
			/*if (!Face->GetNodeRecordPtr(i)->is_tetSupportNode && !Face->GetNodeRecordPtr(i)->model_boundary) 
				innerNum++;*/
		}
		//if (innerNum > 0) continue;//new code added by tianyu
		if (positiveNum == 0 || positiveNum == 3) continue;
		else if (positiveNum == 1) {
			QMeshEdge* isoEdge = new QMeshEdge;

			//detect which node is positive
			QMeshNode* PostiveNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				PostiveNode = Face->GetNodeRecordPtr(index);
				if (PostiveNode->scalarField > isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
		else if (positiveNum == 2) {
			QMeshEdge* isoEdge = new QMeshEdge;
			//detect which node is negative
			QMeshNode* NegativeNode;
			int index = 0;
			for (index = 0; index < 3; index++) {
				NegativeNode = Face->GetNodeRecordPtr(index);
				if (NegativeNode->scalarField < isoValue) break;
			}

			QMeshEdge* Edge = Face->GetEdgeRecordPtr((index + 2) % 3 + 1);
			QMeshNode* startNode = Edge->installedIsoNode;
			isoEdge->SetStartPoint(startNode);

			Edge = Face->GetEdgeRecordPtr(index + 1);
			QMeshNode* endNode = Edge->installedIsoNode;
			isoEdge->SetEndPoint(endNode);

			isoEdge->SetMeshPatchPtr(layer);
			isoEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);

			(startNode->GetEdgeList()).AddTail(isoEdge);
			(endNode->GetEdgeList()).AddTail(isoEdge);

			layer->GetEdgeList().AddTail(isoEdge);
			Face->installedIsoEdge = isoEdge;
			Face->isLocatedIsoEdge = true;
		}
	}

	// build face list for isoSurface, this comes from the face list
	for (GLKPOSITION Pos = m_tetMesh_support->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetMesh_support->GetTetraList().GetNext(Pos);

		int isoEdgeNum = 0;
		for (int i = 0; i < 4; i++) {
			if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) isoEdgeNum++;
		}
		if (isoEdgeNum == 0) continue;
		else if (isoEdgeNum == 2 || isoEdgeNum == 1) std::cout << "Error! isoEdgeNum cannot equal to 1 or 2!" << std::endl << std::endl;
		else if (isoEdgeNum == 3) {
			QMeshFace* isoFace = new QMeshFace;

			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(3);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) {
				if (Tetra->GetFaceRecordPtr(i + 1)->isLocatedIsoEdge == true) {
					FaceList[faceIndex] = Tetra->GetFaceRecordPtr(i + 1);
					faceIndex++;
				}
			}

			//sorting
			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
			if (firstDir == true) {
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}
			else {
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[2]->installedIsoEdge->GetEndPoint()) {
					std::swap(FaceList[1], FaceList[2]);
				}
			}

			//using the first face and add its edge into the isoFace.
			for (int i = 0; i < 3; i++) {
				isoFace->SetEdgeRecordPtr(i, FaceList[i]->installedIsoEdge);
				isoFace->SetDirectionFlag(i, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])));
				if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
					FaceList[i]->installedIsoEdge->SetLeftFace(isoFace);
				else FaceList[i]->installedIsoEdge->SetRightFace(isoFace);
			}

			//push this isoFace back to layer and compute norm
			isoFace->SetEdgeNum(3);
			isoFace->CalPlaneEquation();
			isoFace->SetMeshPatchPtr(layer);
			isoFace->SetIndexNo(layer->GetFaceList().GetCount() + 1);
			layer->GetFaceList().AddTail(isoFace);
		}

		else if (isoEdgeNum == 4) {
			std::vector<QMeshFace*> isoFace; isoFace.resize(2);
			for (int i = 0; i < 2; i++) {
				isoFace[i] = new QMeshFace;
			}
			// build the face list
			std::vector<QMeshFace*> FaceList; FaceList.resize(4);
			int faceIndex = 0;
			for (int i = 0; i < 4; i++) FaceList[i] = Tetra->GetFaceRecordPtr(i + 1);

			bool firstDir = Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])); //detect which one should be the first node!!!
																						//sorting edge
			if (firstDir == true) {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);
			}
			else {
				for (int i = 0; i < 2; i++) {
					if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetStartPoint()
						|| FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[i + 2]->installedIsoEdge->GetEndPoint()) {
						std::swap(FaceList[1], FaceList[i + 2]);
					}
				}
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
					|| FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
				}
				else std::swap(FaceList[2], FaceList[3]);

			}
			////test the sorting
			//cout << Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])) << endl;
			//for (int i = 0; i < 4; i++) {
			//	cout << FaceList[i]->installedIsoEdge->GetStartPoint()->GetIndexNo() << " , " <<
			//		FaceList[i]->installedIsoEdge->GetEndPoint()->GetIndexNo() << endl;	
			//}

			QMeshEdge* midEdge1 = new QMeshEdge;
			midEdge1->isMiddleEdge1 = true;
			if (firstDir == true) {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());
				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge1->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetStartPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[1]->installedIsoEdge->GetEndPoint())
					midEdge1->SetEndPoint(FaceList[1]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge1->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge1->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/


			QMeshEdge* midEdge2 = new QMeshEdge;
			midEdge2->isMiddleEdge = true;
			if (firstDir == true) {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());

				if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 1" << std::endl;
			}
			else {
				midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetStartPoint());

				if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetEndPoint());
				else if (FaceList[0]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
					midEdge2->SetEndPoint(FaceList[3]->installedIsoEdge->GetStartPoint());
				else std::cout << "Wrong case 2" << std::endl;
			}

			/*cout << "The midEdge" <<count << "- "<<midEdge2->GetStartPoint()->GetIndexNo() << " , " <<
			midEdge2->GetEndPoint()->GetIndexNo() << endl;
			cout << endl;*/

			//midEdge2->SetStartPoint(FaceList[0]->installedIsoEdge->GetEndPoint());
			/*
			bool dir = false;
			if (FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetStartPoint() == FaceList[3]->installedIsoEdge->GetEndPoint()) {
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetStartPoint());
			dir = true;
			}
			else if(FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetStartPoint()
			|| FaceList[2]->installedIsoEdge->GetEndPoint() == FaceList[3]->installedIsoEdge->GetEndPoint())
			midEdge2->SetEndPoint(FaceList[2]->installedIsoEdge->GetEndPoint());
			else cout << "Error!" << endl;*/

			QMeshEdge* midEdge;

			if (midEdge1->CalLength() <= midEdge2->CalLength()) {

				midEdge = midEdge1;
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, FaceList[1]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[0]->SetEdgeRecordPtr(2, midEdge);
				isoFace[0]->SetDirectionFlag(2, false);

				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 1) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetRightFace(isoFace[0]);

				isoFace[1]->SetEdgeRecordPtr(0, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[3]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, true);
				for (int i = 0; i < 4; i++) {
					if (i == 2 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[1]);
			}

			else {
				midEdge = midEdge2;
				//first triangle
				isoFace[0]->SetEdgeRecordPtr(0, FaceList[0]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[0])));
				isoFace[0]->SetEdgeRecordPtr(1, midEdge);
				isoFace[0]->SetDirectionFlag(1, true);
				isoFace[0]->SetEdgeRecordPtr(2, FaceList[3]->installedIsoEdge);
				isoFace[0]->SetDirectionFlag(2, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[3])));
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 0 || i == 3) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[0]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[0]);
					}
				}
				midEdge->SetLeftFace(isoFace[0]);

				//first triangle
				isoFace[1]->SetEdgeRecordPtr(0, FaceList[1]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(0, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[1])));
				isoFace[1]->SetEdgeRecordPtr(1, FaceList[2]->installedIsoEdge);
				isoFace[1]->SetDirectionFlag(1, Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[2])));
				isoFace[1]->SetEdgeRecordPtr(2, midEdge);
				isoFace[1]->SetDirectionFlag(2, false);
				//set edge right / left face
				for (int i = 0; i < 4; i++) {
					if (i == 1 || i == 2) {
						if (Tetra->IsNormalDirection(Tetra->GetFaceIndex(FaceList[i])) == true)
							FaceList[i]->installedIsoEdge->SetLeftFace(isoFace[1]);
						else FaceList[i]->installedIsoEdge->SetRightFace(isoFace[1]);
					}
				}
				midEdge->SetRightFace(isoFace[1]);
			}

			//push back midEdge
			midEdge->SetMeshPatchPtr(layer);
			midEdge->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge);


			/*midEdge1->SetMeshPatchPtr(layer);
			midEdge1->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			layer->GetEdgeList().AddTail(midEdge1);*/

			//midEdge2->SetMeshPatchPtr(layer);
			//midEdge2->SetIndexNo(layer->GetEdgeList().GetCount() + 1);
			//layer->GetEdgeList().AddTail(midEdge2);



			//push this isoFace back to layer and compute norm
			for (int i = 0; i < 2; i++) {
				isoFace[i]->SetEdgeNum(3);
				isoFace[i]->CalPlaneEquation();
				isoFace[i]->SetMeshPatchPtr(layer);
				isoFace[i]->SetIndexNo(layer->GetFaceList().GetCount() + 1);
				layer->GetFaceList().AddTail(isoFace[i]);
			}
		}
	}

	//give each node face list
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		for (int i = 0; i < 3; i++) {
			QMeshNode* Node = Face->GetNodeRecordPtr(i);
			Node->GetFaceList().AddTail(Face);
		}
	}

	return layer;
}