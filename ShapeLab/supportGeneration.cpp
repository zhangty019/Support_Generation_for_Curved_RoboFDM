#include <fstream>
#include "../GLKLib/GLKLib.h"
#include "../GLKLib/GLKGeometry.h"
#include "../QMeshLib/PolygenMesh.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

#include "QHullLib/qhull_a.h"
#include "PMBody.h"

#include <igl/copyleft/marching_cubes.h>
#include <igl/marching_cubes.h>

#include "supportGeneration.h"
#include "heatmethodfield.h"
#include "toolpathGeneration.h"
#include "BoundaryFieldCalculation.h"

void supportGeneration::initial(
	PolygenMesh* tetModel, PolygenMesh* layers, PolygenMesh* platform,PolygenMesh* supportRaySet, 
	PolygenMesh* tight_supportLayerSet, PolygenMesh* toolpathSet_support, 
	PolygenMesh* toolpathSet_initial, std::string model_name) {

	m_tetModel = tetModel;
	m_layers = layers;
	m_platform = platform;
	m_supportRaySet = supportRaySet;
	m_tight_supportLayerSet = tight_supportLayerSet;
	m_model_name = model_name;
	m_toolpathSet_support = toolpathSet_support;
	m_toolpathSet_initial = toolpathSet_initial;
		

	// reorganize the index of layers
	std::vector<int> largeLayer_IndexTable(m_layers->GetMeshList().GetCount());
	std::vector<QMeshPatch*> layer_pnt_table(m_layers->GetMeshList().GetCount());

	int layerindex = 0;
	for (GLKPOSITION Pos = m_layers->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_layers->GetMeshList().GetNext(Pos);

		layer->SetIndexNo(layerindex); //start from 0
		layer_pnt_table[layer->GetIndexNo()] = layer;
		layerindex++;
	}

	// reorganize the index of large layers, which means a big isoLayer with support and initial material
	int large_layerindex = 0;
	largeLayer_IndexTable[0] = 0;
	for (int i = 0; i < (layer_pnt_table.size() - 1); i++) {

		if ((layer_pnt_table[i]->is_SupportLayer == layer_pnt_table[i + 1]->is_SupportLayer)
			&&(layer_pnt_table[i]->largeLayer_Index != layer_pnt_table[i + 1]->largeLayer_Index)) {

			large_layerindex = large_layerindex + 1;
		}

		//deal with the special case, meanless： yoga_cut: 7900S 8000S, -8100-, 8200S 8300S
		//if (i == 80 && layer_pnt_table[i]->is_SupportLayer && m_model_name == "yoga_cut") {
		//	large_layerindex++;
		//}
		if (i == 82 && layer_pnt_table[i]->is_SupportLayer && m_model_name == "helmetSmall_cut") {
			large_layerindex++;
		}
		if (i == 40 && layer_pnt_table[i]->is_SupportLayer && m_model_name == "airbus_topopt_cut") {
			large_layerindex++;
		}

		largeLayer_IndexTable[i + 1] = large_layerindex;
	}

	for (GLKPOSITION Pos = m_layers->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_layers->GetMeshList().GetNext(Pos);

		layer->largeLayer_Index = largeLayer_IndexTable[layer->GetIndexNo()];

		//std::cout << "after: " << layer->largeLayer_Index << " layer name: " 
		//<< layer->patchName << " layer index: " << layer->GetIndexNo() << std::endl;
	}

	/*has been ensured when input layers, so masked*/
	//recompute the normal for the mesh
	//for (GLKPOSITION Pos = m_layers->GetMeshList().GetHeadPosition(); Pos;) {
	//	QMeshPatch* layer = (QMeshPatch*)m_layers->GetMeshList().GetNext(Pos);
	//
	//	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
	//		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
	//		Face->CalPlaneEquation();
	//	}
	//
	//	for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
	//		QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
	//		Node->CalNormal();
	//		Node->SetAttribFlag(0, false); // reset the boundary flag[0]
	//	}
	//}

	// fill the initial_patch & support_patch vector with proper order
	for (GLKPOSITION Pos = m_layers->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_layers->GetMeshList().GetNext(Pos);

		if (layer->patchName.find("S") != std::string::npos) {
			support_layers.push_back(layer);
		}
		else {
			initial_layers.push_back(layer);
		}
	}

	// mark the boundary node of initial layers
	for (int i = 0; i < initial_layers.size(); i++) {
		int boundary_edge_NUM = 0;
		double boundary_edge_length = 0.0;
		//std::cout << initial_layers[i]->patchName << std::endl;
		for (GLKPOSITION Pos = initial_layers[i]->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* Edge = (QMeshEdge*)initial_layers[i]->GetEdgeList().GetNext(Pos);
			if (Edge->GetLeftFace() == NULL || Edge->GetRightFace() == NULL) {
				Edge->GetStartPoint()->SetAttribFlag(0, true); // boundary node flag[0];
				Edge->GetEndPoint()->SetAttribFlag(0, true);

				boundary_edge_NUM++;
				boundary_edge_length += Edge->CalLength();
			}
		}
		initial_layers[i]->average_boundary_edge_length = boundary_edge_length / boundary_edge_NUM;
	}

	//flip the normal of face on tet surface (method: )
	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();
	for (GLKPOSITION PosFace = (tet_Model->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((tet_Model->GetFaceList()).GetNext(PosFace));

		double nx, ny, nz;
		face->CalPlaneEquation();
		face->GetNormal(nx, ny, nz);
		face->m_desiredNormal[0] = -nx;		face->m_desiredNormal[1] = -ny;		face->m_desiredNormal[2] = -nz;
	}

	//calculate the center coordinate of each face of ISO-layers
	for (GLKPOSITION Pos = m_layers->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_layers->GetMeshList().GetNext(Pos);

		for (GLKPOSITION PosFace = (layer->GetFaceList()).GetHeadPosition(); PosFace;) {
			QMeshFace* face = (QMeshFace*)((layer->GetFaceList()).GetNext(PosFace));

			face->CalCenterPos();
		}
	}

	//tau tuning for self-support angle change
	if (m_tetModel->getModelName() == "bunnyhead_fabrication" ||
		m_tetModel->getModelName() == "dome_fabrication" ||
		m_tetModel->getModelName() == "bridgeSmall_fabrication"||
		m_tetModel->getModelName() == "topopt_new_fabrication"||
		m_tetModel->getModelName() == "arch_fabrication"
		) {
		tau = 50.0;
	}
	if (m_tetModel->getModelName() == "YogaNew_fabrication") {
		tau = 35.0;
	}
	if (m_tetModel->getModelName() == "helmetSmall_fabrication") {
		tau = 35.0;
	}
	if (m_tetModel->getModelName() == "CSquare_fabrication") {
		tau = 35.0;
	}
	if (m_tetModel->getModelName() == "airbus_topopt_fabrication") {
		tau = 35.0;
	}
}

// for initial support guess
void supportGeneration::initial(
	PolygenMesh* tetModel, 
	PolygenMesh* platform, 
	PolygenMesh* supportRaySet, 
	std::string model_name) {

	m_tetModel = tetModel;
	m_platform = platform;
	m_supportRaySet = supportRaySet;
	m_model_name = model_name;

	//tau tuning for self-support angle change
	if (m_tetModel->getModelName() == "bunnyhead_fabrication" ||
		m_tetModel->getModelName() == "dome_fabrication" ||
		m_tetModel->getModelName() == "topopt_new_fabrication"||
		m_tetModel->getModelName() == "bridgeSmall_fabrication"
		) {
		tau = 35.0;
	}
	if (m_tetModel->getModelName() == "YogaNew_fabrication"||
		m_tetModel->getModelName() == "helmetSmall_fabrication" ||
		m_tetModel->getModelName() == "arch_fabrication") {
		tau = 15.0;
	}
	if (m_tetModel->getModelName() == "CSquare_fabrication") {
		tau = 35.0;
	}
	if (m_tetModel->getModelName() == "airbus_topopt_fabrication") {
		tau = 15.0;
	}

	//flip the normal of face on tet surface (method: )
	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();
	for (GLKPOSITION PosFace = (tet_Model->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((tet_Model->GetFaceList()).GetNext(PosFace));

		double nx, ny, nz;
		face->CalPlaneEquation();
		face->GetNormal(nx, ny, nz);
		face->m_desiredNormal[0] = -nx;		face->m_desiredNormal[1] = -ny;		face->m_desiredNormal[2] = -nz;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// get the support RAY for isoLayers
void supportGeneration::build_SupportRays() {

#pragma omp parallel
	{
#pragma omp for  

		for (int i = 0; i < initial_layers.size(); i++) {
			QMeshPatch* tLayer = initial_layers[i];

			//if (tLayer->GetIndexNo() != 35) continue;
			//std::cout << "Check Layer No. " << tLayer->GetIndexNo() << std::endl;

			Eigen::Vector3d insertP, v1, v2, v3; //caution：omp
			Eigen::Vector3d platform_Normal; platform_Normal << 0, 0, 1;

			/*-- First layer always need to have support with platform --*/
			if (i == 0) {

				for (GLKPOSITION Pos = tLayer->GetNodeList().GetHeadPosition(); Pos;) {
					QMeshNode* Node = (QMeshNode*)tLayer->GetNodeList().GetNext(Pos);
					Node->need_Support = true;	//reset

					Eigen::Vector3d orig; 	Node->GetCoord3D(orig(0), orig(1), orig(2));
					//m_desiredNormal is the correctted normal
					Eigen::Vector3d dir; 	for (int k = 0; k < 3; k++) { dir[k] = Node->m_desiredNormal[k]; }

					double t = -platform_Normal.dot(orig) / platform_Normal.dot(dir);
					insertP = orig + t * dir;

					Node->supportEndPos = insertP;
				}

				continue;
			}

			/*-- For the rest, build a bottom layer set and check --*/

			std::vector<QMeshPatch*> bLayerSet(i);// collect the bottom layers under top tLayer
			for (int j = 0; j < i; j++) {bLayerSet[j] = initial_layers[j];}


			for (GLKPOSITION Pos = tLayer->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode* Node = (QMeshNode*)tLayer->GetNodeList().GetNext(Pos);

				Eigen::Vector3d orig; Node->GetCoord3D(orig(0), orig(1), orig(2));
				Eigen::Vector3d dir; 	for (int k = 0; k < 3; k++) { dir[k] = Node->m_desiredNormal[k]; }
				
				Node->need_Support = true;	//reset
				//check if it will interset with the surface below (all);
				bool support_withplatform = true;

				for (int index = bLayerSet.size() - 1; index >= 0; index--) {
					QMeshPatch* bLayer = bLayerSet[index];

					//get the expand ratio: 0.5mm = sqrt(3)/2/3*average_length*ratio
					//bFaceExpand_ratio = sqrt(3.0) / bLayer->average_boundary_edge_length;
					//std::cout << "bFaceExpand_ratio: " << bFaceExpand_ratio << std::endl;

					for (GLKPOSITION Pos = bLayer->GetFaceList().GetHeadPosition(); Pos;) {
						QMeshFace* Face = (QMeshFace*)bLayer->GetFaceList().GetNext(Pos);

						Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
						Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
						Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));
						
						if (Face->GetNodeRecordPtr(0)->GetAttribFlag(0)
							|| Face->GetNodeRecordPtr(1)->GetAttribFlag(0)
							|| Face->GetNodeRecordPtr(2)->GetAttribFlag(0)) {
						
							Eigen::Vector3d va = (v1 + v2 + v3) / 3;
							v1 = v1 + bFaceExpand_ratio * (v1 - va);
							v2 = v2 + bFaceExpand_ratio * (v2 - va);
							v3 = v3 + bFaceExpand_ratio * (v3 - va);
						}
						//original normal detection
						if (_intersetTriangle(orig, dir, v1, v2, v3, insertP)) {

							if (tLayer->largeLayer_Index == (bLayer->largeLayer_Index + 1)) {
								//std::cout << "supported by below layer (layer i-1), don't need support " << endl;
								Node->need_Support = false;
								support_withplatform = false;
								break;
							}
							else {
								//std::cout << "supported by below layer (not layer i-1), need support " << endl;
								Node->need_Support = true;
								Node->supportEndPos = insertP;
								support_withplatform = false;
								break;
							}
						}
						/*
						else // normal set detection
						{
							std::vector<Eigen::Vector3d> dir_set;// generate a set of candidate normals on cone
							_generate_normalSet_for_supportDetection(dir, dir_set);

							for (int normal_index = 0; normal_index < dir_set.size(); normal_index++) {
								if (_intersetTriangle(orig, dir_set[normal_index], v1, v2, v3, insertP)) {

									if (tLayer->largeLayer_Index == (bLayer->largeLayer_Index + 1)) {
										//std::cout << "supported by below layer (layer i-1), don't need support " << endl;
										Node->need_Support = false;
										support_withplatform = false;
										break;
									}
									else {
										//std::cout << "supported by below layer (not layer i-1), need support " << endl;
										Node->need_Support = true;
										Node->supportEndPos = insertP;
										support_withplatform = false;
										break;
									}
								}
							}
						}
						*/
					}

					if (Node->need_Support == false) break;
					if (Node->need_Support == true && support_withplatform == false) break;
				}

				if (support_withplatform == true) {
					//std::cout << "supported by platform " << endl;
					Eigen::Vector3d n; n << 0, 0, 1;

					if (fabs(n.dot(dir)) < 0.7) {
						dir = (dir * 0.5 - n * 0.5).normalized();
						//std::cout << "flat the normal!" << std::endl;
					}

					double t = -n.dot(orig) / n.dot(dir);
					insertP = orig + t * dir;
					Node->supportEndPos = insertP;
					Node->need_Support = true;
				}

			}

			std::cout << "find support ray of iso-layer " << i << std::endl;
		}
	}

}

// get the support RAY based on tetSurface and vector field
void supportGeneration::build_SupportRays_vertical() {

	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();

	//calculate the direction of support Ray; by laplacian smoothness
	_cal_Ray_direction(tet_Model);

	// reconstruct the Node Array for omp usage
	std::vector<QMeshNode*> tet_surfaceNode_set;
	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);
		tet_surfaceNode_set.push_back(Node);
	}
#pragma omp parallel
	{
#pragma omp for  

		for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
			QMeshNode* Node = tet_surfaceNode_set[i];

			if (Node->inner) {
				/*double xx, yy, zz;
				Node->GetCoord3D(xx, yy, zz);
				std::cout << "inner node : " << xx << " " << yy << " " << zz << std::endl;*/
				continue;
			}	// the inner face is not considered 
			if (Node->need_Support == false) continue;

			Eigen::Vector3d orig; Node->GetCoord3D(orig(0), orig(1), orig(2));
			//1.0 -->//vertical;Eigen::Vector3d dir = { 0.0,0.0,1.0 };
			//2.0 -->//normal direction
			Eigen::Vector3d dir = Node->supportRay_Dir;

			//check if it will interset with the tet boundary surface;
			bool support_withplatform = true;
			int intersect_NUM = 0;
			Eigen::Vector3d closest_endNode = { orig(0),orig(1),0.0 };
			for (GLKPOSITION Pos = tet_Model->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)tet_Model->GetFaceList().GetNext(Pos);

				if (Face->inner) continue;	// the inner face is not considered 
				Eigen::Vector3d insertP, v1, v2, v3; //caution：omp
				Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
				Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
				Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

				//support endNode detection
				if (_intersetTriangle(orig, dir, v1, v2, v3, insertP)) {

					if (insertP(2) < orig(2)) { // make sure the intersection point is below the orig
						support_withplatform = false;
						intersect_NUM++;
						if ((orig - insertP).norm() < (orig - closest_endNode).norm()) {
							closest_endNode = insertP;
						}
						Node->supportEndPos = closest_endNode;
					}
				}
			}
			// if the intersection number is odd, the mid-Point of Ray is inside the TET shell, 
			// which should be thrown away from the ray set
			if (intersect_NUM % 2 == 1) {
				Node->need_Support = false;
			}
			if (support_withplatform == true) {
				////std::cout << "supported by platform " << endl;
				//1. -->//vertical; Node->supportEndPos << orig(0), orig(1), 0.0;
				//2. -->//normal direction
				Eigen::Vector3d n; n << 0, 0, 1;// platform plane direction
				if (fabs(n.dot(dir)) < 0.707) {// 0.707 means [45, 135]deg
					dir = (dir * 0.5 - n * 0.5).normalized();
					//std::cout << "flat the normal!" << std::endl;
				}
				double t = -n.dot(orig) / n.dot(dir);
				Node->supportEndPos = orig + t * dir;
			}
		}
	}
}

// get the support polyline for isoLayers based on curved layer
void supportGeneration::build_Support_PolyLines() {

	//get large layer set table ( i | QmeshPatch* initial | QmeshPatch* support )
	std::vector<std::vector<QMeshPatch*>> largeLayer_matrix;
	_get_largeLayer_list(largeLayer_matrix);

	/*test OK*/
	//for (int i = 0; i < largeLayer_matrix.size(); i++) {
	//	for (int j = 0; j < largeLayer_matrix[i].size(); j++) {
	//		std::cout << largeLayer_matrix[i][j]->largeLayer_Index << " ";
	//	}
	//	std::cout << "\n";
	//}
	//End

	//loop: each node on each initial iso-layer
	for (int i = 0; i < largeLayer_matrix.size(); i++) {

		if (largeLayer_matrix[i].size() < 2) continue; // base on envelope facts, less than 2 means only support layers
		QMeshPatch* top_initial_layer = largeLayer_matrix[i][0];

		//build a vector for omp speed_up
		std::vector<QMeshNode*> node_Set(top_initial_layer->GetNodeNumber());
		int temp_index = 0;
		for (GLKPOSITION Pos = top_initial_layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)top_initial_layer->GetNodeList().GetNext(Pos);

			node_Set[temp_index] = Node;
			temp_index++;
		}

		//for (GLKPOSITION Pos = top_initial_layer->GetNodeList().GetHeadPosition(); Pos;) {
		//	QMeshNode* Node = (QMeshNode*)top_initial_layer->GetNodeList().GetNext(Pos);
#pragma omp parallel
		{
#pragma omp for  
			for (int j = 0; j < node_Set.size(); j++) {
				QMeshNode* Node = node_Set[j];

				Eigen::Vector3d oringin_Node; Node->GetCoord3D(oringin_Node(0), oringin_Node(1), oringin_Node(2));
				Eigen::Vector3d step_Direction; for (int k = 0; k < 3; k++) { step_Direction[k] = Node->m_desiredNormal[k]; }
				/*********** initial the step_direction of last step ***********/
				Eigen::Vector3d last_descend_dir = { 0.0, 0.0, 0.0 };

				for (int q = (i - 1); q >= 0; q--) {

					bool decline_stepDir = true;	
					if (q == (i - 1))	decline_stepDir = false;
					bool stopFlag = _find_targetNode(oringin_Node, step_Direction, Node, largeLayer_matrix[q], decline_stepDir, last_descend_dir);
					if (stopFlag) break;					
				}
			}
		}
	}
}

// get the support polyline based on tetSurface and vector field
void supportGeneration::build_Support_Polyline_fromTETsurface() {

	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();

	//calculate the direction of support Ray; by laplacian smoothness
	_cal_Ray_direction(tet_Model);
	//get large layer set table ( i | QmeshPatch* initial | QmeshPatch* support )
	std::vector<std::vector<QMeshPatch*>> largeLayer_matrix;
	_get_largeLayer_list(largeLayer_matrix);

	// reconstruct the Node Array for omp usage
	std::vector<QMeshNode*> tet_surfaceNode_set;
	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);
		// the inner node is not considered
		if (Node->inner) continue;
		if (Node->need_Support == false) continue;

		tet_surfaceNode_set.push_back(Node);
	}
#pragma omp parallel
	{
#pragma omp for  

		for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
			QMeshNode* Node = tet_surfaceNode_set[i];

			//global variable: initial value
			Eigen::Vector3d oringinNode; Node->GetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
			Eigen::Vector3d step_Direction = Node->supportRay_Dir;
			/*********** initial the step_direction of last step ***********/
			Eigen::Vector3d last_descend_dir = { 0.0, 0.0, 0.0 };

			bool get_1st_intersectPnt = false;
			// 1. find the 1st intersection between polyline and support layer (OR tetSurface) 
			int m;// needed to record the index of the 1st bellow large layer
			for(m = largeLayer_matrix.size() - 1; m >= 0; m--){ // from top to bottom
				for (int n = 0; n < largeLayer_matrix[m].size();n++) {
					QMeshPatch* detect_layer = largeLayer_matrix[m][n];

					Eigen::Vector3d insertP, v1, v2, v3;
					for (GLKPOSITION Pos = detect_layer->GetFaceList().GetHeadPosition(); Pos;) {
						QMeshFace* Face = (QMeshFace*)detect_layer->GetFaceList().GetNext(Pos);

						/*speeding up by distance limit*/
						// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
						Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
						if ((center - oringinNode).norm() > 5) continue;

						Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
						Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
						Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));
						
						if (_intersetTriangle(oringinNode, step_Direction, v1, v2, v3, insertP)) { //intersect Yes
							/* calculate t +/-? : insertP = orig + t * dir; */
							if ((insertP - oringinNode)[0] / step_Direction[0] > 0.01) { // next layer along the + direction of vectorField
								get_1st_intersectPnt = true;
								if (detect_layer->is_SupportLayer) { // only record the support layer intersection
									Node->polyline_node.push_back(insertP);
									oringinNode = insertP;
									for (int i = 0; i < 3; i++) { step_Direction[i] = Face->m_desiredNormal[i]; }
								}
								break;
							}
						}
					}
					if (get_1st_intersectPnt)break;
				}
				if (get_1st_intersectPnt)break;
			}

			// 2. iteratively find the drop points
			for (int q = (m - 1); q >= 0; q--) {

				bool decline_stepDir = true;
				bool stopFlag = _find_targetNode(oringinNode, step_Direction, Node, largeLayer_matrix[q], decline_stepDir, last_descend_dir);
				if (stopFlag) break;
			}

		}
	}
}

// get the support tree  based on tetSurface and vector fileld
void supportGeneration::build_Support_Tree_fromTETsurface(){
	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();

	//calculate the direction of support Ray; by laplacian smoothness
	_cal_Ray_direction(tet_Model);
	//get large layer set table ( i | QmeshPatch* initial | QmeshPatch* support )
	std::vector<std::vector<QMeshPatch*>> largeLayer_matrix;
	_get_largeLayer_list(largeLayer_matrix);

	// reconstruct the Node Array for omp usage; as Face write the treeNode, omp cannot be used
	std::vector<QMeshNode*> tet_surfaceNode_set;
	int host_NodeNum = 0;
	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);
		// the inner node is not considered
		if (Node->inner) continue;
		if (Node->need_Support == false) continue;
		//test sparse polyline trace
		if (
			//yoga
			Node->GetIndexNo() == 5959
			|| Node->GetIndexNo() == 5007
			|| Node->GetIndexNo() == 5401
			|| Node->GetIndexNo() == 8164
			|| Node->GetIndexNo() == 2482
			//bunny
			//Node->GetIndexNo() == 5789
			//|| Node->GetIndexNo() == 6672
			//|| Node->GetIndexNo() == 934
			//topo
			//Node->GetIndexNo() == 3617
			//|| Node->GetIndexNo() == 14481
			//|| Node->GetIndexNo() == 3132
			//|| Node->GetIndexNo() == 9593
			){
			Node->isHostNode = true;
			host_NodeNum++;
		}
		tet_surfaceNode_set.push_back(Node);
	}
	/*tianyu 1207 - next step: find suitable skeleton/host/seed Node: interactive pick or clustering algorithm*/
	std::cout << "skeleton_NodeNum = " << host_NodeNum << std::endl;

	// 1. find the 1st drop point for Node (needs support) on TET surface
#pragma omp parallel
	{
#pragma omp for 
		for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
			QMeshNode* Node = tet_surfaceNode_set[i];

			//global variable: initial value
			Eigen::Vector3d oringinNode; Node->GetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
			Eigen::Vector3d step_Direction = Node->supportRay_Dir; //from tet surface normal

			bool get_1st_intersectPnt = false;
			// 1. find the 1st intersection between polyline and support layer (OR tetSurface) 
			for (int m = largeLayer_matrix.size() - 1; m >= 0; m--) { // from top to bottom
				for (int n = 0; n < largeLayer_matrix[m].size(); n++) {
					QMeshPatch* detect_layer = largeLayer_matrix[m][n];

					Eigen::Vector3d insertP, v1, v2, v3;
					for (GLKPOSITION Pos = detect_layer->GetFaceList().GetHeadPosition(); Pos;) {
						QMeshFace* Face = (QMeshFace*)detect_layer->GetFaceList().GetNext(Pos);

						/*speeding up by distance limit*/
						// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
						Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
						if ((center - oringinNode).norm() > 5) continue;

						Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
						Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
						Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

						if (_intersetTriangle(oringinNode, step_Direction, v1, v2, v3, insertP)) {	/*intersect Yes*/
							/* calculate t +/-? : insertP = orig + t * dir; */
							if ((insertP - oringinNode)[0] / step_Direction[0] > 0.01) {			/*next layer along the "+" direction of vectorField*/
								get_1st_intersectPnt = true;
								if (detect_layer->is_SupportLayer) {								/*only record the support layer intersection*/

									//record the info (of treeNode) for source Node
									Node->polyline_node.push_back(insertP);// continue use the polyline_node for Function reuse
									//record the info for face who includes sptTreeNode(s)
									SptTreeNode stNode;
									stNode.souce_Node = Node;
									stNode.treeNode_descend_Nor = step_Direction; //-->node
									stNode.treeNode_coord3D = insertP;
									if (Node->isHostNode) { stNode.isHostNode = true; }
									else { stNode.isHostNode = false; }
									stNode.isProcessed = true;
									Face->support_treeNode_cell.push_back(stNode);
								}
								break;
							}
						}
					}
					if (get_1st_intersectPnt)break;
				}
				if (get_1st_intersectPnt)break;
			}

			//std::cout << "step 1: node : " << i << std::endl;
		}
	}
	std::cout << "Finish: 1. find the 1st drop point for Node (needs support) on TET surface." << std::endl;

	// 2. find the drop points for host node (layer by layer) --> polyline downward
	for (int m = largeLayer_matrix.size() - 1; m > 0; m--) { // from top to bottom
		for (int n = 0; n < largeLayer_matrix[m].size(); n++) {
			QMeshPatch* detected_support_layer = largeLayer_matrix[m][n];

			if (detected_support_layer->is_SupportLayer == false) continue;

			//top layer face which contains oringin_Node
			for (GLKPOSITION Pos = detected_support_layer->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)detected_support_layer->GetFaceList().GetNext(Pos);

				if (Face->support_treeNode_cell.size() != 0) {
					for (int i = 0; i < Face->support_treeNode_cell.size(); i++) {
						if (Face->support_treeNode_cell[i].isHostNode) {

							Eigen::Vector3d step_Direction = { 0.0,0.0,-1.0 };
							_get_step_direction_DownWard_OR_center(Face, Face->support_treeNode_cell[i].treeNode_coord3D, 
								step_Direction,false);

							Eigen::Vector3d nearest_hostNode_coord3D = { -1.0,-1.0,-1.0 };
							bool stopFlag = _find_targetNode(
								Face->support_treeNode_cell[i].treeNode_coord3D,
								step_Direction,
								Face->support_treeNode_cell[i].souce_Node,
								largeLayer_matrix[m-1],// 1st large layer below
								nearest_hostNode_coord3D,
								true);
							if (stopFlag) break;
						}
					}
				}
			}
		}
		//std::cout << "step 2: large layer : " << m << std::endl;
	}

	std::cout << "Finish: 2. find the drop points for host node (layer by layer) --> polyline downward." << std::endl;

	// 3. find the drop points for slave node (layer by layer) --> polyline tree-like
	for (int m = largeLayer_matrix.size() - 1; m > 0; m--) { // from top to bottom
		for (int n = 0; n < largeLayer_matrix[m].size(); n++) {
			QMeshPatch* detected_support_layer = largeLayer_matrix[m][n];

			// skip initial layers in top large layer
			if (detected_support_layer->is_SupportLayer == false) continue;


			//top layer face which contains oringin_Node
			for (GLKPOSITION Pos = detected_support_layer->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)detected_support_layer->GetFaceList().GetNext(Pos);

				if (Face->support_treeNode_cell.size() != 0) {
					for (int i = 0; i < Face->support_treeNode_cell.size(); i++) {
						//get slave node (oringin_Node)
						if (Face->support_treeNode_cell[i].isHostNode == false) {

							std::vector<SptTreeNode> hostNode_1st_LB_Set;
							//bool is_goto_center = _get_hostNode_on_1st_largeLayer_below(hostNode_1st_LB_Set, host_NodeNum, largeLayer_matrix[m - 1]);

							//MODE: 0 - find center; 1 - find nearest hostNode; 2; along before dir
							int dir_find_MODE = _get_hostNode_on_1st_largeLayer_below(hostNode_1st_LB_Set, host_NodeNum, largeLayer_matrix[m - 1]);
							if (dir_find_MODE == -1) std::cout << "dir_find_MODE ERROR! " << std::endl;

							//test for yoga(40)
							//if (detected_support_layer->GetIndexNo() < 40) dir_find_MODE = 2;

							Eigen::Vector3d step_Direction = { 0.0,0.0,-1.0 };
							int nearest_hostNode_index = _get_step_direction_tree(Face, Face->support_treeNode_cell[i].treeNode_coord3D,
								Face->support_treeNode_cell[i].treeNode_descend_Nor, step_Direction, dir_find_MODE, hostNode_1st_LB_Set);

							Eigen::Vector3d nearest_hostNode_coord3D = { -1.0,-1.0,-1.0 };
							if (dir_find_MODE == 1) {
								nearest_hostNode_coord3D = hostNode_1st_LB_Set[nearest_hostNode_index].treeNode_coord3D;
							}

							bool stopFlag = _find_targetNode(
								Face->support_treeNode_cell[i].treeNode_coord3D,
								step_Direction,
								Face->support_treeNode_cell[i].souce_Node,
								largeLayer_matrix[m - 1],// 1st large layer below
								nearest_hostNode_coord3D,
								dir_find_MODE);
							if (stopFlag) break;
						}
					}
				}
			}
		}
		//std::cout << "step 3: large layer : " << m << std::endl;
	}

	std::cout << "Finish: 3. find the drop points for slave node (layer by layer) --> polyline tree-like." << std::endl;


	//for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
	//	QMeshNode* Node = tet_surfaceNode_set[i];
	//	if (Node->isHostNode) {
	//		for (int q = (m - 1); q >= 0; q--) {
	//
	//			bool decline_stepDir = true;
	//			bool stopFlag = _find_targetNode(oringinNode, step_Direction, Node, largeLayer_matrix[q], decline_stepDir, last_descend_dir);
	//			if (stopFlag) break;
	//		}
	//	}
	//}
	//
	// 2. find the tree node layer by layer (find + merge)
	//for (int m = largeLayer_matrix.size() - 1; m >= 0; m--) { // from top to bottom
	//	for (int n = 0; n < largeLayer_matrix[m].size(); n++) {
	//		QMeshPatch* detected_support_layer = largeLayer_matrix[m][n];
	//
	//		// skip initial layers
	//		if (detected_support_layer->is_SupportLayer == false) continue;
	//		// skip support layers without tree node
	//		double is_treeNode_On_layer = false;
	//		for (GLKPOSITION Pos = detected_support_layer->GetFaceList().GetHeadPosition(); Pos;) {
	//			QMeshFace* Face = (QMeshFace*)detected_support_layer->GetFaceList().GetNext(Pos);
	//
	//			if (Face->support_treeNode_cell.size() != 0) {
	//				is_treeNode_On_layer = true;
	//				break;
	//			}
	//		}
	//		if (is_treeNode_On_layer == false) continue;
	//
	//		if (m != 0) {
	//			//find target node on below large layer
	//			bool stopFlag = _find_targetNode(detected_support_layer, largeLayer_matrix[m - 1]);
	//			if (stopFlag) break;
	//		}
	//		else {
	//			// the bottomest layer
	//			// add the treeNode on the platform
	//		}
	//		std::cout << "large layer contain tree node ind = " << m << std::endl;
	//	}
	//}	
	
}

void supportGeneration::build_Support_Tree_fromTETsurface2() {

	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();

	//calculate the direction of support Ray; by laplacian smoothness
	_cal_Ray_direction(tet_Model);
	//get large layer set table ( i | QmeshPatch* initial | QmeshPatch* support )
	std::vector<std::vector<QMeshPatch*>> largeLayer_matrix;
	_get_largeLayer_list(largeLayer_matrix);

	// reconstruct the Node Array for omp usage; as Face write the treeNode, omp cannot be used
	std::vector<QMeshNode*> tet_surfaceNode_set;
	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);
		// the inner node is not considered
		if (Node->inner) continue;
		if (Node->need_Support == false) continue;
		
		tet_surfaceNode_set.push_back(Node);
	}
	std::cout << "tree_Node_Num = " << tet_surfaceNode_set.size() << std::endl;

	// 1. find the 1st drop point for Node (needs support) on TET surface
#pragma omp parallel
	{
#pragma omp for 
		for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
			QMeshNode* Node = tet_surfaceNode_set[i];

			//global variable: initial value
			Eigen::Vector3d oringinNode; Node->GetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
			Eigen::Vector3d step_Direction = Node->supportRay_Dir; //from tet surface normal

			bool get_1st_intersectPnt = false;
			// 1. find the 1st intersection between polyline and support layer (OR tetSurface)
			for (int m = largeLayer_matrix.size() - 1; m >= 0; m--) { // from top to bottom
				for (int n = 0; n < largeLayer_matrix[m].size(); n++) {
					QMeshPatch* detect_layer = largeLayer_matrix[m][n];

					Eigen::Vector3d insertP, v1, v2, v3;
					for (GLKPOSITION Pos = detect_layer->GetFaceList().GetHeadPosition(); Pos;) {
						QMeshFace* Face = (QMeshFace*)detect_layer->GetFaceList().GetNext(Pos);

						/*speeding up by distance limit*/
						// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
						Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
						if ((center - oringinNode).norm() > 5) continue;

						Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
						Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
						Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

						if (_intersetTriangle(oringinNode, step_Direction, v1, v2, v3, insertP)) {	/*intersect Yes*/
							/* calculate t +/-? : insertP = orig + t * dir; */
							if ((insertP - oringinNode)[0] / step_Direction[0] > 0.01) {			/*next layer along the "+" direction of vectorField*/
								get_1st_intersectPnt = true;
								if (detect_layer->is_SupportLayer) {								/*only record the support layer intersection*/

									//record the info (of treeNode) for source Node
									Node->polyline_node.push_back(insertP);// continue use the polyline_node for Function reuse
									Node->polyline_node_weight.push_back(1);// the 1st polyline give 1 as weight

									//record the info for face who includes sptTreeNode(s)
									SptTreeNode stNode;
									stNode.souce_Node = Node;
									stNode.treeNode_descend_Nor = step_Direction; //-->node
									stNode.treeNode_coord3D = insertP;
									// the transfer from TET surface to nearest layer, and consider these Node as host Node firstly
									stNode.isHostNode = true; 
									stNode.isProcessed = false;
									stNode.belong_Face_Normal << Face->m_desiredNormal[0], Face->m_desiredNormal[1], Face->m_desiredNormal[2];
									Face->support_treeNode_cell.push_back(stNode);
								}
								break;
							}
						}
					}
					if (get_1st_intersectPnt)break;
				}
				if (get_1st_intersectPnt)break;
			}
		}
	}
	std::cout << "Finish: 1. find the 1st drop point for Node (needs support) on TET surface." << std::endl;

	// 2. tree-like structure generation: dynamic host node
	for (int m = largeLayer_matrix.size() - 1; m > 0; m--) { // from top to bottom
		for (int n = 0; n < largeLayer_matrix[m].size(); n++) {
			QMeshPatch* detected_support_layer = largeLayer_matrix[m][n];
			if (detected_support_layer->is_SupportLayer == false) continue;

			//faces on top layer which contain treeNode
			for (GLKPOSITION Pos = detected_support_layer->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)detected_support_layer->GetFaceList().GetNext(Pos);

				// skip when no treeNode on Face or no unprocessed host treeNode on Face
				if (Face->support_treeNode_cell.size() == 0)  continue;
				else {
					// the 1st treeNode is hostNode
					if (Face->support_treeNode_cell[0].isProcessed)  continue;
				}

				//std::printf("\n^-^: this face own tree node &&/|| its neighbor:\n");
				//std::cout << "face include : " << Face->support_treeNode_cell.size() << " tree node(s)" << std::endl;

				// compute drop node in bag:nodeSet_hostORfollower_1ring - useless
				Eigen::Vector3d step_Direction_hostNode = { 0.0,0.0,-1.0 };
				Eigen::Vector3d step_Direction_followNode = { 0.0,0.0,-1.0 };
				Eigen::Vector3d drop_Node_host = { 0.0,0.0,0.0 };

				// treeNode on the Face
				for (int i = 0; i < Face->support_treeNode_cell.size(); i++) {

					Face->support_treeNode_cell[i].isHostNode = false;
					if (i == 0) Face->support_treeNode_cell[i].isHostNode = true;

					if (i == 0) {
						// 1st host node on the Face
						if ((Face->support_treeNode_cell[i].isHostNode)) {
							_get_step_direction_DownWard_OR_center(
								Face, 
								Face->support_treeNode_cell[i].treeNode_coord3D, 
								step_Direction_hostNode, 
								false);

							_get_dropNode_host_treeStructure(
								drop_Node_host,
								Face->support_treeNode_cell[i].treeNode_coord3D,
								step_Direction_hostNode,
								Face->support_treeNode_cell[i].souce_Node,
								largeLayer_matrix[m - 1],
								false);

							Face->support_treeNode_cell[i].isProcessed = true;
						}
						else {
							std::cout << "Error: the 1st treeNode should be HostNode. " << std::endl;
						}
					}
					else {
						// follower node on same Face
						_get_step_direction_dropNode(
							step_Direction_followNode, 
							Face->support_treeNode_cell[i].belong_Face_Normal,
							Face->support_treeNode_cell[i].treeNode_coord3D, 
							drop_Node_host);
							
						_get_dropNode_follow_treeStructure(
							drop_Node_host,
							Face->support_treeNode_cell[i].treeNode_coord3D,
							step_Direction_followNode,
							Face->support_treeNode_cell[i].souce_Node,
							largeLayer_matrix[m - 1],
							Face->support_treeNode_cell[0].souce_Node);
							
						Face->support_treeNode_cell[i].isProcessed = true;
					}
				}

				/***************** Add 1-ring neighbor into faceSet *****************/
				std::vector<QMeshFace*> faceSet_1ring;				// treeNode on the 1ring neighbor of Face
				// --> collect 1-ring face
				for (int i_node = 0; i_node < 3; i_node++) {

					for (GLKPOSITION Pos_1ringFace = Face->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_1ringFace;) {
						QMeshFace* Face_1ring = (QMeshFace*)(Face->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_1ringFace));

						if (Face_1ring == Face) continue; // not include self

						//1-ring face may repeat, avoid repeating
						bool is_exist = false;
						for (int j = 0; j < faceSet_1ring.size(); j++) {
							if (faceSet_1ring[j] == Face_1ring) {
								is_exist = true;
								break;
							}
						}
						if (is_exist == false) { faceSet_1ring.push_back(Face_1ring); }
					}
				}
				//std::cout << "faceSet_1ring Num : " << faceSet_1ring.size();

				/***************** Add 2-ring neighbor into faceSet *****************/
				// collect more neighbor treeNode
				// get 1-ring treeNode NUM
				int NUM_1ring_treeNode = 0;
				for (int i_face = 0; i_face < faceSet_1ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_1ring[i_face]->support_treeNode_cell.size(); j_node++) {
						if (faceSet_1ring[i_face]->support_treeNode_cell[j_node].isProcessed == false) {
							NUM_1ring_treeNode++;
						}
					}
				}
				//std::cout << "NUM_1ring_treeNode: " << NUM_1ring_treeNode << std::endl;
				int faceSet_1ring_Num = faceSet_1ring.size();// record the num of 1-ring faces (faceSet_1ring.size() will change after add 2-ring Faces)
				if (NUM_1ring_treeNode == 0) {

					for (int index_1ring_face = 0; index_1ring_face < faceSet_1ring_Num; index_1ring_face++) {
						for (int i_node = 0; i_node < 3; i_node++) {

							for (GLKPOSITION Pos_2ringFace = faceSet_1ring[index_1ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_2ringFace;) {
								QMeshFace* Face_2ring = (QMeshFace*)(faceSet_1ring[index_1ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_2ringFace));

								if (Face_2ring == Face) continue; // not include self

								//2-ring face may repeat, avoid repeating
								bool is_exist = false;
								for (int j = 0; j < faceSet_1ring.size(); j++) {
									if (faceSet_1ring[j] == Face_2ring) {
										is_exist = true;
										break;
									}
								}
								if (is_exist == false) { faceSet_1ring.push_back(Face_2ring); }
							}
						}
					}
					//std::cout << " faceSet_2ring Num : " << faceSet_1ring.size();
				}

				/***************** Add 3-ring neighbor into faceSet *****************/
				// collect more neighbor treeNode
				// get 2-ring treeNode NUM
				int NUM_2ring_treeNode = 0;
				for (int i_face = 0; i_face < faceSet_1ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_1ring[i_face]->support_treeNode_cell.size(); j_node++) {
						if (faceSet_1ring[i_face]->support_treeNode_cell[j_node].isProcessed == false) {
							NUM_2ring_treeNode++;
						}
					}
				}
				//std::cout << "NUM_2ring_treeNode: " << NUM_2ring_treeNode << std::endl;
				int faceSet_2ring_Num = faceSet_1ring.size();// record the num of 2-ring faces (faceSet_1ring.size() will change after add 2-ring Faces)
				if (NUM_2ring_treeNode == 0) {

					for (int index_2ring_face = faceSet_1ring_Num; index_2ring_face < faceSet_2ring_Num; index_2ring_face++) {
						for (int i_node = 0; i_node < 3; i_node++) {

							for (GLKPOSITION Pos_3ringFace = faceSet_1ring[index_2ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_3ringFace;) {
								QMeshFace* Face_3ring = (QMeshFace*)(faceSet_1ring[index_2ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_3ringFace));

								if (Face_3ring == Face) continue; // not include self

								//3-ring face may repeat, avoid repeating
								bool is_exist = false;
								for (int j = 0; j < faceSet_1ring.size(); j++) {
									if (faceSet_1ring[j] == Face_3ring) {
										is_exist = true;
										break;
									}
								}
								if (is_exist == false) { faceSet_1ring.push_back(Face_3ring); }
							}
						}
					}
					//std::cout << " faceSet_3ring Num : " << faceSet_1ring.size();
				}
				//std::cout << std::endl;
				/******************************* END ********************************/

				// --> compute dropNode for 1/2-ring treeNode (still collected in the "faceSet_1ring")
				for (int i_face = 0; i_face < faceSet_1ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_1ring[i_face]->support_treeNode_cell.size(); j_node++) {

						if (faceSet_1ring[i_face]->support_treeNode_cell[j_node].isProcessed == false) {

							// 1ring neighbor are all follower node
							faceSet_1ring[i_face]->support_treeNode_cell[j_node].isHostNode = false;
							
							_get_step_direction_dropNode(
								step_Direction_followNode, 
								faceSet_1ring[i_face]->support_treeNode_cell[j_node].belong_Face_Normal,
								faceSet_1ring[i_face]->support_treeNode_cell[j_node].treeNode_coord3D,
								drop_Node_host);
								
							_get_dropNode_follow_treeStructure(
								drop_Node_host,
								faceSet_1ring[i_face]->support_treeNode_cell[j_node].treeNode_coord3D,
								step_Direction_followNode,
								faceSet_1ring[i_face]->support_treeNode_cell[j_node].souce_Node,
								largeLayer_matrix[m - 1],
								Face->support_treeNode_cell[0].souce_Node);
								
							faceSet_1ring[i_face]->support_treeNode_cell[j_node].isProcessed = true;
						}
					}
				}	
			}
		}
		std::cout <<".";
		if ((largeLayer_matrix.size() - m) % 100 == 0 || (m == 1)) std::cout << std::endl;
	}
	std::cout << "Finish: 2. find the drop points for tree node (layer by layer) --> tree-like." << std::endl;
}

void supportGeneration::build_Support_Tree_fromTETsurface3() {

	//build a new patch for organizing support Tree
	QMeshPatch* supportRay_patch = new QMeshPatch;
	supportRay_patch->SetIndexNo(m_supportRaySet->GetMeshList().GetCount()); //index begin from 0
	m_supportRaySet->GetMeshList().AddTail(supportRay_patch);
	supportRay_patch->drawThisPatch = true;

	//get Tet Model patch
	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();

	//calculate the direction of support Ray; by laplacian smoothness
	_cal_Ray_direction(tet_Model);
	//get large layer set table ( i | QmeshPatch* initial | QmeshPatch* support )
	std::vector<std::vector<QMeshPatch*>> largeLayer_matrix;
	_get_largeLayer_list(largeLayer_matrix);

	// reconstruct the Node Array for omp usage; as Face write the treeNode, omp cannot be used
	std::vector<QMeshNode*> tet_surfaceNode_set;
	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);
		// the inner node is not considered
		if (Node->inner) continue;
		if (Node->need_Support == false) continue;

		tet_surfaceNode_set.push_back(Node);
	}
	std::cout << "tree_Node_Num = " << tet_surfaceNode_set.size() << std::endl;

	// 1. find the 1st drop point for Node (needs support) on TET surface
#pragma omp parallel
	{
#pragma omp for 
		for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
			QMeshNode* Node = tet_surfaceNode_set[i];

			//global variable: initial value
			Eigen::Vector3d oringinNode; Node->GetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
			Eigen::Vector3d step_Direction = Node->supportRay_Dir; //from tet surface normal

			bool get_1st_intersectPnt = false;
			// 1. find the 1st intersection between polyline and support layer (OR tetSurface)
			for (int m = largeLayer_matrix.size() - 1; m >= 0; m--) { // from top to bottom
				for (int n = 0; n < largeLayer_matrix[m].size(); n++) {
					QMeshPatch* detect_layer = largeLayer_matrix[m][n];

					Eigen::Vector3d insertP, v1, v2, v3;
					for (GLKPOSITION Pos = detect_layer->GetFaceList().GetHeadPosition(); Pos;) {
						QMeshFace* Face = (QMeshFace*)detect_layer->GetFaceList().GetNext(Pos);

						/*speeding up by distance limit*/
						// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
						Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
						if ((center - oringinNode).norm() > 5) continue;

						Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
						Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
						Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

						if (_intersetTriangle(oringinNode, step_Direction, v1, v2, v3, insertP)) {	/*intersect Yes*/
							/* calculate t +/-? : insertP = orig + t * dir; */
							if ((insertP - oringinNode)[0] / step_Direction[0] > 0.01) {			/*next layer along the "+" direction of vectorField*/
								get_1st_intersectPnt = true;
								if (detect_layer->is_SupportLayer) {								/*only record the support layer intersection*/

									//create souceNode
									QMeshNode* sourceNode = new QMeshNode;
									sourceNode->is_Processed = true;
									sourceNode->SetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
									sourceNode->SetMeshPatchPtr(supportRay_patch);
									sourceNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
									supportRay_patch->GetNodeList().AddTail(sourceNode);

									//create one TreeNode
									QMeshNode* TreeNode = new QMeshNode;
									TreeNode->SetCoord3D(insertP[0], insertP[1], insertP[2]);
									TreeNode->is_Host = true;
									TreeNode->is_Processed = false;
									TreeNode->treeNode_belonging_Face = Face;
									TreeNode->descend_To_TreeNode_Dir = step_Direction;
									TreeNode->treeNode_before_branch_NUM = 1;
									TreeNode->SetMeshPatchPtr(supportRay_patch);
									TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
									supportRay_patch->GetNodeList().AddTail(TreeNode);
									Face->support_treeNode_onFace.push_back(TreeNode);

									//create one TreeEdge
									QMeshEdge* TreeEdge = new QMeshEdge;
									TreeEdge->treeEdge_height = m;
									TreeEdge->treeEdge_branch_NUM = 1;
									TreeEdge->SetStartPoint(sourceNode);
									TreeEdge->SetEndPoint(TreeNode);
									TreeEdge->SetMeshPatchPtr(supportRay_patch);
									TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
									supportRay_patch->GetEdgeList().AddTail(TreeEdge);
								}
								break;
							}
						}
					}
					if (get_1st_intersectPnt)break;
				}
				if (get_1st_intersectPnt)break;
			}
		}
	}
	std::cout << "Finish: 1. find the 1st drop point for Node (needs support) on TET surface." << std::endl;

	// 2. tree-like structure generation: dynamic host node
	for (int m = largeLayer_matrix.size() - 1; m > 0; m--) { // from top to bottom
		for (int n = 0; n < largeLayer_matrix[m].size(); n++) {
			QMeshPatch* detected_support_layer = largeLayer_matrix[m][n];
			if (detected_support_layer->is_SupportLayer == false)	continue;

			//faces on TOP layer which contain treeNode
			for (GLKPOSITION Pos = detected_support_layer->GetFaceList().GetHeadPosition(); Pos;) {
				QMeshFace* Face = (QMeshFace*)detected_support_layer->GetFaceList().GetNext(Pos);

				// skip when no treeNode on Face or all treeNode are processed on Face
				if (Face->support_treeNode_onFace.size() == 0)	continue;
				else {
					bool all_processed = true;
					for (int s = 0; s < Face->support_treeNode_onFace.size(); s++) {
						if (Face->support_treeNode_onFace[s]->is_Processed == false) {
							all_processed = false;
							break;
						}
					}
					if (all_processed)	continue;
				}

				/*Main Function of tree generation*/
				// treeNode on the same Face
				// --> find the host treeNode index
				int treeNode_index = -1;// the tree node with the most large of branch_NUM is defined as "hostNode"
				int max_treeNode_NUM = -1;
				for (int i = 0; i < Face->support_treeNode_onFace.size(); i++) {
					if (Face->support_treeNode_onFace[i]->treeNode_before_branch_NUM > max_treeNode_NUM)
						max_treeNode_NUM = Face->support_treeNode_onFace[i]->treeNode_before_branch_NUM;
					treeNode_index = i;
				}
				if (treeNode_index == -1 || max_treeNode_NUM == -1) // for safe
					std::cout << "Error: cannot find the treeNode with max branch NUM!" << std::endl;

				//Debug
				//treeNode_index = 0;
				//END

				// --> find the dropNode for hostNode
				QMeshNode* hostNode = Face->support_treeNode_onFace[treeNode_index];
				_compute_descend_Dir_hostNode(hostNode, false);//zty edit 2021/3/28
				QMeshNode* dropNode_hostNode = _build_host_treeNodeEdge(hostNode, largeLayer_matrix[m - 1], supportRay_patch, (m - 1));
				hostNode->is_Host = true;
				if (dropNode_hostNode == NULL) std::cout << "The dropNode of hostNode should not be NULL!" << std::endl;

				// --> find the dropNode for followNode on the same face;
				for (int i = 0; i < Face->support_treeNode_onFace.size(); i++) {
					if (i == treeNode_index)	continue;
					QMeshNode* followNode = Face->support_treeNode_onFace[i];
					followNode->is_Host = false;
					bool merge = _compute_descend_Dir_followNode(followNode, dropNode_hostNode);
					_build_follow_treeNodeEdge(followNode, dropNode_hostNode, largeLayer_matrix[m - 1], merge, supportRay_patch, (m - 1));
				}

				///***************** Add 1-ring neighbor into faceSet *****************/
				std::vector<QMeshFace*> faceSet_ring;
				// --> collect 1-ring face
				for (int i_node = 0; i_node < 3; i_node++) {
					for (GLKPOSITION Pos_1ringFace = Face->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_1ringFace;) {
						QMeshFace* Face_1ring = (QMeshFace*)(Face->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_1ringFace));

						if (Face_1ring == Face) continue; // not include self

						//1-ring face may repeat, avoid repeating
						bool is_exist = false;
						for (int j = 0; j < faceSet_ring.size(); j++) {
							if (faceSet_ring[j] == Face_1ring) {
								is_exist = true;
								break;
							}
						}
						if (is_exist == false) { faceSet_ring.push_back(Face_1ring); }
					}
				}
				//std::cout << "Face index: " << Face->GetIndexNo(); //right
				//std::cout << " faceSet_1ring Num : " << faceSet_ring.size() << std::endl; // right
				/***************** Add 2-ring neighbor into faceSet *****************/
				// --> collect 2-ring face
				int NUM_1ring_treeNode = 0;
				for (int i_face = 0; i_face < faceSet_ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_ring[i_face]->support_treeNode_cell.size(); j_node++) {
						if (faceSet_ring[i_face]->support_treeNode_cell[j_node].isProcessed == false) {
							NUM_1ring_treeNode++;
						}
					}
				}
				//std::cout << "NUM_1ring_treeNode: " << NUM_1ring_treeNode << std::endl;
				int faceSet_1ring_Num = faceSet_ring.size();// record the num of 1-ring faces
				if (NUM_1ring_treeNode == 0) {

					for (int index_1ring_face = 0; index_1ring_face < faceSet_1ring_Num; index_1ring_face++) {
						for (int i_node = 0; i_node < 3; i_node++) {

							for (GLKPOSITION Pos_2ringFace = faceSet_ring[index_1ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_2ringFace;) {
								QMeshFace* Face_2ring = (QMeshFace*)(faceSet_ring[index_1ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_2ringFace));

								if (Face_2ring == Face) continue; // not include self

								//2-ring face may repeat, avoid repeating
								bool is_exist = false;
								for (int j = 0; j < faceSet_ring.size(); j++) {
									if (faceSet_ring[j] == Face_2ring) {
										is_exist = true;
										break;
									}
								}
								if (is_exist == false) { faceSet_ring.push_back(Face_2ring); }
							}
						}
					}
					//std::cout << " faceSet_2ring Num : " << faceSet_1ring.size();
				}
				/***************** Add 3-ring neighbor into faceSet *****************/
				// --> collect 3-ring face
				int NUM_2ring_treeNode = 0;
				for (int i_face = 0; i_face < faceSet_ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_ring[i_face]->support_treeNode_cell.size(); j_node++) {
						if (faceSet_ring[i_face]->support_treeNode_cell[j_node].isProcessed == false) {
							NUM_2ring_treeNode++;
						}
					}
				}
				//std::cout << "NUM_2ring_treeNode: " << NUM_2ring_treeNode << std::endl;
				int faceSet_2ring_Num = faceSet_ring.size();// record the num of 2-ring faces (faceSet_1ring.size() will change after add 2-ring Faces)
				if (NUM_2ring_treeNode == 0) {
								
					for (int index_2ring_face = faceSet_1ring_Num; index_2ring_face < faceSet_2ring_Num; index_2ring_face++) {
						for (int i_node = 0; i_node < 3; i_node++) {
								
							for (GLKPOSITION Pos_3ringFace = faceSet_ring[index_2ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetHeadPosition(); Pos_3ringFace;) {
								QMeshFace* Face_3ring = (QMeshFace*)(faceSet_ring[index_2ring_face]->GetNodeRecordPtr(i_node)->GetFaceList().GetNext(Pos_3ringFace));
								
								if (Face_3ring == Face) continue; // not include self
								
								//3-ring face may repeat, avoid repeating
								bool is_exist = false;
								for (int j = 0; j < faceSet_ring.size(); j++) {
									if (faceSet_ring[j] == Face_3ring) {
										is_exist = true;
										break;
									}
								}
								if (is_exist == false) { faceSet_ring.push_back(Face_3ring); }
							}
						}
					}
					//std::cout << " faceSet_3ring Num : " << faceSet_1ring.size();
				}
				//std::cout << std::endl;
				
				/******************************* END ********************************/

				// --> compute dropNode for 1/2/3-ring treeNode
				for (int i_face = 0; i_face < faceSet_ring.size(); i_face++) {
					for (int j_node = 0; j_node < faceSet_ring[i_face]->support_treeNode_onFace.size(); j_node++) {

						QMeshNode* followNode_ring = faceSet_ring[i_face]->support_treeNode_onFace[j_node];

						if (followNode_ring->is_Processed == false) {
							followNode_ring->is_Host = false;// 1/2/3-ring neighbor are all followNode
							bool merge = _compute_descend_Dir_followNode(followNode_ring, dropNode_hostNode);
							_build_follow_treeNodeEdge(followNode_ring, dropNode_hostNode, largeLayer_matrix[m - 1], merge, supportRay_patch, (m - 1));
						}
					}
				}
			}
		}
		std::cout << ".";
		if ((largeLayer_matrix.size() - m) % 100 == 0 || (m == 1)) std::cout << std::endl;
	}

	//get the max branch NUM of treeEdge
	int max_branch_NUM = -1;
	for (GLKPOSITION Pos = supportRay_patch->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
		QMeshEdge* edge = (QMeshEdge*)(supportRay_patch->GetEdgeList().GetNext(Pos));

		if (edge->treeEdge_branch_NUM > max_branch_NUM)	max_branch_NUM = edge->treeEdge_branch_NUM;
	}
	std::cout << "max branch NUM = " << max_branch_NUM << std::endl;
	supportRay_patch->max_branch_NUM = max_branch_NUM;

	//get the max height of treeEdge
	int max_height = -1;
	for (GLKPOSITION Pos = supportRay_patch->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
		QMeshEdge* edge = (QMeshEdge*)(supportRay_patch->GetEdgeList().GetNext(Pos));

		if (edge->treeEdge_height > max_height)	max_height = edge->treeEdge_height;
	}
	std::cout << "max height = " << max_height << std::endl;
	supportRay_patch->max_height = max_height;

	std::cout << "treeEdge NUM = " << supportRay_patch->GetEdgeNumber() << std::endl;
	std::cout << "Finish: 2. find the drop points for tree node (layer by layer) --> tree-like." << std::endl;
}

void supportGeneration::_compute_descend_Dir_hostNode(QMeshNode* hostNode, bool is_downWard) {

	//incline a self-supporting angle to the z direction
	Eigen::Vector3d descend_dir = { 0.0,0.0,-1.0 };
	if (!is_downWard) {
		hostNode->GetCoord3D(descend_dir[0], descend_dir[1], descend_dir[2]);
		descend_dir = -descend_dir.normalized();
	}
	Eigen::Vector3d face_normal; face_normal << 
		hostNode->treeNode_belonging_Face->m_desiredNormal[0], 
		hostNode->treeNode_belonging_Face->m_desiredNormal[1],
		hostNode->treeNode_belonging_Face->m_desiredNormal[2];
	Eigen::Vector3d rotateAxis = face_normal.cross(descend_dir);

	//angle between _dir and _descend_dir [0,PI]
	double radian_angle = atan2(face_normal.cross(descend_dir).norm(), face_normal.transpose() * descend_dir);
	if (ROTATE_TO_DEGREE(radian_angle) < tau) 
		hostNode->descend_From_TreeNode_Dir = descend_dir; // in the range of support angle, directly downward
	else {
		// only rotate a support angle
		Eigen::AngleAxisd V(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis，rotate tau(deg)
		hostNode->descend_From_TreeNode_Dir = V * face_normal;
	}
}

QMeshNode* supportGeneration::_build_host_treeNodeEdge(
	QMeshNode* hostNode, 
	const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
	QMeshPatch* supportRay_patch,
	int layer_height) {

	bool isIntersect = false;
	Eigen::Vector3d treeNode_coord3D, step_Direction_hostNode;
	QMeshNode* host_dropNode = NULL;

	hostNode->GetCoord3D(treeNode_coord3D[0], treeNode_coord3D[1], treeNode_coord3D[2]);
	step_Direction_hostNode = hostNode->descend_From_TreeNode_Dir;

	// directly drop the node to the bottom when the height is less than threshold
	if (treeNode_coord3D[2] < threshold_bottom) {

		//create one TreeNode on base
		QMeshNode* bottom_TreeNode = new QMeshNode;
		bottom_TreeNode->SetCoord3D(treeNode_coord3D[0], treeNode_coord3D[1], -2.0);// -2mm: go into the bottom platform
		bottom_TreeNode->is_Host = false;
		bottom_TreeNode->is_Processed = true;   hostNode->is_Processed = true;
		bottom_TreeNode->descend_To_TreeNode_Dir = step_Direction_hostNode;
		bottom_TreeNode->treeNode_before_branch_NUM = hostNode->treeNode_before_branch_NUM;
		bottom_TreeNode->SetMeshPatchPtr(supportRay_patch);
		bottom_TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
		supportRay_patch->GetNodeList().AddTail(bottom_TreeNode);
		host_dropNode = bottom_TreeNode;

		//create one TreeEdge
		QMeshEdge* bottom_TreeEdge = new QMeshEdge;
		bottom_TreeEdge->treeEdge_branch_NUM = hostNode->treeNode_before_branch_NUM;
		bottom_TreeEdge->treeEdge_height = layer_height; // same as the layer contain endNode
		bottom_TreeEdge->SetStartPoint(hostNode);
		bottom_TreeEdge->SetEndPoint(bottom_TreeNode);
		bottom_TreeEdge->SetMeshPatchPtr(supportRay_patch);
		bottom_TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
		supportRay_patch->GetEdgeList().AddTail(bottom_TreeEdge);

		return host_dropNode;
	}

	for (int n = 0; n < largeLayer_vector_m_1.size(); n++) {
		QMeshPatch* bellow_layer = largeLayer_vector_m_1[n];

		Eigen::Vector3d insertP, v1, v2, v3;

		for (GLKPOSITION Pos = bellow_layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)bellow_layer->GetFaceList().GetNext(Pos);

			/*speeding up by distance limit*/
			// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
			Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
			if ((center - treeNode_coord3D).norm() > 5.0) continue;

			Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
			Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
			Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

			if (_intersetTriangle(treeNode_coord3D, step_Direction_hostNode, v1, v2, v3, insertP)) {

				isIntersect = true;
				//create one TreeNode on lower layer's face
				QMeshNode* TreeNode = new QMeshNode;
				TreeNode->SetCoord3D(insertP[0], insertP[1], insertP[2]);
				TreeNode->is_Host = false;
				TreeNode->is_Processed = false;
				TreeNode->treeNode_belonging_Face = Face;
				TreeNode->descend_To_TreeNode_Dir = step_Direction_hostNode;
				TreeNode->treeNode_before_branch_NUM = hostNode->treeNode_before_branch_NUM;
				TreeNode->SetMeshPatchPtr(supportRay_patch);
				TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
				supportRay_patch->GetNodeList().AddTail(TreeNode);
				Face->support_treeNode_onFace.push_back(TreeNode);
				host_dropNode = TreeNode;

				//create one TreeEdge
				QMeshEdge* TreeEdge = new QMeshEdge;
				TreeEdge->treeEdge_height = layer_height;
				TreeEdge->treeEdge_branch_NUM = hostNode->treeNode_before_branch_NUM;
				TreeEdge->SetStartPoint(hostNode);
				TreeEdge->SetEndPoint(TreeNode);
				TreeEdge->SetMeshPatchPtr(supportRay_patch);
				TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
				supportRay_patch->GetEdgeList().AddTail(TreeEdge);

				break;
			}
		}
		if (isIntersect) break;
	}

	if (isIntersect == false) {

		QMeshNode* TreeNode_virtual = new QMeshNode;
		Eigen::Vector3d drop_Node_virtual_host = treeNode_coord3D + 1.0 * step_Direction_hostNode; // 1.0 means the virtual length
		TreeNode_virtual->SetCoord3D(drop_Node_virtual_host[0], drop_Node_virtual_host[1], drop_Node_virtual_host[2]);

		TreeNode_virtual->is_Host = true;
		TreeNode_virtual->is_Processed = false;
		TreeNode_virtual->is_virtual_Host = true;
		TreeNode_virtual->descend_To_TreeNode_Dir = step_Direction_hostNode;
		TreeNode_virtual->treeNode_before_branch_NUM = hostNode->treeNode_before_branch_NUM;
		TreeNode_virtual->SetMeshPatchPtr(supportRay_patch);
		TreeNode_virtual->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
		supportRay_patch->GetNodeList().AddTail(TreeNode_virtual);
		//std::cout << "force the host node have a virtual drop node" << std::endl;
		host_dropNode = TreeNode_virtual;
	}

	// tag --> the hostNode is processed
	hostNode->is_Processed = true;

	return host_dropNode;
}

bool supportGeneration::_compute_descend_Dir_followNode(QMeshNode* followNode, QMeshNode* dropNode_hostNode) {

	//followNode .
	//            \
	//            _\|
	//              .dropNode_hostNode

	bool is_merged = false;
	//incline a self-supporting angle to the z direction
	Eigen::Vector3d dropNode_hostHost_Coord3D; 
	dropNode_hostNode->GetCoord3D(dropNode_hostHost_Coord3D[0], dropNode_hostHost_Coord3D[1], dropNode_hostHost_Coord3D[2]);
	Eigen::Vector3d followNode_coord3D; 
	followNode->GetCoord3D(followNode_coord3D[0], followNode_coord3D[1], followNode_coord3D[2]);

	Eigen::Vector3d descend_dir = (dropNode_hostHost_Coord3D - followNode_coord3D).normalized();

	Eigen::Vector3d face_normal; 
	face_normal << followNode->treeNode_belonging_Face->m_desiredNormal[0],
		followNode->treeNode_belonging_Face->m_desiredNormal[1],
		followNode->treeNode_belonging_Face->m_desiredNormal[2];

	Eigen::Vector3d rotateAxis = face_normal.cross(descend_dir);

	double radian_angle = atan2(face_normal.cross(descend_dir).norm(), face_normal.transpose() * descend_dir);
	if (ROTATE_TO_DEGREE(radian_angle) < tau) {
		followNode->descend_From_TreeNode_Dir = descend_dir; //in the range of support angle
		is_merged = true;
	}
	else {
		// only rotate a support angle
		Eigen::AngleAxisd V(DEGREE_TO_ROTATE(tau), rotateAxis);	//rotateAxis，rotate tau(deg)
		followNode->descend_From_TreeNode_Dir = V * face_normal;
		is_merged = false;
	}

	//protect for safe: merged point cannot occur on the virtual hostNode
	if (dropNode_hostNode->is_virtual_Host) is_merged = false;

	return is_merged;
}

void supportGeneration::_build_follow_treeNodeEdge(
	QMeshNode* followNode,
	QMeshNode* dropNode_hostNode,
	const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
	bool is_merge,
	QMeshPatch* supportRay_patch,
	int layer_height) {

	bool isIntersect = false;
	Eigen::Vector3d treeNode_coord3D, step_Direction_followNode;
	followNode->GetCoord3D(treeNode_coord3D[0], treeNode_coord3D[1], treeNode_coord3D[2]);
	step_Direction_followNode = followNode->descend_From_TreeNode_Dir;

	if (treeNode_coord3D[2] < threshold_bottom) {

		//create one TreeNode on base
		QMeshNode* bottom_TreeNode = new QMeshNode;
		bottom_TreeNode->SetCoord3D(treeNode_coord3D[0], treeNode_coord3D[1], -2.0);//-2mm for reasonable bottom
		bottom_TreeNode->is_Host = false;
		bottom_TreeNode->is_Processed = true;	followNode->is_Processed = true;
		bottom_TreeNode->descend_To_TreeNode_Dir = step_Direction_followNode;
		bottom_TreeNode->treeNode_before_branch_NUM = followNode->treeNode_before_branch_NUM;
		bottom_TreeNode->SetMeshPatchPtr(supportRay_patch);
		bottom_TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
		supportRay_patch->GetNodeList().AddTail(bottom_TreeNode);

		//create one TreeEdge
		QMeshEdge* bottom_TreeEdge = new QMeshEdge;
		bottom_TreeEdge->treeEdge_height = layer_height;
		bottom_TreeEdge->treeEdge_branch_NUM = followNode->treeNode_before_branch_NUM;
		bottom_TreeEdge->SetStartPoint(followNode);
		bottom_TreeEdge->SetEndPoint(bottom_TreeNode);
		bottom_TreeEdge->SetMeshPatchPtr(supportRay_patch);
		bottom_TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
		supportRay_patch->GetEdgeList().AddTail(bottom_TreeEdge);

		return;
	}

	for (int n = 0; n < largeLayer_vector_m_1.size(); n++) {
		QMeshPatch* bellow_layer = largeLayer_vector_m_1[n];

		Eigen::Vector3d insertP, v1, v2, v3; 


		for (GLKPOSITION Pos = bellow_layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)bellow_layer->GetFaceList().GetNext(Pos);

			/*speeding up by distance limit*/
			// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
			Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
			if ((center - treeNode_coord3D).norm() > 5.0) continue;

			Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
			Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
			Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

			if (_intersetTriangle(treeNode_coord3D, step_Direction_followNode, v1, v2, v3, insertP)) {

				isIntersect = true;
				//create one TreeNode on lower layer's face
				if (is_merge) {
					//merge only create TreeEdge
					QMeshEdge* TreeEdge_merge = new QMeshEdge;
					TreeEdge_merge->treeEdge_height = layer_height;
					TreeEdge_merge->treeEdge_branch_NUM = followNode->treeNode_before_branch_NUM;
					TreeEdge_merge->SetStartPoint(followNode);
					TreeEdge_merge->SetEndPoint(dropNode_hostNode);
					TreeEdge_merge->SetMeshPatchPtr(supportRay_patch);
					TreeEdge_merge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
					supportRay_patch->GetEdgeList().AddTail(TreeEdge_merge);

					dropNode_hostNode->treeNode_before_branch_NUM += followNode->treeNode_before_branch_NUM;

				}
				else {
					QMeshNode* TreeNode = new QMeshNode;
					TreeNode->SetCoord3D(insertP[0], insertP[1], insertP[2]);
					TreeNode->is_Host = false;
					TreeNode->is_Processed = false;
					TreeNode->treeNode_belonging_Face = Face;
					TreeNode->descend_To_TreeNode_Dir = step_Direction_followNode;
					TreeNode->treeNode_before_branch_NUM = followNode->treeNode_before_branch_NUM;
					TreeNode->SetMeshPatchPtr(supportRay_patch);
					TreeNode->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
					supportRay_patch->GetNodeList().AddTail(TreeNode);
					Face->support_treeNode_onFace.push_back(TreeNode);

					//create one TreeEdge
					QMeshEdge* TreeEdge = new QMeshEdge;
					TreeEdge->treeEdge_height = layer_height;
					TreeEdge->treeEdge_branch_NUM = followNode->treeNode_before_branch_NUM;
					TreeEdge->SetStartPoint(followNode);
					TreeEdge->SetEndPoint(TreeNode);
					TreeEdge->SetMeshPatchPtr(supportRay_patch);
					TreeEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
					supportRay_patch->GetEdgeList().AddTail(TreeEdge);
				}

				break;
			}
		}
		if (isIntersect) break;
	}
	// tag --> the followNode is processed
	followNode->is_Processed = true;
}

bool supportGeneration::_get_dropNode_host_treeStructure(
	Eigen::Vector3d& drop_Node_host,
	Eigen::Vector3d& treeNode_coord3D,
	Eigen::Vector3d& step_Direction_hostNode,
	QMeshNode* sourceNode,
	const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
	bool is_continous_hostNode
) {

	bool isIntersect = false;
	for (int n = 0; n < largeLayer_vector_m_1.size(); n++) {
		QMeshPatch* bellow_layer = largeLayer_vector_m_1[n];

		Eigen::Vector3d insertP, v1, v2, v3;
		for (GLKPOSITION Pos = bellow_layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)bellow_layer->GetFaceList().GetNext(Pos);

			/*speeding up by distance limit*/
			// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
			Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
			if ((center - treeNode_coord3D).norm() > 5.0) continue;

			Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
			Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
			Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

			if (_intersetTriangle(treeNode_coord3D, step_Direction_hostNode, v1, v2, v3, insertP)) {

				isIntersect = true;
				drop_Node_host = insertP;
				//record the info (of treeNode) for source Node
				sourceNode->polyline_node.push_back(insertP);// continue use the polyline_node for Function reuse
				int before_weight = sourceNode->polyline_node_weight.back();
				sourceNode->polyline_node_weight.push_back(before_weight);// give the host node with beforeWeight as the weight
				//std::cout << "the wight of host treeNode: " << sourceNode->polyline_node_weight.back() << std::endl;
				//record treeNode on Face of bottom layer 
				SptTreeNode stNode;
				stNode.souce_Node = sourceNode;
				stNode.treeNode_descend_Nor = step_Direction_hostNode; //-->node
				stNode.treeNode_coord3D = insertP;
				stNode.isHostNode = false;
				if (is_continous_hostNode)stNode.isHostNode = true;
				stNode.isProcessed = false;
				stNode.belong_Face_Normal << Face->m_desiredNormal[0], Face->m_desiredNormal[1], Face->m_desiredNormal[2];
				Face->support_treeNode_cell.push_back(stNode);
				//std::cout << "*" << std::endl;

				break;
			}
		}
		if (isIntersect) break;
	}

	if (isIntersect == false) {
		drop_Node_host = treeNode_coord3D + 0.8 * step_Direction_hostNode;
		isIntersect = true;
		//std::cout << "force the host node have a virtual drop node" << std::endl;
	}

	return isIntersect;
}

bool supportGeneration::_get_dropNode_follow_treeStructure(
	Eigen::Vector3d& drop_Node_host,
	Eigen::Vector3d& treeNode_coord3D,
	Eigen::Vector3d& step_Direction_followNode,
	QMeshNode* sourceNode,
	const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
	QMeshNode* host_sourceNode
) {

	bool isIntersect = false;
	for (int n = 0; n < largeLayer_vector_m_1.size(); n++) {
		QMeshPatch* bellow_layer = largeLayer_vector_m_1[n];

		Eigen::Vector3d insertP, v1, v2, v3;
		for (GLKPOSITION Pos = bellow_layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)bellow_layer->GetFaceList().GetNext(Pos);

			/*speeding up by distance limit*/
			// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
			Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
			if ((center - treeNode_coord3D).norm() > 5.0) continue;

			Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
			Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
			Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

			if (_intersetTriangle(treeNode_coord3D, step_Direction_followNode, v1, v2, v3, insertP)) {

				isIntersect = true;
				//record the info (of treeNode) for source Node
				sourceNode->polyline_node.push_back(insertP);// continue use the polyline_node for Function reuse
				int before_weight = sourceNode->polyline_node_weight.back();
				sourceNode->polyline_node_weight.push_back(before_weight);// give the follow node with before as the weight
				//std::cout << "the wight of follow treeNode: " << sourceNode->polyline_node_weight.back() << std::endl;

				//record treeNode on Face of bottom layer 
				SptTreeNode stNode;
				stNode.souce_Node = sourceNode;
				stNode.treeNode_descend_Nor = step_Direction_followNode; //-->node
				stNode.treeNode_coord3D = insertP;
				stNode.isHostNode = false;
				stNode.isProcessed = false;
				//follower tree node merging
				if ((drop_Node_host - insertP).norm() < 0.01) {	
					stNode.isProcessed = true; 
					host_sourceNode->polyline_node_weight.back() += sourceNode->polyline_node_weight.back();
					//std::cout << "the wight of merged treeNode: " << sourceNode->polyline_node_weight.back() << std::endl;
					//host_sourceNode->polyline_node_weight.back()++;
				}
				stNode.belong_Face_Normal << Face->m_desiredNormal[0], Face->m_desiredNormal[1], Face->m_desiredNormal[2];
				Face->support_treeNode_cell.push_back(stNode);

				break;
			}
		}
		if (isIntersect) break;
	}

	return isIntersect;
}

//true - downward; false - center
void supportGeneration::_get_step_direction_DownWard_OR_center(
	QMeshFace* Face, 
	Eigen::Vector3d& treeNode_coord3D,
	Eigen::Vector3d& step_Direction, 
	bool is_downWard
) {

	//incline a self-supporting angle to the z direction
	Eigen::Vector3d descend_dir = { 0.0,0.0,-1.0 };
	if (!is_downWard) {
		descend_dir = -treeNode_coord3D.normalized();
	}
	Eigen::Vector3d face_normal; face_normal << Face->m_desiredNormal[0], Face->m_desiredNormal[1], Face->m_desiredNormal[2];
	Eigen::Vector3d rotateAxis = face_normal.cross(descend_dir);

	//angle between _dir and _descend_dir [0,PI]
	double radian_angle = atan2(face_normal.cross(descend_dir).norm(), face_normal.transpose() * descend_dir);
	if (ROTATE_TO_DEGREE(radian_angle) < tau) step_Direction = descend_dir; // in the range of support angle, directly downward
	else {
		// only rotate a support angle
		Eigen::AngleAxisd V1(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis，rotate tau(deg)
		step_Direction = V1 * face_normal;
	}
}

//for follower to get step Direction
void supportGeneration::_get_step_direction_dropNode(
	Eigen::Vector3d& step_Direction,//return
	Eigen::Vector3d& belong_Face_Normal,
	Eigen::Vector3d& treeNode_coord3D,
	Eigen::Vector3d& drop_Node_host)
{

	//incline a self-supporting angle to the z direction
	Eigen::Vector3d descend_dir = (drop_Node_host - treeNode_coord3D).normalized();

	Eigen::Vector3d face_normal; face_normal = belong_Face_Normal;
	Eigen::Vector3d rotateAxis = face_normal.cross(descend_dir);

	//angle between _dir and _descend_dir [0,PI]
	double radian_angle = atan2(face_normal.cross(descend_dir).norm(), face_normal.transpose() * descend_dir);
	if (ROTATE_TO_DEGREE(radian_angle) < tau) step_Direction = descend_dir; // in the range of support angle, directly downward
	else {
		// only rotate a support angle
		Eigen::AngleAxisd V1(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis，rotate tau(deg)
		step_Direction = V1 * face_normal;
	}
}

int supportGeneration::_get_hostNode_on_1st_largeLayer_below(
	std::vector<SptTreeNode>& hostNode_1st_LB_Set, 
	int host_NodeNum,
	const std::vector<QMeshPatch*>& largeLayer_vector_m_1
) {

	int hostNode_onBellowLayer = 0;

	for (int n = 0; n < largeLayer_vector_m_1.size(); n++) {
		QMeshPatch* bellow_layer = largeLayer_vector_m_1[n];

		// skip initial layers in below large layer
		if (bellow_layer->is_SupportLayer == false) continue;

		//bellow layer face which contains host node
		for (GLKPOSITION Pos = bellow_layer->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)bellow_layer->GetFaceList().GetNext(Pos);

			if (Face->support_treeNode_cell.size() != 0) {
				for (int i = 0; i < Face->support_treeNode_cell.size(); i++) {
					//get host node on bottom layer
					if (Face->support_treeNode_cell[i].isHostNode) {
						hostNode_1st_LB_Set.push_back(Face->support_treeNode_cell[i]);
						hostNode_onBellowLayer++;
					}
				}
			}
		}
	}

	bool dir_find_MODE = -1;
	//if (hostNode_onBellowLayer < host_NodeNum) {
	// there are at least one host node
	if (hostNode_onBellowLayer == 0) {
		dir_find_MODE = 0;
	}
	else if(hostNode_onBellowLayer == host_NodeNum){
		dir_find_MODE = 1;
	}
	else if (hostNode_onBellowLayer > 0 && hostNode_onBellowLayer < host_NodeNum) {
		dir_find_MODE = 2;
	}
	else {
		std::cout << "Error: hostNode_onBellowLayer should be in [0, host_NodeNum]" << std::endl;
	}
	return dir_find_MODE;
}

int supportGeneration::_get_step_direction_tree(
	QMeshFace* Face,
	Eigen::Vector3d slave_node_coord3D,
	Eigen::Vector3d slave_node_Normal,
	Eigen::Vector3d& step_Direction, 
	int dir_find_MODE,
	const std::vector<SptTreeNode>& hostNode_1st_LB_Set) {

	Eigen::Vector3d descend_dir = { 0.0,0.0,0.0 };
	Eigen::Vector3d face_normal; face_normal << Face->m_desiredNormal[0], Face->m_desiredNormal[1], Face->m_desiredNormal[2];
	Eigen::Vector3d rotateAxis;
	double radian_angle;

	int nearest_hostNode_index = -1;

	if (dir_find_MODE == 0) {
		//incline a self-supporting angle to the {0,0,0}
		descend_dir = -slave_node_coord3D.normalized(); 
		rotateAxis = face_normal.cross(descend_dir);
		//angle between _dir and _descend_dir [0,PI]
		radian_angle = atan2(face_normal.cross(descend_dir).norm(), face_normal.transpose() * descend_dir);

		if (ROTATE_TO_DEGREE(radian_angle) < tau) { 
			// in the range of support angle, rotate radian_angle
			Eigen::AngleAxisd V(radian_angle, rotateAxis);//rotateAxis，rotate radian_angle(rad)
			step_Direction = V * face_normal; 
		} 
		else {
			// only rotate a support angle
			Eigen::AngleAxisd V1(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis，rotate tau(deg)
			step_Direction = V1 * face_normal;
		}
	}
	else if (dir_find_MODE == 1) {

		double minDis_squared = 9999999.9;
		for (int i = 0; i < hostNode_1st_LB_Set.size(); i++) {
			if ((hostNode_1st_LB_Set[i].treeNode_coord3D - slave_node_coord3D).squaredNorm() < minDis_squared) {
				nearest_hostNode_index = i;
			}
		}
		if (nearest_hostNode_index == -1) std::cout << "Error: did not find nearest host node in (dir_find_MODE == 1)" << std::endl;

		//incline a self-supporting angle to the hostNode
		descend_dir = (hostNode_1st_LB_Set[nearest_hostNode_index].treeNode_coord3D - slave_node_coord3D).normalized();
		rotateAxis = face_normal.cross(descend_dir);
		//angle between _dir and _descend_dir [0,PI]
		radian_angle = atan2(face_normal.cross(descend_dir).norm(), face_normal.transpose() * descend_dir);

		if (ROTATE_TO_DEGREE(radian_angle) < tau) {
			// in the range of support angle, rotate radian_angle
			Eigen::AngleAxisd V(radian_angle, rotateAxis);//rotateAxis，rotate radian_angle(rad)
			//Eigen::AngleAxisd V(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis，rotate tau(deg)
			step_Direction = V * face_normal;
		}
		else {
			// only rotate a support angle
			Eigen::AngleAxisd V1(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis，rotate tau(deg)
			step_Direction = V1 * face_normal;
		}
	}
	else if (dir_find_MODE == 2) {
		step_Direction = slave_node_Normal;
	}
	else {
		std::cout << "Error: dir_find_MODE out of range\n --> MODE: 0 - find center; 1 - find nearest hostNode; 2; along before dir\n";
	}
	return nearest_hostNode_index;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Line: orig + ->dir; triangle: v0, v1, v2; insertP: intersection point
// reference: https://www.cnblogs.com/graphics/archive/2010/08/09/1795348.html
bool supportGeneration::_intersetTriangle(Eigen::Vector3d& orig, Eigen::Vector3d& dir,
	Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& insertP) {

	double t, u, v;

	// E1,E2,P
	Eigen::Vector3d E1 = v1 - v0;	
	Eigen::Vector3d E2 = v2 - v0;
	Eigen::Vector3d P = dir.cross(E2);

	// determinant
	double det = E1.dot(P);

	// keep det > 0, modify T accordingly
	Eigen::Vector3d T;
	if (det > 0)
	{
		T = orig - v0;
	}
	else
	{
		T = v0 - orig;
		det = -det;
	}

	// If determinant is near zero, ray lies in plane of triangle
	if (det < 0.0001) {
		//std::cout << "this node lies in plane!!!" << std::endl;
		return false;
	}

	// Calculate u and make sure u <= 1
	u = T.dot(P);
	if (u < 0.0f || u > det)
		//if (u < -0.5f || u > det*1.5)
		return false;

	// Q
	Eigen::Vector3d Q = T.cross(E1);

	// Calculate v and make sure u + v <= 1
	v = dir.dot(Q);
	if (v < 0.0f || u + v > det)
		//if (v < -0.5f || u + v > det*1.5)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	t = E2.dot(Q);

	float fInvDet = 1.0f / det;
	t *= fInvDet;
	u *= fInvDet;
	v *= fInvDet;

	insertP = orig + t * dir;
	//std::cout << insertP << endl;
	//std::cout << (1 - u - v)*v0 + u*v1 + v*v2 << endl << endl;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// collect the support RAY for isoLayers
void supportGeneration::collect_SupportRays() {

	QMeshPatch* supportRay_patch = new QMeshPatch;
	supportRay_patch->SetIndexNo(m_supportRaySet->GetMeshList().GetCount()); //index begin from 0
	m_supportRaySet->GetMeshList().AddTail(supportRay_patch);
	supportRay_patch->drawThisPatch = true;

	for (int i = 0; i < initial_layers.size(); i++) {
		QMeshPatch* layer = initial_layers[i];

		int num_needSupport = 0;
		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

			if (Node->need_Support) num_needSupport++;
		}
		if (num_needSupport <= needSupport_threshold) continue;

		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

			if (Node->need_Support) {

				double xx, yy, zz;

				QMeshNode* orgin_Node = new QMeshNode;
				Node->GetCoord3D(xx, yy, zz);
				orgin_Node->SetCoord3D(xx, yy, zz);
				orgin_Node->SetMeshPatchPtr(supportRay_patch);
				orgin_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
				supportRay_patch->GetNodeList().AddTail(orgin_Node);
				orgin_Node->isOringin = true;

				QMeshNode* target_Node = new QMeshNode;
				target_Node->SetCoord3D(Node->supportEndPos[0], Node->supportEndPos[1], Node->supportEndPos[2]);
				target_Node->SetMeshPatchPtr(supportRay_patch);
				target_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
				supportRay_patch->GetNodeList().AddTail(target_Node);

				QMeshEdge* rayEdge = new QMeshEdge;
				rayEdge->SetStartPoint(orgin_Node);
				rayEdge->SetEndPoint(target_Node);

				rayEdge->SetMeshPatchPtr(supportRay_patch);
				rayEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0

				orgin_Node->GetEdgeList().AddTail(rayEdge);
				target_Node->GetEdgeList().AddTail(rayEdge);
				supportRay_patch->GetEdgeList().AddTail(rayEdge);

			}
		}
	}
	std::cout << "Ray num: "<< supportRay_patch->GetEdgeNumber() << std::endl;
}

// collect the support RAY based on tetSurface and vector field
void supportGeneration::collect_SupportRays_vertical() {

	QMeshPatch* supportRay_patch = new QMeshPatch;
	supportRay_patch->SetIndexNo(m_supportRaySet->GetMeshList().GetCount()); //index begin from 0
	m_supportRaySet->GetMeshList().AddTail(supportRay_patch);
	supportRay_patch->drawThisPatch = true;

	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();

	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);

		if (Node->inner) continue;	// the inner face is not considered 
		if (Node->need_Support == false) continue;

		double xx, yy, zz;

		QMeshNode* orgin_Node = new QMeshNode;
		Node->GetCoord3D(xx, yy, zz);
		orgin_Node->SetCoord3D(xx, yy, zz);
		orgin_Node->SetMeshPatchPtr(supportRay_patch);
		orgin_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
		supportRay_patch->GetNodeList().AddTail(orgin_Node);
		orgin_Node->isOringin = true;

		QMeshNode* target_Node = new QMeshNode;
		target_Node->SetCoord3D(Node->supportEndPos[0], Node->supportEndPos[1], Node->supportEndPos[2]);
		target_Node->SetMeshPatchPtr(supportRay_patch);
		target_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
		supportRay_patch->GetNodeList().AddTail(target_Node);

		QMeshEdge* rayEdge = new QMeshEdge;
		rayEdge->SetStartPoint(orgin_Node);
		rayEdge->SetEndPoint(target_Node);

		rayEdge->SetMeshPatchPtr(supportRay_patch);
		rayEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0

		orgin_Node->GetEdgeList().AddTail(rayEdge);
		target_Node->GetEdgeList().AddTail(rayEdge);
		supportRay_patch->GetEdgeList().AddTail(rayEdge);

	}
	std::cout << " --> Ray num: " << supportRay_patch->GetEdgeNumber() << std::endl;
}

// collect the support polyline for isoLayers
void supportGeneration::collect_Support_Polyline() {

	QMeshPatch* supportRay_patch = new QMeshPatch;
	supportRay_patch->SetIndexNo(m_supportRaySet->GetMeshList().GetCount()); //index begin from 0
	m_supportRaySet->GetMeshList().AddTail(supportRay_patch);
	supportRay_patch->drawThisPatch = true;

	for (int i = 0; i < initial_layers.size(); i++) {
		QMeshPatch* layer = initial_layers[i];

		/*test: maybe use in the future*/
		////little trick: if the ray NUM is too less, consider this layer does not need  support rays.
		//int num_needSupport = 0;
		//for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
		//	QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

		//	if (Node->need_Support) num_needSupport++;
		//}
		//if (num_needSupport <= needSupport_threshold) continue;

		for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

			if (Node->polyline_node.size() != 0) {

				double xx, yy, zz;
				std::vector<QMeshNode*> polyLine_node_set(Node->polyline_node.size() + 1);

				//add the 1st node
				QMeshNode* orgin_Node = new QMeshNode;
				Node->GetCoord3D(xx, yy, zz);
				orgin_Node->SetCoord3D(xx, yy, zz);
				orgin_Node->SetMeshPatchPtr(supportRay_patch);
				orgin_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
				supportRay_patch->GetNodeList().AddTail(orgin_Node);
				orgin_Node->isOringin = true;
				polyLine_node_set[0] = orgin_Node;

				//add rest polyline node
				for (int j = 0; j < Node->polyline_node.size(); j++) {

					QMeshNode* poly_Node = new QMeshNode;
					poly_Node->SetCoord3D(Node->polyline_node[j][0], Node->polyline_node[j][1], Node->polyline_node[j][2]);
					poly_Node->SetMeshPatchPtr(supportRay_patch);
					poly_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
					supportRay_patch->GetNodeList().AddTail(poly_Node);
					polyLine_node_set[j + 1] = poly_Node;
				}

				//add polyline edge
				for (int k = 0; k < polyLine_node_set.size() - 1; k++) {
					QMeshEdge* rayEdge = new QMeshEdge;
					rayEdge->SetStartPoint(polyLine_node_set[k]);
					rayEdge->SetEndPoint(polyLine_node_set[k + 1]);
					rayEdge->SetMeshPatchPtr(supportRay_patch);
					rayEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
					supportRay_patch->GetEdgeList().AddTail(rayEdge);
				}

			}
		}
	}
	std::cout << "polyline segments num: " << supportRay_patch->GetEdgeNumber() << std::endl;
}

// collect the support polyline for isoLayers based on tetSurface and vector field
void supportGeneration::collect_Support_Polyline_fromTETsurface() {

	QMeshPatch* supportRay_patch = new QMeshPatch;
	supportRay_patch->SetIndexNo(m_supportRaySet->GetMeshList().GetCount()); //index begin from 0
	m_supportRaySet->GetMeshList().AddTail(supportRay_patch);
	supportRay_patch->drawThisPatch = true;

	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();
	int max_segment_NUM = 0;

	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);

		if (Node->polyline_node.size() != 0) {

			//std::cout << "Node: " << Node->GetIndexNo() << std::endl;
			//if (Node->polyline_node.size() != Node->polyline_node_weight.size())
			//	std::cout << "Error: polyline_node NUM != polyline_node_weight NUM" << std::endl;

			//record the max segment NUM
			if (Node->polyline_node.size() > max_segment_NUM) // size return unsigned Int
				max_segment_NUM = Node->polyline_node.size();


			double xx, yy, zz;
			std::vector<QMeshNode*> polyLine_node_set(Node->polyline_node.size() + 1);

			//add the 1st node
			QMeshNode* orgin_Node = new QMeshNode;
			Node->GetCoord3D(xx, yy, zz);
			orgin_Node->SetCoord3D(xx, yy, zz);
			orgin_Node->SetMeshPatchPtr(supportRay_patch);
			orgin_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
			supportRay_patch->GetNodeList().AddTail(orgin_Node);
			orgin_Node->isOringin = true;
			polyLine_node_set[0] = orgin_Node;

			//add rest polyline node
			for (int j = 0; j < Node->polyline_node.size(); j++) {

				QMeshNode* poly_Node = new QMeshNode;
				if (Node->isHostNode) poly_Node->isHostNode = true;
				poly_Node->SetCoord3D(Node->polyline_node[j][0], Node->polyline_node[j][1], Node->polyline_node[j][2]);
				poly_Node->SetMeshPatchPtr(supportRay_patch);
				poly_Node->SetIndexNo(supportRay_patch->GetNodeList().GetCount());//index begin from 0
				supportRay_patch->GetNodeList().AddTail(poly_Node);
				polyLine_node_set[j + 1] = poly_Node;

				//temp: tianyu 20201217
				//std::cout << Node->polyline_node_weight[j] << " ";
				//if (j > 0) {
				//	if (Node->polyline_node_weight[j] < Node->polyline_node_weight[j - 1])
				//		std::cout << "ERROR: the weight is not increase" << std::endl;
				//}
				

			}
			//std::cout << std::endl;

			//add polyline edge
			for (int k = 0; k < polyLine_node_set.size() - 1; k++) {
				QMeshEdge* rayEdge = new QMeshEdge;
				//method 1: section NUM
				//rayEdge->length_index = k + 1;
				//method 2: merge edge NUM
				//if (k == 0) rayEdge->length_index = 1;
				//else	rayEdge->length_index = Node->polyline_node_weight[k - 1];
				//std::cout << "rayEdge->weight -> "<< rayEdge->length_index << std::endl;
				rayEdge->SetStartPoint(polyLine_node_set[k]);
				rayEdge->SetEndPoint(polyLine_node_set[k + 1]);
				rayEdge->SetMeshPatchPtr(supportRay_patch);
				rayEdge->SetIndexNo(supportRay_patch->GetEdgeList().GetCount());//index begin from 0
				supportRay_patch->GetEdgeList().AddTail(rayEdge);
			}

		}
	}

	supportRay_patch->max_segment_NUM = max_segment_NUM;
	std::cout << "max segment num: " << supportRay_patch->max_segment_NUM << std::endl;
	std::cout << "polyline segments num: " << supportRay_patch->GetEdgeNumber() << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Line: O->T; Sphere: center,R; mu: Explicit argument
// reference: http://www.ambrsoft.com/TrigoCalc/Sphere/SpherLineIntersection_.htm
bool supportGeneration::_lineIntersectSphere
(Eigen::Vector3d& O, Eigen::Vector3d& T, Eigen::Vector3d& Center, double R, double& mu1, double& mu2) {

	//std::cout << R << std::endl;

	double a = (T - O).squaredNorm();
	double b = -2 * (T - O).dot(Center - O);
	double c = (Center - O).squaredNorm() - R * R;

	double bb4ac = b * b - 4 * a * c;

	if (fabs(a) == 0 || bb4ac <= 0) {
		mu1 = 0;
		mu2 = 0;
		return false;
	}

	mu1 = (-b + sqrt(bb4ac)) / (2 * a);
	mu2 = (-b - sqrt(bb4ac)) / (2 * a);

	// keep mu1 < mu2
	if (mu2 < mu1) {
		double temp_mu = mu1;
		mu1 = mu2;
		mu2 = temp_mu;
		//std::cout << "exchange mu1,mu2" << std::endl;
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double supportGeneration::_implicitDistance_Func(Eigen::Vector3d& queryPnt) {

	double F_queryPnt = 0.0;
	double r_i = 3.25;
	r_i_MAX = r_i;// refresh the r_i_MAX for adaptive r_i algorithm (min radius)
	double R = r_i_MAX * detectRadius_ratio; // refresh detection radius R

	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		Eigen::Vector2d m = { 1.0, 3.2 };//2.5
		Eigen::Vector2d n = { std::sqrt((double)layer->max_branch_NUM), 4.5 };//4.0
		double k = (n[1] - m[1]) / (n[0] - m[0]);

		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
			QMeshEdge* edge = (QMeshEdge*)(layer->GetEdgeList().GetNext(Pos));

			if (!edge->GetEndPoint()->isUseful_Node_SupportRay ||
				!edge->GetStartPoint()->isUseful_Node_SupportRay) continue;

			//linear interpolation: sqrt(edge->treeEdge_branch_NUM) V.S. r_i
			double sqrt_branch_NUM = std::sqrt((double)edge->treeEdge_branch_NUM);
			//std::cout << sqrt_branch_NUM << std::endl;
			//double r_i = k * (sqrt_branch_NUM - m[0]) + m[1];
			// test: add hight into r_i
			//r_i += (layer->max_height - edge->treeEdge_height) * (5.0 / layer->max_height);
			//double R = r_i * detectRadius_ratio;	// refresh detection radius R

			Eigen::Vector3d O;	edge->GetStartPoint()->GetCoord3D(O[0], O[1], O[2]); // Vs
			Eigen::Vector3d T;	edge->GetEndPoint()->GetCoord3D(T[0], T[1], T[2]); // Ve

			//build a box and _queryPnt is center, only computing the polyline in the box --> speed up
			Eigen::Vector3d delta_Oq = (queryPnt - O);
			Eigen::Vector3d delta_Tq = (queryPnt - T);
			if ((abs(delta_Oq[0]) > R || abs(delta_Oq[1]) > R || abs(delta_Oq[2]) > R) &&
				(abs(delta_Tq[0]) > R || abs(delta_Tq[1]) > R || abs(delta_Tq[2]) > R))
				continue;
			
			double mu1, mu2;

			//test
			//std::cout << "O: " << O.transpose() << std::endl;
			//std::cout << "T: " << T.transpose() << std::endl;
			//std::cout << "------------------" << std::endl;

			bool intersection = _lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);

			if (intersection == false) continue;	// no intersection
			if (mu1 > 1.0 && mu2 > 1.0) continue;	// O -> T ()
			if (mu1 < 0.0 && mu2 < 0.0) continue;	// () O -> T 

			Eigen::Vector3d iPnt_1 = O + mu1 * (T - O); //p1
			Eigen::Vector3d iPnt_2 = O + mu2 * (T - O); //p2

			//std::cout << "iPnt_1: " << iPnt_1.transpose() << std::endl;
			//std::cout << "iPnt_2: " << iPnt_2.transpose() << std::endl;
			//std::cout << "------------------" << std::endl;

			double s1 = MAX(0, (O - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());
			double s2 = MIN(1, (T - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());

			if (s1 > 1.0 || s2 < 0.0) {
				std::cout << "------------------" << std::endl;
				std::cout << "ERROR:0.0 < s1 < s2 < 1.0 !!!" << std::endl;
				std::cout << "s1: " << s1 << std::endl;
				std::cout << "s2: " << s2 << std::endl;
				std::cout << "------------------" << std::endl;
			}

			double l = (iPnt_2 - iPnt_1).norm();
			double a = (queryPnt - iPnt_1).dot(iPnt_2 - iPnt_1);

			double F_queryPnt_eachRay = r_i / 15.0 / pow(R, 4) * (
				(3 * pow(l, 4) * pow(s2, 5) - 15 * a * pow(l, 2) * pow(s2, 4) + 20 * pow(a, 2) * pow(s2, 3))
				- (3 * pow(l, 4) * pow(s1, 5) - 15 * a * pow(l, 2) * pow(s1, 4) + 20 * pow(a, 2) * pow(s1, 3)));

			F_queryPnt += F_queryPnt_eachRay;
		}
	}

	return F_queryPnt;
}

double supportGeneration::_implicitDistance_Func(Eigen::Vector3d& queryPnt, int& intersection_NUM) {

	double F_queryPnt = 0.0;
	r_i_MAX = a(1);
	double R = r_i_MAX * detectRadius_ratio; // refresh detection radius R
	
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {//only one ray set (layer)
		QMeshPatch* layer = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
			QMeshEdge* edge = (QMeshEdge*)(layer->GetEdgeList().GetNext(Pos));

			if (!edge->GetEndPoint()->isUseful_Node_SupportRay ||
				!edge->GetStartPoint()->isUseful_Node_SupportRay) continue;

			Eigen::Vector3d O;	edge->GetStartPoint()->GetCoord3D(O[0], O[1], O[2]); // Vs
			Eigen::Vector3d T;	edge->GetEndPoint()->GetCoord3D(T[0], T[1], T[2]); // Ve

			double mu1, mu2;

			//test
			//std::cout << "O: " << O.transpose() << std::endl;
			//std::cout << "T: " << T.transpose() << std::endl;
			//std::cout << "------------------" << std::endl;

			double r_i = _func_adaptive_ratio(intersection_NUM);// swell radius

			bool intersection = _lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);

			if (intersection == false) continue;	// no intersection
			if (mu1 > 1.0 && mu2 > 1.0) continue;	// O -> T ()
			if (mu1 < 0.0 && mu2 < 0.0) continue;	// () O -> T 

			Eigen::Vector3d iPnt_1 = O + mu1 * (T - O); //p1
			Eigen::Vector3d iPnt_2 = O + mu2 * (T - O); //p2

			//std::cout << "iPnt_1: " << iPnt_1.transpose() << std::endl;
			//std::cout << "iPnt_2: " << iPnt_2.transpose() << std::endl;
			//std::cout << "------------------" << std::endl;

			double s1 = MAX(0, (O - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());
			double s2 = MIN(1, (T - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());

			if (s1 > 1.0 || s2 < 0.0) {
				std::cout << "------------------" << std::endl;
				std::cout << "ERROR:0.0 < s1 < s2 < 1.0 !!!" << std::endl;
				std::cout << "s1: " << s1 << std::endl;
				std::cout << "s2: " << s2 << std::endl;
				std::cout << "------------------" << std::endl;
			}

			double l = (iPnt_2 - iPnt_1).norm();
			double a = (queryPnt - iPnt_1).dot(iPnt_2 - iPnt_1);

			double F_queryPnt_eachRay = r_i / 15.0 / pow(R, 4) * (
				(3 * pow(l, 4) * pow(s2, 5) - 15 * a * pow(l, 2) * pow(s2, 4) + 20 * pow(a, 2) * pow(s2, 3))
				- (3 * pow(l, 4) * pow(s1, 5) - 15 * a * pow(l, 2) * pow(s1, 4) + 20 * pow(a, 2) * pow(s1, 3)));

			F_queryPnt += F_queryPnt_eachRay;
		}
	}

	return F_queryPnt;
}

int supportGeneration::_Intersection_NUM(Eigen::Vector3d& queryPnt) {

	int intersect_NUM = 0;
	r_i_MAX = a(1);// refresh the r_i_MAX for adaptive r_i algorithm (max radius)
	double R = r_i_MAX * detectRadius_ratio; // refresh detection radius R

	QMeshPatch* ray_patch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetHead();
	for (GLKPOSITION Pos = ray_patch->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
		QMeshEdge* edge = (QMeshEdge*)(ray_patch->GetEdgeList().GetNext(Pos));

		if (!edge->GetEndPoint()->isUseful_Node_SupportRay ||
			!edge->GetStartPoint()->isUseful_Node_SupportRay) continue;

		Eigen::Vector3d O;	edge->GetStartPoint()->GetCoord3D(O[0], O[1], O[2]); // Vs
		Eigen::Vector3d T;	edge->GetEndPoint()->GetCoord3D(T[0], T[1], T[2]); // Ve

		double mu1, mu2;

		// use initial R guess: 2*r_i
		bool intersection = _lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);

		if (intersection == false) continue;	// no intersection
		if (mu1 > 1.0 && mu2 > 1.0) continue;	// O -> T ()
		if (mu1 < 0.0 && mu2 < 0.0) continue;	// () O -> T 

		intersect_NUM++;
	}
	
	//std::cout << "intersect times: " << intersect_NUM << std::endl;
	return intersect_NUM;
}

double supportGeneration::_implicitDistance_Func(Eigen::Vector3d& queryPnt, Eigen::ArrayXXd& planeSet) {

	double F_queryPnt = 0.0;
	double r_i = 0.5 * (a(1) + b(1));
	r_i_MAX = r_i;// refresh the r_i_MAX for adaptive r_i algorithm (min radius)
	double R = r_i_MAX * detectRadius_ratio; // refresh detection radius R

	for (int i = 0; i < planeSet.rows();i++) {

		Eigen::Vector3d plane_normal;
		plane_normal << planeSet(i, 0), planeSet(i, 1), planeSet(i, 2);
		double d = plane_normal.dot(queryPnt) + planeSet(i, 3);

		if (abs(d) > R) continue;

		//std::cout << "d: " << d << std::endl;

		double F_queryPnt_eachRay = PI * r_i * pow((R * R - d * d), 3) / 3.0 / pow(R, 4);

		F_queryPnt += F_queryPnt_eachRay;
	}
	
	return F_queryPnt;
}

double supportGeneration::_implicitDistance_Func_small_ployline(Eigen::Vector3d& queryPnt) {

	double F_queryPnt = 0.0;
	double r_i = 0.5 * (a(1) + b(1));
	r_i_MAX = r_i;// refresh the r_i_MAX for adaptive r_i algorithm (min radius)
	double R = r_i_MAX * detectRadius_ratio; // refresh detection radius R

	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
			QMeshEdge* edge = (QMeshEdge*)(layer->GetEdgeList().GetNext(Pos));

			if (!edge->GetEndPoint()->isUseful_Node_SupportRay ||
				!edge->GetStartPoint()->isUseful_Node_SupportRay) continue;

			Eigen::Vector3d O;	edge->GetStartPoint()->GetCoord3D(O[0], O[1], O[2]); // Vs
			Eigen::Vector3d T;	edge->GetEndPoint()->GetCoord3D(T[0], T[1], T[2]); // Ve

			//build a box and _queryPnt is center, only computing the polyline in the box --> speed up
//			Eigen::Vector3d delta_Oq = (queryPnt - O);
//			Eigen::Vector3d delta_Tq = (queryPnt - T);
//			if ((abs(delta_Oq[0]) > R || abs(delta_Oq[1]) > R || abs(delta_Oq[2]) > R) &&
//				(abs(delta_Tq[0]) > R || abs(delta_Tq[1]) > R || abs(delta_Tq[2]) > R))			
//				continue;

			double mu1, mu2;

			//test
			//std::cout << "O: " << O.transpose() << std::endl;
			//std::cout << "T: " << T.transpose() << std::endl;
			//std::cout << "------------------" << std::endl;

			bool intersection = _lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);

			if (intersection == false) continue;	// no intersection
			if (mu1 > 1.0 && mu2 > 1.0) continue;	// O -> T ()
			if (mu1 < 0.0 && mu2 < 0.0) continue;	// () O -> T 

			Eigen::Vector3d iPnt_1 = O + mu1 * (T - O); //p1
			Eigen::Vector3d iPnt_2 = O + mu2 * (T - O); //p2

			//std::cout << "iPnt_1: " << iPnt_1.transpose() << std::endl;
			//std::cout << "iPnt_2: " << iPnt_2.transpose() << std::endl;
			//std::cout << "------------------" << std::endl;

			double s1 = MAX(0, (O - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());
			double s2 = MIN(1, (T - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());

			if (s1 > 1.0 || s2 < 0.0) {
				std::cout << "------------------" << std::endl;
				std::cout << "ERROR:0.0 < s1 < s2 < 1.0 !!!" << std::endl;
				std::cout << "s1: " << s1 << std::endl;
				std::cout << "s2: " << s2 << std::endl;
				std::cout << "------------------" << std::endl;
			}

			double l = (iPnt_2 - iPnt_1).norm();
			double a = (queryPnt - iPnt_1).dot(iPnt_2 - iPnt_1);

			double F_queryPnt_eachRay = r_i / 15.0 / pow(R, 4) * (
				(3 * pow(l, 4) * pow(s2, 5) - 15 * a * pow(l, 2) * pow(s2, 4) + 20 * pow(a, 2) * pow(s2, 3))
				- (3 * pow(l, 4) * pow(s1, 5) - 15 * a * pow(l, 2) * pow(s1, 4) + 20 * pow(a, 2) * pow(s1, 3)));

			F_queryPnt += F_queryPnt_eachRay;
		}
	}

	return F_queryPnt;
}

double supportGeneration::_implicitDistance_Func_tree(Eigen::Vector3d& queryPnt) {

	// TET surface RAY (fixed host)
	/*  ^ r_i (energy radius)
	*6.0|_______ n
	*	|	    /|
	*	|	   / |
	*   |	  /  |
	*3.0|__m_/   |
	*	|____|___|____> node section NUM
	*	0    1  max_section_num
	*/

	// TET surface RAY (dynamic host)
	/*  ^ r_i (energy radius)
	*3.8|_______ n____
	*	|	    /|   |
	*	|	   / |   |
	*   |	  /  |   |
	*0.5|__m_/   |   |
	*	|____|___|___|_> node section NUM
	*	0    1  ratio*M 
	* M = max_section_num = layer->max_segment_NUM
	*/

	double F_queryPnt = 0.0;
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		double ratio = 0.2;//0.2
		Eigen::Vector2d m = { 1.0,0.5 };//{ 1.0,0.5 };
		Eigen::Vector2d n = { ratio * layer->max_branch_NUM,3.8 };//{ ratio * layer->max_segment_NUM,3.8 };
		double k = (n[1] - m[1]) / (n[0] - m[0]);
		r_i_MAX = n[1];								// refresh the r_i_MAX for adaptive r_i algorithm (min radius)
		double R = r_i_MAX * detectRadius_ratio;	// refresh detection radius R

		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
			QMeshEdge* edge = (QMeshEdge*)(layer->GetEdgeList().GetNext(Pos));

			if (!edge->GetEndPoint()->isUseful_Node_SupportRay ||
				!edge->GetStartPoint()->isUseful_Node_SupportRay) continue;

			Eigen::Vector3d O;	edge->GetStartPoint()->GetCoord3D(O[0], O[1], O[2]); // Vs
			Eigen::Vector3d T;	edge->GetEndPoint()->GetCoord3D(T[0], T[1], T[2]); // Ve


			//build a box and _queryPnt is center, only computing the polyline in the box --> speed up
			Eigen::Vector3d delta_Oq = (queryPnt - O);
			Eigen::Vector3d delta_Tq = (queryPnt - T);
			if ((abs(delta_Oq[0]) > R || abs(delta_Oq[1]) > R || abs(delta_Oq[2]) > R) &&
				(abs(delta_Tq[0]) > R || abs(delta_Tq[1]) > R || abs(delta_Tq[2]) > R))			
				continue;


			double mu1, mu2;

			bool intersection = _lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);

			if (intersection == false) continue;	// no intersection
			if (mu1 > 1.0 && mu2 > 1.0) continue;	// O -> T ()
			if (mu1 < 0.0 && mu2 < 0.0) continue;	// () O -> T 

			Eigen::Vector3d iPnt_1 = O + mu1 * (T - O); //p1
			Eigen::Vector3d iPnt_2 = O + mu2 * (T - O); //p2

			double s1 = MAX(0, (O - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());
			double s2 = MIN(1, (T - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());

			if (s1 > 1.0 || s2 < 0.0) {
				std::cout << "------------------" << std::endl;
				std::cout << "ERROR:0.0 < s1 < s2 < 1.0 !!!" << std::endl;
				std::cout << "s1: " << s1 << std::endl;
				std::cout << "s2: " << s2 << std::endl;
				std::cout << "------------------" << std::endl;
			}

			double l = (iPnt_2 - iPnt_1).norm();
			double a = (queryPnt - iPnt_1).dot(iPnt_2 - iPnt_1);

			//double r_i = k * (edge->length_index - m[0]) + m[1];
			double r_i;
			if (edge->treeEdge_branch_NUM < n[0]) { r_i = k * (edge->treeEdge_branch_NUM - m[0]) + m[1]; }
			else { r_i = n[1]; }
			
			double F_queryPnt_eachRay = r_i / 15.0 / pow(R, 4) * (
				(3 * pow(l, 4) * pow(s2, 5) - 15 * a * pow(l, 2) * pow(s2, 4) + 20 * pow(a, 2) * pow(s2, 3))
				- (3 * pow(l, 4) * pow(s1, 5) - 15 * a * pow(l, 2) * pow(s1, 4) + 20 * pow(a, 2) * pow(s1, 3)));

			F_queryPnt += F_queryPnt_eachRay;
		}
	}

	return F_queryPnt;
}

double supportGeneration::_implicitDistance_Func_tree2(Eigen::Vector3d& queryPnt) {

	//
	/*  ^ r_i (energy radius)
	*4.0|__m
	*	|  \      
	*	|   \     
	*   |	 \    
	*0.5|     \n___
	*	|__________> node section NUM
	*	0  1  M
	* M = max_section_num = layer->max_segment_NUM
	*/

	double F_queryPnt = 0.0;
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		Eigen::Vector2d m = { 1.0, 5.0 };
		Eigen::Vector2d n = { layer->max_height, 0.1 };
		double k = (n[1] - m[1]) / (n[0] - m[0]);
		r_i_MAX = m[1];								// refresh the r_i_MAX for adaptive r_i algorithm (min radius)
		double R = r_i_MAX * detectRadius_ratio;	// refresh detection radius R

		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
			QMeshEdge* edge = (QMeshEdge*)(layer->GetEdgeList().GetNext(Pos));

			if (!edge->GetEndPoint()->isUseful_Node_SupportRay ||
				!edge->GetStartPoint()->isUseful_Node_SupportRay) continue;

			Eigen::Vector3d O;	edge->GetStartPoint()->GetCoord3D(O[0], O[1], O[2]); // Vs
			Eigen::Vector3d T;	edge->GetEndPoint()->GetCoord3D(T[0], T[1], T[2]); // Ve


			//build a box and _queryPnt is center, only computing the polyline in the box --> speed up
			Eigen::Vector3d delta_Oq = (queryPnt - O);
			Eigen::Vector3d delta_Tq = (queryPnt - T);
			if ((abs(delta_Oq[0]) > R || abs(delta_Oq[1]) > R || abs(delta_Oq[2]) > R) &&
				(abs(delta_Tq[0]) > R || abs(delta_Tq[1]) > R || abs(delta_Tq[2]) > R))
				continue;


			double mu1, mu2;

			bool intersection = _lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);

			if (intersection == false) continue;	// no intersection
			if (mu1 > 1.0 && mu2 > 1.0) continue;	// O -> T ()
			if (mu1 < 0.0 && mu2 < 0.0) continue;	// () O -> T 

			Eigen::Vector3d iPnt_1 = O + mu1 * (T - O); //p1
			Eigen::Vector3d iPnt_2 = O + mu2 * (T - O); //p2

			double s1 = MAX(0, (O - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());
			double s2 = MIN(1, (T - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());

			if (s1 > 1.0 || s2 < 0.0) {
				std::cout << "------------------" << std::endl;
				std::cout << "ERROR:0.0 < s1 < s2 < 1.0 !!!" << std::endl;
				std::cout << "s1: " << s1 << std::endl;
				std::cout << "s2: " << s2 << std::endl;
				std::cout << "------------------" << std::endl;
			}

			double l = (iPnt_2 - iPnt_1).norm();
			double a = (queryPnt - iPnt_1).dot(iPnt_2 - iPnt_1);

			//double r_i = k * (edge->length_index - m[0]) + m[1];
			double r_i;
			if (edge->treeEdge_height < m[0]) { r_i = m[1]; }
			else if (edge->treeEdge_height < n[0]){ r_i = k * (edge->treeEdge_height - n[0]) + n[1]; }
			else { r_i = n[1]; }

			double F_queryPnt_eachRay = r_i / 15.0 / pow(R, 4) * (
				(3 * pow(l, 4) * pow(s2, 5) - 15 * a * pow(l, 2) * pow(s2, 4) + 20 * pow(a, 2) * pow(s2, 3))
				- (3 * pow(l, 4) * pow(s1, 5) - 15 * a * pow(l, 2) * pow(s1, 4) + 20 * pow(a, 2) * pow(s1, 3)));

			F_queryPnt += F_queryPnt_eachRay;
		}
	}

	return F_queryPnt;
}

double supportGeneration::_implicitDistance_Func_tree3(Eigen::Vector3d& queryPnt) {

	// TET surface RAY (dynamic host)
	/*  ^ r_i (energy radius)
	*3.8|_______ n
	*	|	    /|   
	*	|	   / |   
	*   |	  /  |   
	*2.5|  m /   |   
	*	|____|___|____> sqrt(node section NUM)
	*	0    1   M
	*/

	double F_queryPnt = 0.0;
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* layer = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		Eigen::Vector2d m = { 1.0, 2.5 };
		Eigen::Vector2d n = { std::sqrt((double)layer->max_branch_NUM), 5.5 };//4.5
		double k = (n[1] - m[1]) / (n[0] - m[0]);

		for (GLKPOSITION Pos = layer->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
			QMeshEdge* edge = (QMeshEdge*)(layer->GetEdgeList().GetNext(Pos));

			if (!edge->GetEndPoint()->isUseful_Node_SupportRay ||
				!edge->GetStartPoint()->isUseful_Node_SupportRay) continue;

			Eigen::Vector3d O;	edge->GetStartPoint()->GetCoord3D(O[0], O[1], O[2]); // Vs
			Eigen::Vector3d T;	edge->GetEndPoint()->GetCoord3D(T[0], T[1], T[2]); // Ve
	
			//linear interpolation: sqrt(edge->treeEdge_branch_NUM) V.S. r_i
			double sqrt_branch_NUM = std::sqrt((double)edge->treeEdge_branch_NUM);
			//std::cout << sqrt_branch_NUM << std::endl;
			double r_i = k * (sqrt_branch_NUM - m[0]) + m[1];
			// test: add hight into r_i
			r_i += (layer->max_height - edge->treeEdge_height) * (3.0 / layer->max_height);
			double R = r_i * detectRadius_ratio;	// refresh detection radius R

			//build a box and _queryPnt is center, only computing the polyline in the box --> speed up
			Eigen::Vector3d delta_Oq = (queryPnt - O);
			Eigen::Vector3d delta_Tq = (queryPnt - T);
			if ((abs(delta_Oq[0]) > R || abs(delta_Oq[1]) > R || abs(delta_Oq[2]) > R) &&
				(abs(delta_Tq[0]) > R || abs(delta_Tq[1]) > R || abs(delta_Tq[2]) > R))
				continue;

			double mu1, mu2;
			bool intersection = _lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);

			if (intersection == false) continue;	// no intersection
			if (mu1 > 1.0 && mu2 > 1.0) continue;	// O -> T ()
			if (mu1 < 0.0 && mu2 < 0.0) continue;	// () O -> T 

			Eigen::Vector3d iPnt_1 = O + mu1 * (T - O); //p1
			Eigen::Vector3d iPnt_2 = O + mu2 * (T - O); //p2

			double s1 = MAX(0, (O - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());
			double s2 = MIN(1, (T - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());

			if (s1 > 1.0 || s2 < 0.0) {
				std::cout << "------------------" << std::endl;
				std::cout << "ERROR:0.0 < s1 < s2 < 1.0 !!!" << std::endl;
				std::cout << "s1: " << s1 << std::endl;
				std::cout << "s2: " << s2 << std::endl;
				std::cout << "------------------" << std::endl;
			}

			double l = (iPnt_2 - iPnt_1).norm();
			double a = (queryPnt - iPnt_1).dot(iPnt_2 - iPnt_1);

			double F_queryPnt_eachRay = r_i / 15.0 / pow(R, 4) * (
				(3 * pow(l, 4) * pow(s2, 5) - 15 * a * pow(l, 2) * pow(s2, 4) + 20 * pow(a, 2) * pow(s2, 3))
				- (3 * pow(l, 4) * pow(s1, 5) - 15 * a * pow(l, 2) * pow(s1, 4) + 20 * pow(a, 2) * pow(s1, 3)));

			F_queryPnt += F_queryPnt_eachRay;
		}
	}

	return F_queryPnt;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void supportGeneration::build_SupportSurface_MarchingCube() {


	QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetHead();// only one patch now
	QMeshPatch* tet_model_Patch = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();

	//build grid
	double xmin, ymin, zmin, xmax, ymax, zmax;
	double xmin_s, ymin_s, zmin_s, xmax_s, ymax_s, zmax_s;
	double xmin_i, ymin_i, zmin_i, xmax_i, ymax_i, zmax_i;
	supportRay_Patch->ComputeBoundingBox(xmin_s, ymin_s, zmin_s, xmax_s, ymax_s, zmax_s);
	tet_model_Patch->ComputeBoundingBox(xmin_i, ymin_i, zmin_i, xmax_i, ymax_i, zmax_i);
	xmin = xmin_s; if (xmin_s > xmin_i) xmin = xmin_i;
	ymin = ymin_s; if (ymin_s > ymin_i) ymin = ymin_i;
	zmin = zmin_s; if (zmin_s > zmin_i) zmin = zmin_i;
	xmax = xmax_s; if (xmax_s < xmax_i) xmax = xmax_i;
	ymax = ymax_s; if (ymax_s < ymax_i) ymax = ymax_i;
	zmax = zmax_s; if (zmax_s < zmax_i) zmax = zmax_i;

	Eigen::RowVector3d Vmin_temp(xmin, ymin, zmin);
	Eigen::RowVector3d Vmax_temp(xmax, ymax, zmax);
	//std::cout << "Boundary:\ntemp min: " << Vmin_temp << "\nmax temp:" << Vmax_temp << std::endl;
	//expand the bounding box
	const Eigen::RowVector3d delta_expand = boxExpand_ratio* (Vmax_temp - Vmin_temp);
	const Eigen::RowVector3d Vmin = Vmin_temp - delta_expand;
	const Eigen::RowVector3d Vmax = Vmax_temp + delta_expand;

	std::cout << "Boundary:\nmin: " << Vmin << "\nmax:" << Vmax << std::endl;
	const Eigen::RowVector3i res = (s * ((Vmax - Vmin) / (Vmax - Vmin).maxCoeff())).cast<int>();

	std::cout << "Voxel num:\n " << res << std::endl;

	// create grid
	std::cout << "Creating grid..." << std::endl;
	Eigen::MatrixXd GV(res(0) * res(1) * res(2), 3);
	for (int zi = 0; zi < res(2); zi++)
	{
		const auto lerp = [&](const int di, const int d)->double
		{return Vmin(d) + (double)di / (double)(res(d) - 1) * (Vmax(d) - Vmin(d)); };
		const double z = lerp(zi, 2);
		for (int yi = 0; yi < res(1); yi++)
		{
			const double y = lerp(yi, 1);
			for (int xi = 0; xi < res(0); xi++)
			{
				const double x = lerp(xi, 0);
				GV.row(xi + res(0) * (yi + res(1) * zi)) = Eigen::RowVector3d(x, y, z);
			}
		}
	}
	// compute values
	std::cout << "Computing distances..." << std::endl;
	Eigen::VectorXd B = Eigen::VectorXd::Zero(GV.rows());
	std::vector<bool> inModel(GV.rows());

#pragma omp parallel
	{
#pragma omp for  

		for (int i = 0; i < GV.rows(); i++) {
			Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
			for (int j = 0; j < 3; j++) { queryPnt[j] = GV.row(i)[j]; }
			//Fixed r_i
			//B[i] = _implicitDistance_Func(queryPnt) - C;
			//Variable r_i(brach NUM)
			//B[i] = _implicitDistance_Func_tree(queryPnt) - C;
			//Variable r_i(brach NUM + branch height)
			//B[i] = _implicitDistance_Func_tree2(queryPnt) - C;
			//Variable r_i(test)
			B[i] = _implicitDistance_Func_tree3(queryPnt) - C;//(good)

			inModel[i] = _checkSingleNode_inTETmodel(tet_model_Patch, queryPnt, ((Vmax - Vmin).maxCoeff() / s * 0.5));
		}
	}
	//std::cout << "B:\n" << B << std::endl;

	// compute center of mass
	Eigen::Vector3d CoM = Eigen::Vector3d::Zero();
	int CoM_Node_NUM = -1;
	for (int nodeIndex = 0; nodeIndex < GV.rows(); nodeIndex++) {
		if (B[nodeIndex] > 0.0) {
			for (int k = 0; k < 3; k++) { CoM[k] += GV.row(nodeIndex)[k]; }
			CoM_Node_NUM++;
		}
		if (inModel[nodeIndex] && (B[nodeIndex] < 0.0)) { //skip the overlap of support and initial gridNode
			for (int k = 0; k < 3; k++) { CoM[k] += GV.row(nodeIndex)[k]; }
			CoM_Node_NUM++;
		}
	}

	//for (int i = 0; i < GV.rows(); i++) {
	//	Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
	//	for (int j = 0; j < 3; j++) { queryPnt[j] = GV.row(i)[j]; }
	//	// decide the queryPnt is in the tet model
	//	bool isInsideTET = _checkSingleNode_inTETmodel(tet_model_Patch, queryPnt, ((Vmax - Vmin).maxCoeff()/s*0.5));
	//	if (isInsideTET && (B[i] < 0.0)) { //skip the overlap of support and initial gridNode
	//		for (int k = 0; k < 3; k++) { CoM[k] += GV.row(i)[k]; }
	//		CoM_Node_NUM++;
	//	}
	//}
	CoM = CoM / CoM_Node_NUM;

	// compute marching cube
	std::cout << "Marching cubes..." << std::endl;
	Eigen::MatrixXd BV; // surface Vertex table
	Eigen::MatrixXi BF; // surface Face table
	igl::copyleft::marching_cubes(B, GV, res(0), res(1), res(2), BV, BF);
	//igl::marching_cubes(B, GV, res(0), res(1), res(2), BV, BF);
	//std::cout << "BF"<< BF << std::endl;
	//std::cout << "VF" << BF << std::endl;

	// build new mesh from vertex table and face table
	float* nodeTable;
	nodeTable = (float*)malloc(sizeof(float) * BV.rows() * 3);
	for (int j = 0; j < BV.rows(); j++) {
		for (int i = 0; i < 3; i++) nodeTable[j * 3 + i] = (float)BV(j, i);
	}
	unsigned int* faceTable;
	faceTable = (unsigned int*)malloc(sizeof(unsigned int) * BF.rows() * 3);
	for (int j = 0; j < BF.rows(); j++) {
		for (int i = 0; i < 3; i++) faceTable[j * 3 + i] = BF(j,i);
	}

	QMeshPatch* surface = new QMeshPatch;
	surface->SetIndexNo(m_supportRaySet->GetMeshList().GetCount()); //index begin from 0
	m_supportRaySet->GetMeshList().AddTail(surface);
	surface->constructionFromVerFaceTable(BV.rows(), nodeTable, BF.rows(), faceTable);
	surface->drawThisPatch = true;

	//add CoM into Nodelist
	QMeshNode* CoM_Node = new QMeshNode;
	CoM_Node->SetCoord3D(CoM[0], CoM[1], CoM[2]);
	CoM_Node->isHighlight = true;
	CoM_Node->SetMeshPatchPtr(surface);
	CoM_Node->SetIndexNo(surface->GetNodeList().GetCount());//index begin from 0
	surface->GetNodeList().AddTail(CoM_Node);
	std::cout << "CoM Coordinate: " << CoM << std::endl;
		
	_output_SupportSurface(surface, "../DataSet/SUPPORT_SURFACE/", true);

	//std::cout << m_supportRaySet->GetMeshList().GetCount() << std::endl;

	//close ray display
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		// 0 - support ray; 1 - iso-surface from marching cube
		if (supportRay_Patch->GetIndexNo() == 0) supportRay_Patch->drawThisPatch = false;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void supportGeneration::_output_SupportSurface(QMeshPatch* isoSurface, std::string path, bool isRun) {

	if (isRun == false) return;

	double pp[3];
	path += (m_model_name + "_supportSurface.obj");
	std::ofstream nodeSelection(path);

	int index = 0;
	for (GLKPOSITION posNode = isoSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)isoSurface->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->SetIndexNo(index);
	}
	for (GLKPOSITION posFace = isoSurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(posFace);
		nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
	}
	nodeSelection.close();
	std::cout << "Output the support surface file: " << 
		m_model_name + "_supportSurface.obj" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void supportGeneration::compute_supportRayField() {

	bool adaptive_r_i_radius = true;

	// Step1: get the intersection Num between SupportRay and Sphere
	if (adaptive_r_i_radius) {
		std::cout << " --> adaptive r_i when calculate implicit face" <<std::endl;
#pragma omp parallel
		{
#pragma omp for  

			for (int i = 0; i < support_layers.size(); i++) {
				QMeshPatch* each_support_patch = support_layers[i];

				for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
					QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);

					Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
					node->GetCoord3D(queryPnt[0], queryPnt[1], queryPnt[2]);
					node->intersect_NUM = _Intersection_NUM(queryPnt);
				}
			}
		}

		/*************** TEST ***************/
		// output intersect_Num of each node of support Layer for test
		/*
		std::string path = "../DataSet/SUPPORT_SURFACE/intersect_NUM.txt";
		std::ofstream nodeSelection(path);

		for (int i = 0; i < support_layers.size(); i++) {
			QMeshPatch* each_support_patch = support_layers[i];

			for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
				QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);

				nodeSelection << node->intersect_NUM << std::endl;
			}
		}
		nodeSelection.close();
		std::cout << "Output the intersect_NUM of nodes on support surface" << std::endl;
		*/
		/*************** END ***************/
	}
	else{ std::cout << " --> fixed r_i when calculate implicit face" << std::endl; }
	// Step2: adaptive change the radius of SupportRay
#pragma omp parallel
	{
#pragma omp for  

		for (int i = 0; i < support_layers.size(); i++) {
			QMeshPatch* each_support_patch = support_layers[i];

			for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
				QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);

				Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
				node->GetCoord3D(queryPnt[0], queryPnt[1], queryPnt[2]);
				if (!adaptive_r_i_radius)
					//constant R
					node->implicitSurface_value_outer_Boundary = _implicitDistance_Func(queryPnt) - C;
				else
					//adaptive R
					node->implicitSurface_value_outer_Boundary = _implicitDistance_Func(queryPnt, node->intersect_NUM) - C;
			}

			//each_support_patch->drawSupportField = true;
		}
	}
	// Step3: re-get implicit filed value by using grid planes
	// get XY bounding box
	double xmin_all = 9999.9; double ymin_all = 9999.9; double xmax_all = -9999.9; double ymax_all = -9999.9;
	for (int i = 0; i < support_layers.size(); i++) {
		QMeshPatch* each_support_patch = support_layers[i];
		double xmin, ymin, zmin, xmax, ymax, zmax;
		each_support_patch->ComputeBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);

		if (xmin < xmin_all) xmin_all = xmin;
		if (ymin < ymin_all) ymin_all = ymin;
		if (xmax > xmax_all) xmax_all = xmax;
		if (ymax > ymax_all) ymax_all = ymax;
	}
	std::cout << " --> the X-Y range of the support layers of model is \n\t X:[" 
		<< xmin_all << " " << xmax_all << "], Y:[" << ymin_all << " " << ymax_all << "]\n";

	// plane Set: Ax + By + Cz + D = 0
	//			  1	   0    0    ?[xmin_all : n : xmax_all]
	//			  0    1    0    ?[ymin_all : n : ymax_all]
	Eigen::ArrayXXd x_planeSet = Eigen::ArrayXXd::Zero(num, 4);
	x_planeSet.col(3) = Eigen::ArrayXd::LinSpaced(num, xmin_all, xmax_all);
	x_planeSet.col(0) = Eigen::ArrayXd::LinSpaced(num, 1.0, 1.0);
	//std::cout << "x_planeSet:\n " << x_planeSet << std::endl;
	Eigen::ArrayXXd y_planeSet = Eigen::ArrayXXd::Zero(num, 4);
	y_planeSet.col(3) = Eigen::ArrayXd::LinSpaced(num, ymin_all, ymax_all);
	y_planeSet.col(1) = Eigen::ArrayXd::LinSpaced(num, 1.0, 1.0);
	//std::cout << "y_planeSet:\n " << y_planeSet << std::endl;
	Eigen::ArrayXXd planeSet(2 * num, 4);
	planeSet << x_planeSet, y_planeSet;
	//std::cout << "planeSet:\n " << planeSet << std::endl;

#pragma omp parallel
	{
#pragma omp for  

		for (int i = 0; i < support_layers.size(); i++) {
			QMeshPatch* each_support_patch = support_layers[i];

			for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
				QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);

				Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
				node->GetCoord3D(queryPnt[0], queryPnt[1], queryPnt[2]);
				
				//only change the Positive area into Negative
				if (node->implicitSurface_value_outer_Boundary > 0.0) {
					//std::cout << "orginal implicitSurface_value: " << node->implicitSurface_value << std::endl;
					node->implicitSurface_value_gridPlane_Boundary = _implicitDistance_Func(queryPnt, planeSet) - C;
					//std::cout << "after implicitSurface_value: " << node->implicitSurface_value << std::endl;
				}
			}
			//each_support_patch->drawSupportField = true;
		}
	}
	//// Step4: get inner Boundary implicit value;
	//for (int i = 0; i < support_layers.size(); i++) {
	//	QMeshPatch* each_support_patch = support_layers[i];
	//
	//	for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
	//		QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);
	//
	//		if (node->implicitSurface_value_outer_Boundary > 0.0) {
	//			
	//			node->implicitSurface_value_inner_Boundary = node->implicitSurface_value_outer_Boundary - 3*C;			
	//		}
	//	}
	//}
	
	// Step5: blend implicit filed value to form the final field value
	for (int i = 0; i < support_layers.size(); i++) {
		QMeshPatch* each_support_patch = support_layers[i];

		for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);

			node->implicitSurface_value = node->implicitSurface_value_outer_Boundary;

			// future use...
			// keep a ring of boundary by using C value
			if (node->implicitSurface_value_outer_Boundary > 1.7*C ) {
				node->implicitSurface_value = node->implicitSurface_value_gridPlane_Boundary;
			}
			// END
		}
		each_support_patch->drawSupportField = true;
	}

	
	//close ray display
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		// 0 - support ray; 1 - iso-surface from marching cube
		if (supportRay_Patch->GetIndexNo() == 0) supportRay_Patch->drawThisPatch = false;
		break;
	}
}

void supportGeneration::compute_Support_polyLlne_Field() {

	std::cout << " --> Implicit Distance Compute Running ..." << std::endl;
	long time = clock();


	for (int i = 0; i < support_layers.size(); i++) {
		QMeshPatch* each_support_patch = support_layers[i];

		std::vector<QMeshNode*> patch_NodeSet(each_support_patch->GetNodeNumber());

		int tempInt = 0;
		for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);
			patch_NodeSet[tempInt] = node;
			tempInt++;
		}


#pragma omp parallel
		{
#pragma omp for

			for (int j = 0; j < patch_NodeSet.size(); j++) {

				QMeshNode* node = patch_NodeSet[j];
				Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
				node->GetCoord3D(queryPnt[0], queryPnt[1], queryPnt[2]);

				//constant R
				node->implicitSurface_value = _implicitDistance_Func_small_ployline(queryPnt) - C;

			}
		}

		each_support_patch->drawSupportField = true;
		std::cout << ".";
		if ((i == support_layers.size() - 1) || ((i + 1) % 100 == 0)) std::cout << std::endl;

	}

	//#pragma omp parallel
	//	{
	//#pragma omp for  
	//
	//		for (int i = 0; i < support_layers.size(); i++) {
	//			QMeshPatch* each_support_patch = support_layers[i];
	//
	//			for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
	//				QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);
	//
	//				Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
	//				node->GetCoord3D(queryPnt[0], queryPnt[1], queryPnt[2]);
	//				
	//				//constant R
	//				node->implicitSurface_value = _implicitDistance_Func_small_ployline(queryPnt) - C;
	//				
	//			}
	//			each_support_patch->drawSupportField = true;
	//
	//			//DEBUG
	//			//if (i >= 10) break;
	//			std::cout << "layer : " << i << std::endl;
	//			//END
	//		}
	//	}

		//close ray display
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		// 0 - support ray; 1 - iso-surface from marching cube
		if (supportRay_Patch->GetIndexNo() == 0) supportRay_Patch->drawThisPatch = false;
		break;
	}

	std::printf(" --> Solve takes %ld ms.\n", clock() - time);

}

void supportGeneration::compute_Support_tree_Field() {

	std::cout << " --> Implicit Distance Compute Running ..." << std::endl;
	long time = clock();


	for (int i = 0; i < support_layers.size(); i++) {
		QMeshPatch* each_support_patch = support_layers[i];

		std::vector<QMeshNode*> patch_NodeSet(each_support_patch->GetNodeNumber());

		int tempInt = 0;
		for (GLKPOSITION posNode = each_support_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(posNode);
			patch_NodeSet[tempInt] = node;
			tempInt++;
		}


#pragma omp parallel
		{
#pragma omp for

			for (int j = 0; j < patch_NodeSet.size(); j++) {

				QMeshNode* node = patch_NodeSet[j];
				Eigen::Vector3d queryPnt = Eigen::Vector3d::Zero();
				node->GetCoord3D(queryPnt[0], queryPnt[1], queryPnt[2]);

				//constant R
				node->implicitSurface_value = _implicitDistance_Func_tree3(queryPnt) - C;

			}
		}

		each_support_patch->drawSupportField = true;
		std::cout << ".";
		if ((i == support_layers.size() - 1) || ((i + 1) % 100 == 0)) std::cout << std::endl;

	}

		//close ray display
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		// 0 - support ray; 1 - iso-surface from marching cube
		if (supportRay_Patch->GetIndexNo() == 0) supportRay_Patch->drawThisPatch = false;
		break;
	}

	std::printf(" --> Solve takes %ld ms.\n", clock() - time);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void supportGeneration::build_tight_supportLayers() {

	//clean the single or two negative point
	for (int layerInd = 0; layerInd < support_layers.size(); layerInd++) {
		QMeshPatch* each_support_patch = support_layers[layerInd];

		//// node loop
		for (GLKPOSITION Pos = each_support_patch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(Pos);

			if (Node->implicitSurface_value > 0.0) continue;

			// node one ring loop
			int neighbor_positive_nodeNum = 0;
			for (GLKPOSITION Pos_neighbor = Node->GetEdgeList().GetHeadPosition(); Pos_neighbor;) {
				QMeshEdge* oneRing_Edge = (QMeshEdge*)Node->GetEdgeList().GetNext(Pos_neighbor);

				QMeshNode* Node_1ring = oneRing_Edge->GetStartPoint();
				if(Node_1ring == Node) Node_1ring = oneRing_Edge->GetEndPoint();

				if (Node_1ring->implicitSurface_value > 0.0) neighbor_positive_nodeNum++;
			}

			if (neighbor_positive_nodeNum == Node->GetEdgeNumber()
				|| neighbor_positive_nodeNum == (Node->GetEdgeNumber() - 1)) {

				Node->implicitSurface_value = 1.0;
			}
		}
	}

	double cut_isoValue = 0.0; // cutting occurring on the implicit surface

	for (int layerInd = 0; layerInd < support_layers.size(); layerInd++) {
		QMeshPatch* each_support_patch = support_layers[layerInd];

		/* get the VERTEX NUM of tight support Layer */
		int structuredMesh_NodeNum = 0;
		//// node loop
		for (GLKPOSITION Pos = each_support_patch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(Pos);

			Node->implicitSurface_cut_index = -1;//reset 

			if (Node->implicitSurface_value > 0) {

				Node->implicitSurface_cut_index = structuredMesh_NodeNum; // start from 0
				structuredMesh_NodeNum++;
			}
		}
		//// edge loop
		for (GLKPOSITION Pos = each_support_patch->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* Edge = (QMeshEdge*)each_support_patch->GetEdgeList().GetNext(Pos);

			double a = Edge->GetStartPoint()->implicitSurface_value;
			double b = Edge->GetEndPoint()->implicitSurface_value;

			if ((cut_isoValue - a) * (cut_isoValue - b) < 0.0) {
				double alpha = (cut_isoValue - a) / (b - a);
				double p1[3], p2[3], pp[3];
				Edge->GetStartPoint()->GetCoord3D(p1[0], p1[1], p1[2]);
				Edge->GetEndPoint()->GetCoord3D(p2[0], p2[1], p2[2]);

				for (int j = 0; j < 3; j++) {
					//compute the position for cut_isoValue
					pp[j] = (1.0 - alpha) * p1[j] + alpha * p2[j];
				}

				QMeshNode* cut_Node = new QMeshNode;
				cut_Node->cutNode_related_LayerEdge = Edge;
				cut_Node->implicitSurface_value = cut_isoValue;

				cut_Node->SetCoord3D(pp[0], pp[1], pp[2]);
				//cutNode_index should increase based on the "structuredMesh_NodeNum"
				cut_Node->implicitSurface_cut_index = structuredMesh_NodeNum;
				structuredMesh_NodeNum++;

				//install this cutNode to its Edge
				Edge->installed_CutNode = cut_Node;
				Edge->isLocate_CutNode_layerEdge = true;
			}
		}

		/* get the VERTEX Table of tight support Layer */
		if (structuredMesh_NodeNum == 0) continue;// avoid the meanless compute
		Eigen::MatrixXd V = Eigen::MatrixXd::Zero(structuredMesh_NodeNum, 3);
		//// node loop
		for (GLKPOSITION Pos = each_support_patch->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)each_support_patch->GetNodeList().GetNext(Pos);

			if (Node->implicitSurface_cut_index >= 0) { // VERTEX needed to be kept

				double xx, yy, zz;
				Node->GetCoord3D(xx, yy, zz);
				V.row(Node->implicitSurface_cut_index) << xx, yy, zz;
			}
		}
		//// edge loop
		for (GLKPOSITION Pos = each_support_patch->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* Edge = (QMeshEdge*)each_support_patch->GetEdgeList().GetNext(Pos);

			if (Edge->isLocate_CutNode_layerEdge) {

				double xx, yy, zz;
				Edge->installed_CutNode->GetCoord3D(xx, yy, zz);
				V.row(Edge->installed_CutNode->implicitSurface_cut_index) << xx, yy, zz;

			}
		}

		/* get the FACE NUM of tight support Layer */
		int structuredMesh_FaceNum = 0;
		//// face loop
		for (GLKPOSITION Pos = each_support_patch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)each_support_patch->GetFaceList().GetNext(Pos);

			int positive_Num = 0;
			for (int k = 0; k < 3; k++) {
				if (Face->GetNodeRecordPtr(k)->implicitSurface_value > 0) {
					positive_Num++;
				}
			}

			if (positive_Num < 0 || positive_Num > 3)
				std::cout << "ERROR: the NUM of point with positive implicite surface field value is out of range\n";

			if (positive_Num == 3) {
				structuredMesh_FaceNum = structuredMesh_FaceNum + 1;
				Face->faceType = KEEP;
			}
			else if (positive_Num == 2) {
				structuredMesh_FaceNum = structuredMesh_FaceNum + 2;
				Face->faceType = CUT_2;
			}
			else if (positive_Num == 1) {
				structuredMesh_FaceNum = structuredMesh_FaceNum + 1;
				Face->faceType = CUT_1;
			}
			else {
				Face->faceType = DISCARD;
			}
		}// END: get the FACE NUM of tight support Layer

		/* get the FACE Table of tight support Layer */
		Eigen::MatrixXi F = Eigen::MatrixXi::Zero(structuredMesh_FaceNum, 3);
		// face loop
		// collect the 3+ faces (the implicit_cut_values of 3 vertexes of one face are all large than 0)
		int F_row_index = 0;
		for (GLKPOSITION Pos = each_support_patch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)each_support_patch->GetFaceList().GetNext(Pos);

			if (Face->faceType == KEEP) {

				for (int k = 0; k < 3; k++) {
					F(F_row_index, k) = Face->GetNodeRecordPtr(k)->implicitSurface_cut_index;
				}
				F_row_index++;
			}
			else if (Face->faceType == CUT_2) {

				int baseIndex = -1;// record the index of edge without cut_Node --> range: [0-2]
				for (int k = 0; k < 3; k++) {
					if (!Face->GetEdgeRecordPtr(k + 1)->isLocate_CutNode_layerEdge) {
						baseIndex = k;
						break;
					}
				}
				if (baseIndex == -1) std::cout << "ERROR: cannot find the edge without cut_Node" << std::endl;

				// the 1st added face
				F(F_row_index, 0) = Face->GetNodeRecordPtr(baseIndex)->implicitSurface_cut_index;
				F(F_row_index, 1) = Face->GetNodeRecordPtr((baseIndex + 1) % 3)->implicitSurface_cut_index;
				F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F_row_index++;

				// the 2nd added face
				F(F_row_index, 0) = Face->GetNodeRecordPtr(baseIndex)->implicitSurface_cut_index;
				F(F_row_index, 1) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 2) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F_row_index++;
			}
			else if (Face->faceType == CUT_1) {

				int baseIndex = -1;// record the index of edge without cut_Node --> range: [0-2]
				for (int k = 0; k < 3; k++) {
					if (!Face->GetEdgeRecordPtr(k + 1)->isLocate_CutNode_layerEdge) {
						baseIndex = k;
						break;
					}
				}
				if (baseIndex == -1) std::cout << "ERROR: cannot find the edge without cut_Node." << std::endl;
				
				F(F_row_index, 0) = Face->GetNodeRecordPtr((baseIndex + 2) % 3)->implicitSurface_cut_index;
				F(F_row_index, 1) = Face->GetEdgeRecordPtr((baseIndex + 2) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F(F_row_index, 2) = Face->GetEdgeRecordPtr((baseIndex + 1) % 3 + 1)->installed_CutNode->implicitSurface_cut_index;
				F_row_index++;
			
			}
			else if (Face->faceType == DISCARD) {
				// do nothing
			}
			else {
				std::cout << "ERROR Face Type !" << std::endl;
			}
				
		}

		// build new mesh from vertex table and face table
		float* nodeTable;
		nodeTable = (float*)malloc(sizeof(float) * V.rows() * 3);
		for (int j = 0; j < V.rows(); j++) {
			for (int i = 0; i < 3; i++) nodeTable[j * 3 + i] = (float)V(j, i);
		}
		unsigned int* faceTable;
		faceTable = (unsigned int*)malloc(sizeof(unsigned int) * F.rows() * 3);
		for (int j = 0; j < F.rows(); j++) {
			for (int i = 0; i < 3; i++) faceTable[j * 3 + i] = F(j, i);
		}

		QMeshPatch* surface = new QMeshPatch;
		surface->SetIndexNo(m_tight_supportLayerSet->GetMeshList().GetCount()); //index begin from 0
		m_tight_supportLayerSet->GetMeshList().AddTail(surface);
		surface->constructionFromVerFaceTable(V.rows(), nodeTable, F.rows(), faceTable);

		//smoothing layers // this method will cause some problems
		//for (int i = 0; i < 5; i++) _smoothingIsoSurface();

		surface->drawThisPatch = true;
		surface->is_SupportLayer = true;
		surface->largeLayer_Index = support_layers[layerInd]->largeLayer_Index;
	}
}

// will cause topology defect, so it is not be used
void supportGeneration::_smoothingIsoSurface() {

	for (GLKPOSITION posMesh = m_tight_supportLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* isoLayer = (QMeshPatch*)m_tight_supportLayerSet->GetMeshList().GetNext(posMesh);

		for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* Node = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

			Node->SetAttribFlag(0, false);
		}

		for (GLKPOSITION Pos = isoLayer->GetEdgeList().GetHeadPosition(); Pos;) {
			QMeshEdge* thisEdge = (QMeshEdge*)isoLayer->GetEdgeList().GetNext(Pos);
			if (thisEdge->IsBoundaryEdge()) {
				thisEdge->GetStartPoint()->SetAttribFlag(0, true);
				thisEdge->GetEndPoint()->SetAttribFlag(0, true);
			}
		}

		//laplacian smoothness
		for (GLKPOSITION Pos = isoLayer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoLayer->GetNodeList().GetNext(Pos);

			if (thisNode->GetAttribFlag(0)) continue;
			else {
				double pp[3] = { 0 }; int neighNum = 0;
				for (GLKPOSITION Pos = thisNode->GetEdgeList().GetHeadPosition(); Pos;) {
					QMeshEdge* neighEdge = (QMeshEdge*)thisNode->GetEdgeList().GetNext(Pos);

					QMeshNode* neighNode = neighEdge->GetStartPoint();
					if (neighNode == thisNode) neighNode = neighEdge->GetEndPoint();

					double p1[3];
					neighNode->GetCoord3D(p1[0], p1[1], p1[2]);

					for (int i = 0; i < 3; i++) pp[i] += p1[i];
					neighNum++;
				}
				for (int i = 0; i < 3; i++) pp[i] /= neighNum;
				thisNode->SetCoord3D(pp[0], pp[1], pp[2]);
			}
		}

	}
}
// will cause low speed, so it is not be used
void supportGeneration::_generate_normalSet_for_supportDetection(Eigen::Vector3d dir, std::vector<Eigen::Vector3d> dir_set) {

	int div_circle = 60;
	double self_support_angle = 15.0;//deg
	double rad_XtiltAngle = DEGREE_TO_ROTATE(self_support_angle);
	dir_set.resize(360 / div_circle);
	int i = 0;
	for (int ZrotateAngle = 0; ZrotateAngle < 360; ZrotateAngle = ZrotateAngle + div_circle) {
		
		double rad_ZrotateAngle = DEGREE_TO_ROTATE(ZrotateAngle);		
		Eigen::Vector3d candidateNor = _calCandidateNormal(dir, rad_ZrotateAngle, rad_XtiltAngle);
		//cout << "candidateNor:\n " << candidateNor << endl;
		dir_set[i] = candidateNor; i++;
	}
}
// will cause low speed, so it is not be used
Eigen::Vector3d supportGeneration::_calCandidateNormal(Eigen::Vector3d normal, double rad_ZrotateAngle, double rad_XtiltAngle) {

	Eigen::Matrix3d Xrot_Matrix, Zrot_Matrix, Xback_Matrix, Zback_Matrix;
	Eigen::Vector2d normalXY_temp;
	Eigen::Vector3d candidateNormal, candidateNor_temp;

	normalXY_temp << normal(0), normal(1);// (nx, ny)
	double alpha = atan2(normal(0), normal(1));// normal Z rotate
	double beta = atan2(normalXY_temp.norm(), normal(2));// normal X rotate
	Xback_Matrix << 1, 0, 0, 0, cos(-beta), -sin(-beta), 0, sin(-beta), cos(-beta);
	Zback_Matrix << cos(-alpha), -sin(-alpha), 0, sin(-alpha), cos(-alpha), 0, 0, 0, 1;

	Xrot_Matrix << 1, 0, 0, 0, cos(rad_XtiltAngle), -sin(rad_XtiltAngle), 0, sin(rad_XtiltAngle), cos(rad_XtiltAngle);
	Zrot_Matrix << cos(rad_ZrotateAngle), -sin(rad_ZrotateAngle), 0, sin(rad_ZrotateAngle), cos(rad_ZrotateAngle), 0, 0, 0, 1;
	candidateNor_temp = (Zrot_Matrix * Xrot_Matrix).col(2);// extract last vector of Z direction

	candidateNormal = Zback_Matrix * Xback_Matrix * candidateNor_temp;
	return candidateNormal;
}

double supportGeneration::_func_adaptive_ratio(int intersect_NUM) {

	/*  ^ r_i (energy radius)
	*   |___ a
	*	|	|\
	*	|	| \
	*   |	|  \
	*   |_ _|_ b\____
	*	|___|___|____>NUM
	*	0   
	*/

	double r_i = 0.0;

	if (intersect_NUM < (int)a[0])
		r_i = a[1];
	else if (intersect_NUM < (int)b[0])
		r_i = (b[1] - a[1]) / (b[0] - a[0]) * (intersect_NUM - a[0]) + a[1];
	else 
		r_i = b[1];

	if (r_i == 0.0) {
		std::cout << "ERROR:r_i should not be ZERO!! " << std::endl; 
		r_i = 0.5 * (a[1] + b[1]);
	}
	//std::cout << "_" << r_i << "_" << std::endl;
	return r_i;
}

void supportGeneration::markSupportFace() {

	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();
	for (GLKPOSITION PosFace = (tet_Model->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((tet_Model->GetFaceList()).GetNext(PosFace));

		face->needSupport = false;//reset
		
		if (face->inner == true) continue;// the inner face is not considered 

		//get TET element
		QMeshTetra* surface_tetElement = NULL;
		if (face->GetLeftTetra() == NULL) surface_tetElement = face->GetRightTetra();
		else surface_tetElement = face->GetLeftTetra();

		//test 
		if (surface_tetElement == NULL) std::cout << "ERROR: no need TET element of surface\n";

		Eigen::Vector3d n;// tansform into vector
		for (int i = 0; i < 3; i++) {n[i] = face->m_desiredNormal[i];}

		//std::cout << surface_tetElement->vectorField << std::endl;
		//surface_tetElement->vectorField << 0, 0, 1;
		//mark risk face
		if ((n.dot(surface_tetElement->vectorField) + sin(DEGREE_TO_ROTATE(tau))) < 0) { 

			face->needSupport = true;
		}
	}

	//record node which need support
	//reset flag (needsupport) of node 
	for (GLKPOSITION PosNode = (tet_Model->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
		QMeshNode* node = (QMeshNode*)((tet_Model->GetNodeList()).GetNext(PosNode));
		node->need_Support == false;
	}
	for (GLKPOSITION PosFace = (tet_Model->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((tet_Model->GetFaceList()).GetNext(PosFace));

		if (face->needSupport == true) {
			for (int i = 0; i < 3; i++)
				face->GetNodeRecordPtr(i)->need_Support = true;
		}
	}
	tau = 30; //resume 30 to build tree;
}

void supportGeneration::_cal_Ray_direction(QMeshPatch* tet_Model) {

	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);

		if (Node->inner) {continue;}	// the inner face is not considered 
		if (Node->need_Support == false) continue;

		// get average Vector field of support node
		Eigen::Vector3d averageField = { 0.0,0.0,1.0 };
		for (GLKPOSITION node_neighborTet_Pos = Node->GetTetraList().GetHeadPosition(); node_neighborTet_Pos;) {
			QMeshTetra* ConnectTetra = (QMeshTetra*)Node->GetTetraList().GetNext(node_neighborTet_Pos);

			averageField += ConnectTetra->vectorField;
		}
		Node->supportRay_Dir = -averageField.normalized();// reverse direction for projecting ray onto the platform
	}
}

/* This function is useless for mesh with Genus*/
void supportGeneration::clearRay_insideModelTET_vertical() {

	int timer = 0;

	QMeshPatch* raySet = (QMeshPatch*)m_supportRaySet->GetMeshList().GetHead();
	for (GLKPOSITION Pos = raySet->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
		QMeshEdge* ray = (QMeshEdge*)(raySet->GetEdgeList().GetNext(Pos));

		Eigen::Vector3d midCoord3D, origCoord3D, targetCoord3D;
		ray->GetStartPoint()->GetCoord3D(origCoord3D[0], origCoord3D[1], origCoord3D[2]);
		ray->GetEndPoint()->GetCoord3D(targetCoord3D[0], targetCoord3D[1], targetCoord3D[2]);

		midCoord3D = 0.5 * (origCoord3D + targetCoord3D);

		if (_inside_TET_Shell(midCoord3D)) {// middle point of ray inside the tet model?

			timer++;
			ray->GetEndPoint()->isUseful_Node_SupportRay = false;
			ray->GetStartPoint()->isUseful_Node_SupportRay = false;
		}
	}
	std::cout <<  timer << std::endl;
	std::cout << "clearRay_insideModelTET_vertical finish! " << std::endl;
}
/* This function is useless for mesh with Genus also*/
bool supportGeneration::_inside_TET_Shell(Eigen::Vector3d midCoord3D) {

	bool is_inside = true;
	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();

	for (GLKPOSITION PosFace = (tet_Model->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
		QMeshFace* face = (QMeshFace*)((tet_Model->GetFaceList()).GetNext(PosFace));

		if (face->inner == true) continue;// the inner face is not considered 

		Eigen::Vector3d faceNormal; double D;
		//face->CalPlaneEquation();
		face->GetPlaneEquation(faceNormal[0], faceNormal[1], faceNormal[2], D);

		double dist = midCoord3D.dot(faceNormal) + D;

		if (dist < 0.0) { is_inside = false; break; }
	}

	return is_inside;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// calculate boundary toolpath for support layers
void supportGeneration::toolPathCompute_support() {

	std::cout << "Support ToolPath Compute Running ..." << std::endl;
	long time = clock();

	int layerNum = m_tight_supportLayerSet->GetMeshList().GetCount();
	std::cout << " --> Support layerNum: " << layerNum << std::endl;
	std::vector<QMeshPatch*> toolpathVector(layerNum);

#pragma omp parallel
	{
#pragma omp for
		for (int omptime = 0; omptime < CPUCoreNum; omptime++) {
			for (GLKPOSITION posMesh = m_tight_supportLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch* layer = (QMeshPatch*)m_tight_supportLayerSet->GetMeshList().GetNext(posMesh);

				if (layer->GetIndexNo() % CPUCoreNum != omptime) continue;
				//std::cout << "--> Toolpath " << layer->GetIndexNo() << " Generation Start" << std::endl;
				/*DEBUG - use OK */
				if (layer->GetIndexNo() == 64 && m_model_name == "topo_cut") {
					/* ---- multi_source Dijkstra distance field ---- */
					BoundaryFieldCalculation* distField_layer = new BoundaryFieldCalculation();
					//distField_layer->meshRefinement(layer);
					distField_layer->GenerateBndDistMap2(layer);
					delete distField_layer;
					/* ---- END ---- */
				}
				else {
					/* ---- Generate boundary heat field ---- */
					heatMethodField* heatField_layer = new heatMethodField(layer);
					heatField_layer->meshRefinement();
					heatField_layer->compBoundaryHeatKernel();
					delete heatField_layer;
					/* ---- END ---- */
				}
				/*END*/

				for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
					QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);
					// check the result of Boundary field from HeatMethod, as there may be some NAN! _(:з」∠)_
					if (!isfinite(Node->boundaryValue))
						std::cout << " error: Node index: " << Node->GetIndexNo()
						<< " Dist = " << Node->boundaryValue
						<< " Support layer = " << layer->GetIndexNo() << std::endl;
				}

				/* ---- Generate boundary Toolpath ---- */
				toolpathGeneration* ToolPathComp_layer = new toolpathGeneration(layer, 0.8, 1);
				QMeshPatch* singlePath = ToolPathComp_layer->generateBundaryToolPath();
				ToolPathComp_layer->resampleToolpath(singlePath);
				if (singlePath != NULL) {
					singlePath->largeLayer_Index = layer->largeLayer_Index;
					singlePath->attached_layer_for_toolpath = layer;
				}
				/* ---- END ---- */

				toolpathVector[layer->GetIndexNo()] = singlePath;

				//std::cout << "--> Toolpath " << layer->GetIndexNo() << " Generation Finish" << std::endl;
				//std::printf("------------------------------------------------\n");
			}
		}
	}

	for (int i = 0; i < layerNum; i++) {
		//std::cout << "toolpathVector[i]" << toolpathVector[i] << std::endl;
		if (toolpathVector[i] != NULL) {
			QMeshPatch* singlePath = toolpathVector[i];
			singlePath->SetIndexNo(i);
			m_toolpathSet_support->GetMeshList().AddTail(singlePath);
		}
	}

	std::printf(" --> Solve takes %ld ms.\n", clock() - time);
}

// calculate boundary toolpath for initial layers
void supportGeneration::toolPathCompute_initial() {

	std::cout << "Initial ToolPath Compute Running ..." << std::endl;
	long time = clock();

	int layerNum = initial_layers.size();
	std::cout << " --> Initial layerNum: " << layerNum << std::endl;
	std::vector<QMeshPatch*> toolpathVector(layerNum);

#pragma omp parallel
	{
#pragma omp for

		for (int i = 0; i < initial_layers.size(); i++) {
			/* ---- Generate boundary heat field ---- */
			heatMethodField* heatField_layer = new heatMethodField(initial_layers[i]);
			heatField_layer->meshRefinement();
			heatField_layer->compBoundaryHeatKernel();
			delete heatField_layer;
			/* ---- END ---- */

			//check the heatmethod value
			for (GLKPOSITION Pos = initial_layers[i]->GetNodeList().GetHeadPosition(); Pos;) {
				QMeshNode* Node = (QMeshNode*)initial_layers[i]->GetNodeList().GetNext(Pos);
				// check the result of Boundary field from HeatMethod, as there may be some NAN! _(:з」∠)_
				if (!isfinite(Node->boundaryValue))
					std::cout << " error: Node index: " << Node->GetIndexNo() << " Dist = "
					<< Node->boundaryValue << std::endl;
			}

			/* ---- Generate boundary Toolpath ---- */
			toolpathGeneration* ToolPathComp_layer = new toolpathGeneration(initial_layers[i], 0.6, 0.8);
			QMeshPatch* singlePath = ToolPathComp_layer->generateBundaryToolPath();
			ToolPathComp_layer->resampleToolpath(singlePath);
			if (singlePath != NULL) { 
				singlePath->largeLayer_Index = initial_layers[i]->largeLayer_Index; 
				singlePath->attached_layer_for_toolpath = initial_layers[i];
			}
			/* ---- END ---- */

			toolpathVector[i] = singlePath;
		}
	}

	for (int i = 0; i < layerNum; i++) {
		//std::cout << "toolpathVector[i]" << toolpathVector[i] << std::endl;
		if (toolpathVector[i] != NULL) {
			QMeshPatch* singlePath = toolpathVector[i];
			singlePath->SetIndexNo(i);
			m_toolpathSet_initial->GetMeshList().AddTail(singlePath);
		}
	}

	std::printf(" --> Solve takes %ld ms.\n", clock() - time);
}

// get toolpath output list
void supportGeneration::_get_toolPath_list_output(std::vector<QMeshPatch*>& toolpath_list) {

	//get the maxNum of large layer <--based on envolop support input(estimation, the real situation is less than it)
	int large_layer_NUM = support_layers.size();
	std::cout << "Max large layer num = " << large_layer_NUM << std::endl;

	bool initial_layer_1st = false;

	for (int i = 0; i < large_layer_NUM; i++) {

		QMeshPatch* initial_toolpath_patch = NULL;
		QMeshPatch* support_toolpath_patch = NULL;

		for (GLKPOSITION posMesh = m_toolpathSet_initial->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* layer = (QMeshPatch*)m_toolpathSet_initial->GetMeshList().GetNext(posMesh);

			if (layer->largeLayer_Index == i) {
				initial_toolpath_patch = layer;
				break;
			}
		}

		for (GLKPOSITION posMesh = m_toolpathSet_support->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
			QMeshPatch* layer = (QMeshPatch*)m_toolpathSet_support->GetMeshList().GetNext(posMesh);

			if (layer->largeLayer_Index == i) {
				support_toolpath_patch = layer;
				break;
			}
		}

		if (initial_toolpath_patch == NULL && support_toolpath_patch != NULL) {//only support

				toolpath_list.push_back(support_toolpath_patch); 
		}
		else if (initial_toolpath_patch != NULL && support_toolpath_patch != NULL) { // all

			if (initial_layer_1st) {
				toolpath_list.push_back(initial_toolpath_patch);
				toolpath_list.push_back(support_toolpath_patch);
			}
			else {
				toolpath_list.push_back(support_toolpath_patch);
				toolpath_list.push_back(initial_toolpath_patch);
			}
			initial_layer_1st = !initial_layer_1st;

		}
		else if (initial_toolpath_patch != NULL && support_toolpath_patch == NULL) { // only initial
			toolpath_list.push_back(initial_toolpath_patch);
		}
		else {
			//std::cout <<"initial and support toolpath are all NULL, please check!!" << std::endl;
		}	
	}
}

// output toolpath
void supportGeneration::output_toolpath() {

	std::vector<QMeshPatch*> toolpath_list;
	_get_toolPath_list_output(toolpath_list);

	std::cout << "There are " << toolpath_list.size() << " toolpath." << std::endl;

	for (int i = 0; i < toolpath_list.size(); i++) {

		QMeshPatch* each_toolpath = toolpath_list[i];

		std::string TOOLPATH_waypoint_dir;
		if(toolpath_list[i]->attached_layer_for_toolpath->is_SupportLayer)
			TOOLPATH_waypoint_dir = "../DataSet/TOOL_PATH/waypoint/" + std::to_string(i) + "S.txt";
		else
			TOOLPATH_waypoint_dir = "../DataSet/TOOL_PATH/waypoint/" + std::to_string(i) + ".txt";

		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;

		std::ofstream toolpathFile(TOOLPATH_waypoint_dir);

		double pp[3]; double n[3]; 
		QMeshEdge* sEdge = (QMeshEdge*)each_toolpath->GetEdgeList().GetHead(); // the first Edge of Toolpath
		QMeshNode* sNode = sEdge->GetStartPoint();
		sNode->GetCoord3D(pp); sNode->GetNormal(n[0], n[1], n[2]);
		toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;

		for (GLKPOSITION posEdge = each_toolpath->GetEdgeList().GetHeadPosition(); posEdge != nullptr;) {
			QMeshEdge* Edge = (QMeshEdge*)each_toolpath->GetEdgeList().GetNext(posEdge);

			//std::cout << "start Node " << Edge->GetStartPoint()->GetIndexNo() << " end Node " << Edge->GetEndPoint()->GetIndexNo() << std::endl;

			QMeshNode* eNode = Edge->GetStartPoint();
			if (eNode == sNode) eNode = Edge->GetEndPoint();

			eNode->GetCoord3D(pp); eNode->GetNormal(n[0], n[1], n[2]);
			toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;
		}
		toolpathFile.close();
	}
	std::cout << "Output waypoint finish" << std::endl;

	//output layers
	for (int i = 0; i < toolpath_list.size(); i++) {
	
		QMeshPatch* each_layer = toolpath_list[i]->attached_layer_for_toolpath;

		std::string TOOLPATH_layer_dir;

		if(each_layer->is_SupportLayer)
			TOOLPATH_layer_dir = "../DataSet/TOOL_PATH/layer/" + std::to_string(i) + "S.obj";
		else
			TOOLPATH_layer_dir = "../DataSet/TOOL_PATH/layer/" + std::to_string(i) + ".obj";

		std::ofstream toolpathFile(TOOLPATH_layer_dir);

		double pp[3];

		int index = 0;
		for (GLKPOSITION posNode = each_layer->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)each_layer->GetNodeList().GetNext(posNode);
			node->GetCoord3D(pp);
			toolpathFile << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
			index++; node->SetIndexNo(index);
		}
		for (GLKPOSITION posFace = each_layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)each_layer->GetFaceList().GetNext(posFace);
			toolpathFile << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
				<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
				<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
		}
		toolpathFile.close();
	}

	std::cout << "Output layers finish" << std::endl;
}

void supportGeneration::_get_largeLayer_list(std::vector<std::vector<QMeshPatch*>>& largeLayer_matrix) {

	//get maximum largeLayer_Index
	int max_Layer_NUM = 0;

	for (GLKPOSITION posMesh = m_layers->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* Patch = (QMeshPatch*)m_layers->GetMeshList().GetNext(posMesh);

		if (Patch->largeLayer_Index > max_Layer_NUM)
			max_Layer_NUM = Patch->largeLayer_Index;
	}

	largeLayer_matrix.resize(max_Layer_NUM + 1);

	for (int i = 0; i < (max_Layer_NUM + 1); i++) {

		QMeshPatch* initial_layer_patch = NULL;
		QMeshPatch* support_layer_patch = NULL;

		for (int j = 0; j < initial_layers.size(); j++) {

			if (initial_layers[j]->largeLayer_Index == i) {
				initial_layer_patch = initial_layers[j];
				break;
			}
		}

		for (int k = 0; k < support_layers.size(); k++) {		

			if (support_layers[k]->largeLayer_Index == i) {
				support_layer_patch = support_layers[k];
				break;
			}
		}

		if (initial_layer_patch == NULL && support_layer_patch != NULL) {		// only support

			largeLayer_matrix[i].push_back(support_layer_patch);
		}
		else if (initial_layer_patch != NULL && support_layer_patch != NULL) {	// all
		
			largeLayer_matrix[i].push_back(initial_layer_patch);
			largeLayer_matrix[i].push_back(support_layer_patch);
		}
		else if (initial_layer_patch != NULL && support_layer_patch == NULL) {	// only initial
			largeLayer_matrix[i].push_back(initial_layer_patch);
		}
		else {
			//std::cout <<"initial and support layer are all NULL, please check!!" << std::endl;
		}
	}
}

//// ------------------------------- useless
//bool supportGeneration::_support_by_1st_below_initialLayer_normal
//(QMeshNode* top_node, QMeshPatch* below_initial_layer) 
//{
//	if (below_initial_layer == NULL) return false;
//
//	bool support_by_1st_below_initialLayer = false;
//
//	Eigen::Vector3d insertP, v1, v2, v3;
//	Eigen::Vector3d orig;	top_node->GetCoord3D(orig(0), orig(1), orig(2));
//	Eigen::Vector3d dir;	for (int k = 0; k < 3; k++) { dir[k] = top_node->m_desiredNormal[k]; }
//
//	for (GLKPOSITION Pos = below_initial_layer->GetFaceList().GetHeadPosition(); Pos;) {
//		QMeshFace* Face = (QMeshFace*)below_initial_layer->GetFaceList().GetNext(Pos);
//
//		// can be speeding up by AABB, do at next step;
//
//		Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
//		Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
//		Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));
//
//		//original normal detection
//		if (_intersetTriangle(orig, dir, v1, v2, v3, insertP)) {
//			support_by_1st_below_initialLayer = true;
//			break;
//		}
//	}
//	return support_by_1st_below_initialLayer;
//}
//
//// ------------------------------- useless
//bool supportGeneration::_support_by_1st_below_initialLayer_incline
//(QMeshNode* top_node, QMeshPatch* below_initial_layer)
//{
//	if (below_initial_layer == NULL) return false;
//
//	bool support_by_1st_below_initialLayer = false;
//
//	Eigen::Vector3d insertP, v1, v2, v3;
//	Eigen::Vector3d orig;	top_node->GetCoord3D(orig(0), orig(1), orig(2));
//	Eigen::Vector3d dir;	for (int k = 0; k < 3; k++) { dir[k] = top_node->m_desiredNormal[k]; }
//
//	//incline a self-supporting angle to the z direction
//	Eigen::Vector3d descend_dir = { 0.0,0.0,-1.0 };
//	Eigen::Vector3d rotateAxis = dir.cross(descend_dir);
//
//	//angle between _dir and _descend_dir [0,PI]
//	double radian_angle = atan2(dir.cross(descend_dir).norm(), dir.transpose() * descend_dir);
//	if (ROTATE_TO_DEGREE(radian_angle) < tau) dir = descend_dir; // in the range of support angle, directly downward
//	else {
//		// only rotate a support angle
//		Eigen::AngleAxisf V1(DEGREE_TO_ROTATE(tau), rotateAxis);//以（0,0,1）为旋转轴，旋转45度
//		dir = V1.matrix() * dir;
//	}
//
//	for (GLKPOSITION Pos = below_initial_layer->GetFaceList().GetHeadPosition(); Pos;) {
//		QMeshFace* Face = (QMeshFace*)below_initial_layer->GetFaceList().GetNext(Pos);
//
//		// can be speeding up by AABB, do at next step;
//
//		Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
//		Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
//		Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));
//
//		//original normal detection
//		if (_intersetTriangle(orig, dir, v1, v2, v3, insertP)) {
//			support_by_1st_below_initialLayer = true;
//			break;
//		}
//	}
//	return support_by_1st_below_initialLayer;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool supportGeneration::_find_targetNode(
	Eigen::Vector3d& oringinNode,
	Eigen::Vector3d& step_Direction,
	QMeshNode* Node,
	const std::vector<QMeshPatch*>& largeLayer_vector_q,
	bool decline_stepDir,
	Eigen::Vector3d& last_descend_dir
)
{
	bool stopFlag = false;

	/******************* hit the bottom platform ******************/
	if (oringinNode[2] < 1.0) { //if the node hight(Z) less than 1mm, we consider it hit the bottom
		/*test*/
		//std::cout << "----------------------" << std::endl;
		//std::cout << "hit bottom." << std::endl;
		//std::cout << "node " << Node->GetIndexNo() << " on layer: " << Node->GetMeshPatchPtr()->largeLayer_Index << std::endl;
		//std::cout << "detected layer:" << largeLayer_vector_q[0]->largeLayer_Index << std::endl;
		//END
		Eigen::Vector3d target_bottom_node = { oringinNode[0],oringinNode[1],0.0 };
		// add one more node to link the platform (based on at least 1 support layer)
		Node->polyline_node.push_back(target_bottom_node);
		return true; 
	}

	/***************** incline the step_direction *****************/
	if (decline_stepDir) {
		//incline a self-supporting angle to the z direction
		/* method 1: only descend */
		Eigen::Vector3d descend_dir = { 0.0,0.0,-1.0 };
		/* method 2: go to {0 0 0} */
		//Eigen::Vector3d descend_dir = -oringinNode.normalized();

		Eigen::Vector3d rotateAxis = step_Direction.cross(descend_dir);

		//angle between _dir and _descend_dir [0,PI]
		double radian_angle = atan2(step_Direction.cross(descend_dir).norm(), step_Direction.transpose() * descend_dir);
		if (ROTATE_TO_DEGREE(radian_angle) < tau) {
			step_Direction = descend_dir; // in the range of support angle, directly downward

			//Eigen::AngleAxisd V(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis，rotate tau(deg)
			//step_Direction = V * step_Direction;
		}
		else {
			// only rotate a support angle
			Eigen::AngleAxisd V1(DEGREE_TO_ROTATE(tau), rotateAxis);//rotateAxis，rotate tau(deg)
			step_Direction = V1 * step_Direction;
		}
	}

	if (largeLayer_vector_q.size() == 2) {
		
		if (_intersect_Line_Face(oringinNode, step_Direction, Node, largeLayer_vector_q[0], last_descend_dir)) {
			// hit the initial layer -> stop tracing
			stopFlag = true;
		}
		
		else if (_intersect_Line_Face(oringinNode, step_Direction, Node, largeLayer_vector_q[1], last_descend_dir)) {
			// run once to get the intersection point on support layer
			if(largeLayer_vector_q[1]->largeLayer_Index == 0) stopFlag = true;
		}
		else {
			/*test*/ // there are no (1.initial & 2.support) faces wide enough to intersect
			//std::cout << "----------------------" << std::endl;
			//std::cout << "ERROR: topNode intersect with No Face(initial and support)."<< std::endl;
			//std::cout << "node "<< Node->GetIndexNo() << " on layer: " << Node->GetMeshPatchPtr()->largeLayer_Index << std::endl;
			//std::cout << "detected layer:" << largeLayer_vector_q[0]->largeLayer_Index << std::endl;
			//END
			stopFlag = true;
			Node->isHighlight = true;		
		}
	}
	else if (largeLayer_vector_q.size() == 1) {

		if (_intersect_Line_Face(oringinNode, step_Direction, Node, largeLayer_vector_q[0], last_descend_dir)){
			if (largeLayer_vector_q[0]->largeLayer_Index == 0) stopFlag = true;
		}
		else {
			/*test*/ // there are no support faces wide enough to intersect
			//std::cout << "----------------------" << std::endl;
			//std::cout << "ERROR: topNode intersect with No Face(one support)." << std::endl;
			//std::cout << "node " << Node->GetIndexNo() << " on layer: " << Node->GetMeshPatchPtr()->largeLayer_Index << std::endl;
			//std::cout << "detected layer:" << largeLayer_vector_q[0]->largeLayer_Index << std::endl;
			//END
			stopFlag = true;
			Node->isHighlight = true;
		}
	}
	else {
		std::cout << "ERROR: wrong large layer list" << std::endl;
		stopFlag = true;
	}

	return stopFlag;
}

// useless
//bool supportGeneration::_find_targetNode(QMeshPatch* top_layer, const std::vector<QMeshPatch*>& bottom_large_2layer) {
//
//	/*******************************************/
//	//(host tree node) O --> A(nearest tree node)
//	//                 |\,  /|
//	//                 | \./ |
//	/*******************************************/
//
//	bool stopflag = false;
//
//	for (GLKPOSITION Pos = top_layer->GetFaceList().GetHeadPosition(); Pos;) {
//		QMeshFace* Face = (QMeshFace*)top_layer->GetFaceList().GetNext(Pos);
//
//		if (Face->support_treeNode_cell.size() != 0) {
//
//			//only one treeNode on one face -> find at 1-ring neighbor
//			if (Face->support_treeNode_cell.size() == 1) {
//
//				//1-ring face 
//				std::vector<QMeshFace*> faceSet_1ring;
//				//collect 1-ring face
//				for (int i = 0; i < 3; i++) {
//
//					for (GLKPOSITION Pos_1ringFace = Face->GetNodeRecordPtr(i)->GetFaceList().GetHeadPosition(); Pos_1ringFace;){
//						QMeshFace* Face_1ring = (QMeshFace*)(Face->GetNodeRecordPtr(i)->GetFaceList().GetNext(Pos_1ringFace));
//
//						if (Face_1ring == Face) continue; // not include self
//
//						//1-ring face may repeat, avoid repeating
//						bool is_exist = false;
//						for (int j = 0; j < faceSet_1ring.size(); j++) {
//							if (faceSet_1ring[j] == Face_1ring) {
//								is_exist = true;
//								break;
//							}
//						}
//						if (is_exist == false) {faceSet_1ring.push_back(Face_1ring);}
//
//					}
//
//				}
//
//				/*get the nearest 1-ring face[k]*/
//				//nearest treeNode message:
//				QMeshFace* nearest_treeNode_onWhichFace = NULL;
//				int nearest_treeNode_onWhichPos_Face_treeNodeCell = -1;
//				double min_dist_O_A = 99999.9;
//				for (int k = 0; k < faceSet_1ring.size(); k++) {
//
//					// 1-ring neighbor with treeNode (may own more than one treeNode[s])
//					if (faceSet_1ring[k]->support_treeNode_cell.size() != 0) {
//						for (int s = 0; s < faceSet_1ring[k]->support_treeNode_cell.size(); s++) {
//							double dist_O_A = (faceSet_1ring[k]->support_treeNode_cell[s].treeNode_coord3D
//								- Face->support_treeNode_cell[0].treeNode_coord3D).norm();
//
//							if (dist_O_A < min_dist_O_A) {
//								min_dist_O_A = dist_O_A;
//								nearest_treeNode_onWhichFace = faceSet_1ring[k];
//								nearest_treeNode_onWhichPos_Face_treeNodeCell = s;
//							}
//						}
//
//					}
//				}
//				
//				if (nearest_treeNode_onWhichFace != NULL) {
//
//					std::cout << "--------\nOh!!! find nieghbor tree Node" << "top layer is: layer " << top_layer->largeLayer_Index << std::endl;
//
//					/*compute the intersection of two rays*/
//					Eigen::Vector3d intersection;
//					Eigen::Vector3d O = Face->support_treeNode_cell[0].treeNode_coord3D;
//					Eigen::Vector3d A = nearest_treeNode_onWhichFace->support_treeNode_cell[nearest_treeNode_onWhichPos_Face_treeNodeCell].treeNode_coord3D;
//
//					Eigen::Vector3d OA = A - O;
//					Eigen::Vector3d Oo; Face->GetNormal(Oo[0], Oo[1], Oo[2]);				
//					// cal host node direction
//					Eigen::Vector3d Axis_O = Oo.cross(OA);
//					Eigen::AngleAxisd V1(DEGREE_TO_ROTATE(tau), Axis_O);//rotateAxis，rotate tau(deg)
//					Oo = V1 * Oo;
//					Face->support_treeNode_cell[0].treeNode_descend_Nor = Oo;
//
//					// cal slave node direction
//					Eigen::Vector3d Aa; nearest_treeNode_onWhichFace->GetNormal(Aa[0], Aa[1], Aa[2]);
//					// proj face normal of slave node to the common plane
//					Aa = Aa - Aa.dot(Axis_O) / Axis_O.squaredNorm() * Axis_O;
//					Eigen::AngleAxisd V2(DEGREE_TO_ROTATE(-tau), Axis_O);//rotateAxis，rotate tau(deg)
//					Aa = V2 * Aa;
//					Face->support_treeNode_cell[0].treeNode_descend_Nor = Aa;
//
//					//std::cout << "O " << O.transpose() << std::endl;
//					//std::cout << "Oo " << Oo.transpose() << std::endl;
//					//std::cout << "A " << A.transpose() << std::endl;
//					//std::cout << "Aa " << Aa.transpose() << std::endl;
//					bool is_intersect = _LineLineIntersection(intersection, O, Oo, A, Aa);
//					/*END: compute the intersection of two rays*/
//					std::cout <<"is_intersect ? " << is_intersect << std::endl;
//					if(is_intersect) std::cout << "Intersection point:" << intersection.transpose() << std::endl;
//
//					bool intersect_IorS = false;
//					for (int layerIndex = 0; layerIndex < bottom_large_2layer.size(); layerIndex++) {
//						if (_pnt_2_layer(intersection, bottom_large_2layer[layerIndex])) {
//
//							std::printf("point is above 1st layer below (./)\n");
//
//							Eigen::Vector3d hit_bottom_Dirction = (Oo + Aa).normalized();// average normal of Oo and Aa
//							//record message for host Node
//							intersect_IorS = _intersect_Line_Face(O, intersection, hit_bottom_Dirction,
//								Face->support_treeNode_cell[0].souce_Node, bottom_large_2layer[layerIndex]);
//							//record message for slave Node
//							nearest_treeNode_onWhichFace->support_treeNode_cell[nearest_treeNode_onWhichPos_Face_treeNodeCell].souce_Node->polyline_node.push_back(intersection);
//						}
//						else {
//							std::printf("point is under 1st layer below (/.)\n");
//						}
//					}
//
//					if (intersect_IorS == false) {
//						std::printf("below layer is not wide enough.\n");
//						return true;
//					}
//				}
//				//downforward
//				else {
//
//				}
//
//			}
//			//more than one treeNode on one face -> find on self-face
//			else {
//
//			}
//
//
//		}
//	}
//
//	return stopflag;
//}

bool supportGeneration::_find_targetNode(
	Eigen::Vector3d oringinNode,
	Eigen::Vector3d step_Direction,
	QMeshNode* Node,
	const std::vector<QMeshPatch*>& largeLayer_vector_m,
	Eigen::Vector3d& nearest_hostNode_coord3D,
	bool is_goto_center) {

	bool stopFlag = false;

	/******************* hit the bottom platform ******************/
	if (oringinNode[2] < 0.5) { //if the node hight(Z) less than 0.5mm, we consider it hit the bottom
		/*test*/
		//std::cout << "----------------------" << std::endl;
		//std::cout << "hit bottom." << std::endl;
		//std::cout << "node " << Node->GetIndexNo() << " on layer: " << Node->GetMeshPatchPtr()->largeLayer_Index << std::endl;
		//std::cout << "detected layer:" << largeLayer_vector_q[0]->largeLayer_Index << std::endl;
		//END
		Eigen::Vector3d target_bottom_node = { oringinNode[0],oringinNode[1],0.0 };
		// add one more node to link the platform (based on at least 1 support layer)
		Node->polyline_node.push_back(target_bottom_node);
		return true;
	}

	Eigen::Vector3d intersetPnt = { -2.0,-2.0,-2.0 };

	if (largeLayer_vector_m.size() == 2) {

		if (_intersect_Line_Face(intersetPnt, oringinNode, step_Direction, Node, largeLayer_vector_m[0])) {
			// hit the initial layer -> stop tracing
			stopFlag = true;
		}

		else if (_intersect_Line_Face(intersetPnt, oringinNode, step_Direction, Node, largeLayer_vector_m[1])) {
			// run once to get the intersection point on support layer
			if (largeLayer_vector_m[1]->largeLayer_Index == 0) stopFlag = true;

			/*if (!is_goto_center) {
				if ((nearest_hostNode_coord3D - intersetPnt).norm() < 2.0) stopFlag = true;
			}*/
		}
		else {
			/*test*/ // there are no (1.initial & 2.support) faces wide enough to intersect
			//std::cout << "----------------------" << std::endl;
			//std::cout << "ERROR: topNode intersect with No Face(initial and support)."<< std::endl;
			//std::cout << "node "<< Node->GetIndexNo() << " on layer: " << Node->GetMeshPatchPtr()->largeLayer_Index << std::endl;
			//std::cout << "detected layer:" << largeLayer_vector_q[0]->largeLayer_Index << std::endl;
			//END
			stopFlag = true;
			Node->isHighlight = true;
		}
	}
	else if (largeLayer_vector_m.size() == 1) {

		if (_intersect_Line_Face(intersetPnt, oringinNode, step_Direction, Node, largeLayer_vector_m[0])) {
			if (largeLayer_vector_m[0]->largeLayer_Index == 0) stopFlag = true;

			/*if (!is_goto_center) {
				if ((nearest_hostNode_coord3D - intersetPnt).norm() < 2.0) stopFlag = true;
			}*/
		}
		else {
			/*test*/ // there are no support faces wide enough to intersect
			//std::cout << "----------------------" << std::endl;
			//std::cout << "ERROR: topNode intersect with No Face(one support)." << std::endl;
			//std::cout << "node " << Node->GetIndexNo() << " on layer: " << Node->GetMeshPatchPtr()->largeLayer_Index << std::endl;
			//std::cout << "detected layer:" << largeLayer_vector_q[0]->largeLayer_Index << std::endl;
			//END
			stopFlag = true;
			Node->isHighlight = true;
		}
	}
	else {
		std::cout << "ERROR: wrong large layer list" << std::endl;
		stopFlag = true;
	}

	return stopFlag;

}

// for polyline
bool supportGeneration::_intersect_Line_Face
(
	Eigen::Vector3d& oringinNode, 
	Eigen::Vector3d& step_Direction, 
	QMeshNode* Node,
	QMeshPatch* detect_layer,
	Eigen::Vector3d& last_step_Direction
)
{
	bool isIntersect = false;
	Eigen::Vector3d insertP, v1, v2, v3;
	for (GLKPOSITION Pos = detect_layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)detect_layer->GetFaceList().GetNext(Pos);

		/*speeding up by distance limit*/
		// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
		Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
		if ((center - oringinNode).norm() > 4) continue;

		Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
		Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
		Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

		if (_intersetTriangle(oringinNode, step_Direction, v1, v2, v3, insertP)) {

			isIntersect = true;
			if (detect_layer->is_SupportLayer) {
				//record the info for intersection face
//				SptTreeNode stNode;
//				stNode.souce_Node = Node;
//				stNode.treeNode_descend_Nor = step_Direction; //-->./Node
//				stNode.treeNode_coord3D = insertP;
//				if (Node->isHostNode) { stNode.isHostNode = true; }
//				else { stNode.isHostNode = false; }
//				stNode.isProcessed = true;
//				Face->support_treeNode_cell.push_back(stNode);
				/***********decide whether pop back the middle point of the skeleton line to speed up***********/
				/*std::cout << "step_Direction: " << step_Direction.transpose() 
					<< "last_step_Direction: " << last_step_Direction.transpose() << std::endl;*/
				if (step_Direction == last_step_Direction) Node->polyline_node.pop_back();
				/*********** record the step_direction of last step ***********/
				last_step_Direction = step_Direction;
				Node->polyline_node.push_back(insertP);
				oringinNode = insertP;
				for (int i = 0; i < 3; i++) { step_Direction[i] = Face->m_desiredNormal[i]; 
						
				}
			}
			break;
		}
	}
	return isIntersect;
}

// for tree-like support generation

// (host)\    /(slave)
//       |\  /
//       | \/ line 2 line intersection (oringinNode)
//        \/\
//        /\.\ merged point
// useless

bool supportGeneration::_intersect_Line_Face
(
	Eigen::Vector3d& intersetPnt,
	Eigen::Vector3d& oringinNode,
	Eigen::Vector3d& step_Direction,
	QMeshNode* Node,
	QMeshPatch* detect_layer	
)
{

	bool isIntersect = false;
	Eigen::Vector3d insertP, v1, v2, v3;
	for (GLKPOSITION Pos = detect_layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)detect_layer->GetFaceList().GetNext(Pos);

		/*speeding up by distance limit*/
		// when the distance bewteen face and originNode is too far than 5mm(temporary), skip it;
		Eigen::Vector3d center; Face->GetCenterPos(center[0], center[1], center[2]);
		if ((center - oringinNode).norm() > 5) continue;

		Face->GetNodeRecordPtr(0)->GetCoord3D(v1(0), v1(1), v1(2));
		Face->GetNodeRecordPtr(1)->GetCoord3D(v2(0), v2(1), v2(2));
		Face->GetNodeRecordPtr(2)->GetCoord3D(v3(0), v3(1), v3(2));

		if (_intersetTriangle(oringinNode, step_Direction, v1, v2, v3, insertP)) {

			isIntersect = true;
			Node->polyline_node.push_back(insertP);
			// drop on initial layers means tracing end, and does not record stNode
			if (detect_layer->is_SupportLayer) {
				/***********decide whether pop back the middle point of the skeleton line to speed up***********/
				/*std::cout << "step_Direction: " << step_Direction.transpose()
					<< "last_step_Direction: " << last_step_Direction.transpose() << std::endl;*/
				//if (step_Direction == last_step_Direction) Node->polyline_node.pop_back();
				/*********** record the step_direction of last step ***********/
				//last_step_Direction = step_Direction;
				
				SptTreeNode stNode;
				stNode.souce_Node = Node;
				stNode.treeNode_descend_Nor = step_Direction; //-->node
				stNode.treeNode_coord3D = insertP;
				if(Node->isHostNode)stNode.isHostNode = true;
				else stNode.isHostNode = false;
				stNode.isProcessed = true;
				if (Face->support_treeNode_cell.size() == 0){// only record one target point to face
					Face->support_treeNode_cell.push_back(stNode);
				}

				intersetPnt = insertP;
			}
			break;
		}
	}
	return isIntersect;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Line1: p1->(v1); Line2: p2->(v2); => return whether intersection & intersection point
// reference: https://blog.csdn.net/xdedzl/article/details/86009147?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-6.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-6.control
bool supportGeneration::_LineLineIntersection(Eigen::Vector3d& intersection,
	Eigen::Vector3d& p1, Eigen::Vector3d& v1,
	Eigen::Vector3d& p2, Eigen::Vector3d& v2) {

	intersection = Eigen::Vector3d::Zero();
	if (v1.dot(v2) == 1.0)
	{
		// line1 is parallel with line2
		return false;
	}

	Eigen::Vector3d startPointSeg = p2 - p1;
	Eigen::Vector3d vecS1 = v1.cross(v2);            // axis 1
	Eigen::Vector3d vecS2 = startPointSeg.cross(v2); // axis 2
	double num = startPointSeg.dot(vecS1);

	// decide line1 and line2 are in the same plane
	if (num >= 1E-05f || num <= -1E-05f)
	{
		return false;
	}

	// calculate the ratio of similar triangle
	double num2 = vecS2.dot(vecS1) / vecS1.dot(vecS1);

	intersection = p1 + v1 * num2;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ./ -> true    | /. -> false
//        \/     |  \    /
// ____(above)___|___\__/____
//               |    \/(below)
// decide whether the point is above OR below layer 
bool supportGeneration::_pnt_2_layer(Eigen::Vector3d& queryPnt, QMeshPatch* detect_layer) {

	bool is_above = true;

	for (GLKPOSITION posFace = detect_layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)detect_layer->GetFaceList().GetNext(posFace);

		Eigen::Vector3d plane_normal;
		plane_normal << face->m_desiredNormal[0], face->m_desiredNormal[1], face->m_desiredNormal[2];
		double d = plane_normal.dot(queryPnt) + face->m_desired_D;

		if (d >= 0.0) { // if detecting node below any face, break;
			is_above = false;
			break;
		}
	}

	return is_above;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// input:  TET model + vector field 
// output: convex hull (wide enough for including model)
void supportGeneration::initial_Guess_SupportEnvelope() {

	markSupportFace(); // the threshold of self-support angle (tau) tuned in initial Function

	QMeshPatch* tet_Model = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();
	//calculate the direction of support Ray; by laplacian smoothness
	_cal_Ray_direction(tet_Model);

	// reconstruct the Node Array for omp usage
	std::vector<QMeshNode*> tet_surfaceNode_set;
	for (GLKPOSITION Pos = tet_Model->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)tet_Model->GetNodeList().GetNext(Pos);
		// the inner node is not considered
		if (Node->inner) continue;
		if (Node->need_Support == false) continue;
		tet_surfaceNode_set.push_back(Node);
	}

#pragma omp parallel
	{
#pragma omp for  

		for (int i = 0; i < tet_surfaceNode_set.size(); i++) {
			QMeshNode* Node = tet_surfaceNode_set[i];

			Eigen::Vector3d oringinNode; Node->GetCoord3D(oringinNode[0], oringinNode[1], oringinNode[2]);
			Eigen::Vector3d step_Direction = Node->supportRay_Dir;	

			// 2. iteratively find the drop points
			for (;;) {
				bool decline_stepDir = true;
				if (Node->polyline_node.size() == 0)decline_stepDir = false;
				bool stopFlag = _find_targetNode(oringinNode, step_Direction, Node, decline_stepDir);
				if (stopFlag) break;
			}
		}
	}
}

bool supportGeneration::_find_targetNode(
	Eigen::Vector3d& oringinNode,
	Eigen::Vector3d& step_Direction,
	QMeshNode* Node,
	bool decline_stepDir
) {

	//parameter:
	double step_length = 1.0; //mm
	double descend_angle = 2.0; //deg

	/******************* hit the bottom platform ******************/
	if (oringinNode[2] < 1.0) { //if the node hight(Z) less than 1mm, we consider it hit the bottom
		 
		Eigen::Vector3d target_bottom_node = { oringinNode[0],oringinNode[1],0.0 };
		// add one more node to link the platform (based on at least 1 support layer)
		Node->polyline_node.push_back(target_bottom_node);
		return true; //stopFlag
	}

	/***************** incline the step_direction *****************/
	if (decline_stepDir) {
		//incline a self-supporting angle to the z direction
		Eigen::Vector3d descend_dir = { 0.0,0.0,-1.0 };
		Eigen::Vector3d rotateAxis = step_Direction.cross(descend_dir);

		//angle between _dir and _descend_dir [0,PI]
		double radian_angle = atan2(step_Direction.cross(descend_dir).norm(), step_Direction.transpose() * descend_dir);
		if (ROTATE_TO_DEGREE(radian_angle) < descend_angle) step_Direction = descend_dir; // in the range of support angle, directly downward
		else {
			// only rotate a support angle
			Eigen::AngleAxisd V1(DEGREE_TO_ROTATE(descend_angle), rotateAxis);//rotateAxis，rotate tau(deg)
			step_Direction = V1 * step_Direction;
		}
	}

	Eigen::Vector3d target_node = oringinNode + step_length * step_Direction;

	Node->polyline_node.push_back(target_node);
	oringinNode = target_node;

	return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Envelope Hull compute
void supportGeneration::computer_initialGuess_EnvelopeHull() {
	//build Convex-Hull (inclouding supporing point and surface of tetrahedral mesh)
	int pntNum = 0;
	QMeshPatch* supportNodePatch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetHead();
	pntNum += supportNodePatch->GetNodeNumber();

	QMeshPatch* tetMesh = (QMeshPatch*)m_tetModel->GetMeshList().GetHead();
	pntNum += tetMesh->GetNodeNumber();

	facetT* facet;		vertexT* vertex, ** vertexp;
	int i, index, num, stIndex;			float pos[3];
	double vec[3][3], dir[3], v1[3], v2[3], pp[3];
	QHULLSET* newConvexFront = NULL; // new convexhull used for checking

	double* pntArray = (double*)malloc(sizeof(double) * 3 * pntNum); //all the point use to compute convex hull

	int nodeIndex = 0;
	for (GLKPOSITION posMesh = supportNodePatch->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)supportNodePatch->GetNodeList().GetNext(posMesh);
		node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++)
			pntArray[nodeIndex * 3 + i] = pp[i];
		nodeIndex++;
	}
	for (GLKPOSITION posMesh = tetMesh->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)tetMesh->GetNodeList().GetNext(posMesh);
		node->GetCoord3D(pp);
		for (int i = 0; i < 3; i++)
			pntArray[nodeIndex * 3 + i] = pp[i];
		nodeIndex++;
	}

	//-------------------------------------------------------------------------------------
	//	Step 2: computaing the convex-hull
	qh_init_A(stdin, stdout, stderr, 0, NULL);
	qh_initflags("Qt Qx");
	qh_init_B(pntArray, pntNum, 3, false);
	qh_qhull();
	qh_check_output();
	qh_triangulate();
	if (qh VERIFYoutput && !qh STOPpoint && !qh STOPcone) qh_check_points();

	//-------------------------------------------------------------------------------------
	//	Step 3: output the results of convex-hull computation
	int nodeNum = 0, faceNum = 0;
	faceNum = qh_qh.num_facets;		nodeNum = qh_qh.num_vertices;
	//printf("Convex-Hull: %d faces with %d vertices\n",faceNum,nodeNum);
	if (faceNum > 0 && nodeNum > 0) {
		newConvexFront = _mallocMemoryConvexHull(faceNum, nodeNum);
		//---------------------------------------------------------------------------------
		index = 0;
		FORALLvertices{
			vertex->id = index;	// before this assignment, "vertex->id" contains the id of input vertices
		newConvexFront->vertPos[index * 3] = vertex->point[0];
		newConvexFront->vertPos[index * 3 + 1] = vertex->point[1];
		newConvexFront->vertPos[index * 3 + 2] = vertex->point[2];
		index++;
		}
			//---------------------------------------------------------------------------------
		index = 0;
		FORALLfacets{
			newConvexFront->normalVec[index * 3] = facet->normal[0];
		newConvexFront->normalVec[index * 3 + 1] = facet->normal[1];
		newConvexFront->normalVec[index * 3 + 2] = facet->normal[2];
		newConvexFront->offset[index] = facet->offset;
		//	It has been verified all normal[] vectors generated by qhull library are pointing outwards and are unit-vectors 
		//		(verified by the function -- QuadTrglMesh* convexHullGeneration(QuadTrglMesh* inputMesh)  ).

		int i = 0;
		FOREACHvertex_(facet->vertices) {
			newConvexFront->faceTable[index * 3 + i] = vertex->id + 1; //index start from 1;
																	   //newConvexFront->faceTable[index * 3 + i] = vertex->id; //index start from 0;

			SET(vec[i],vertex->point);
			i++;
			if (i >= 3) break; // Note that it could be a facet with more than 3 vertices if not applying "qh_triangulate();"
		}

		//-----------------------------------------------------------------------------
		//	Check if the vertices on this face is given in the anti-clockwise order
		SUB(v1,vec[1],vec[0]);
		SUB(v2,vec[2],vec[0]);
		CROSS(dir,v1,v2);
		if (DOT(dir,facet->normal) < 0) {
			unsigned int temp = newConvexFront->faceTable[index * 3];
			newConvexFront->faceTable[index * 3] = newConvexFront->faceTable[index * 3 + 2];
			newConvexFront->faceTable[index * 3 + 2] = temp;
		}

		index++;
		}
	}

	//-------------------------------------------------------------------------------------
	//	Step 4: free the memory
	int curlong, totlong;
	qh_freeqhull(false);
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong) fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	//-------------------------------------------------------------------------------------
	free(pntArray);

	_drawConvexHull(newConvexFront, m_supportRaySet); /*********************** also output convex Hull ***********************/
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);

		// 0 - support ray; 1 - envelope HULL
		if (supportRay_Patch->GetIndexNo() == 1) supportRay_Patch->drawThisPatch = false;
		if (supportRay_Patch->GetIndexNo() == 0) supportRay_Patch->drawThisPatch = true;

		//supportRay_Patch->drawThisPatch = true;
	}
}

QHULLSET* supportGeneration::_mallocMemoryConvexHull(int faceNum, int vertNum) {

	QHULLSET* pConvexHull;

	pConvexHull = (QHULLSET*)malloc(sizeof(QHULLSET));
	pConvexHull->faceNum = faceNum;
	pConvexHull->normalVec = (double*)malloc(sizeof(double) * 3 * faceNum);
	pConvexHull->offset = (double*)malloc(sizeof(double) * faceNum);

	pConvexHull->faceTable = (unsigned int*)malloc(sizeof(unsigned int) * 3 * faceNum);

	pConvexHull->vertNum = vertNum;
	pConvexHull->vertPos = (double*)malloc(sizeof(double) * 3 * vertNum);

	return pConvexHull;
}

void supportGeneration::_drawConvexHull(QHULLSET* ConvexHULL, PolygenMesh* m_supportRaySet) {

	QMeshPatch* convexHullVisual = new QMeshPatch;
	convexHullVisual->drawThisPatch = true;
	convexHullVisual->SetIndexNo(m_supportRaySet->GetMeshList().GetCount()); //index begin from 0
	m_supportRaySet->GetMeshList().AddTail(convexHullVisual);

	float* nodeTable;
	nodeTable = (float*)malloc(sizeof(float) * ConvexHULL->vertNum * 3);
	for (int i = 0; i < ConvexHULL->vertNum * 3; i++)
		nodeTable[i] = (float)ConvexHULL->vertPos[i];
	unsigned int* faceTable;
	faceTable = (unsigned int*)malloc(sizeof(unsigned int) * ConvexHULL->faceNum * 3);
	for (int i = 0; i < ConvexHULL->faceNum * 3; i++)
		faceTable[i] = ConvexHULL->faceTable[i] - 1;

	convexHullVisual->constructionFromVerFaceTable
	(ConvexHULL->vertNum, nodeTable, ConvexHULL->faceNum, faceTable);
	_freeMemoryConvexHull(ConvexHULL);
	free(nodeTable);	free(faceTable);

	std::string supportConvex_dir = "../DataSet/SUPPORT_SURFACE/" + m_model_name + "_ConvexHull";
	output_OneSurfaceMesh(convexHullVisual, supportConvex_dir);
}

void supportGeneration::_freeMemoryConvexHull(QHULLSET*& pConvexHull){
	free((pConvexHull->normalVec));
	free((pConvexHull->offset));
	free((pConvexHull->faceTable));
	free((pConvexHull->vertPos));
	free(pConvexHull);

	pConvexHull = NULL;
}

// check grid node is in(true) the tet model
bool supportGeneration::_checkSingleNode_inTETmodel(QMeshPatch* tet_Model, Eigen::Vector3d pp, double delta_expand) {

	bool node_in_TETmodel = false;

	Eigen::Vector3d boxRange_min = { pp[0] - delta_expand,pp[1] - delta_expand,pp[2] - delta_expand };
	Eigen::Vector3d boxRange_max = { pp[0] + delta_expand,pp[1] + delta_expand,pp[2] + delta_expand };

	for (GLKPOSITION Pos = (tet_Model->GetTetraList()).GetHeadPosition(); Pos != NULL;) {
		QMeshTetra* tet_Ele = (QMeshTetra*)((tet_Model->GetTetraList()).GetNext(Pos));

		Eigen::Vector3d tetNode_center;
		tet_Ele->CalCenterPos(tetNode_center[0], tetNode_center[1], tetNode_center[2]);

		if ((tetNode_center[0] > boxRange_min[0])
			&& (tetNode_center[0] < boxRange_max[0])
			&& (tetNode_center[1] > boxRange_min[1])
			&& (tetNode_center[1] < boxRange_max[1])
			&& (tetNode_center[2] > boxRange_min[2])
			&& (tetNode_center[2] < boxRange_max[2])) 
		{
			node_in_TETmodel = true;
			//for (int i = 0; i < 4; i++) {tet_Ele->GetNodeRecordPtr(i+1)->isHighlight = true;}
			break;
		}

	}
	return  node_in_TETmodel;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//change Zup to Yup
void supportGeneration::output_OneSurfaceMesh(QMeshPatch* isoSurface, std::string path) {

	double pp[3];
	path += ".obj";
	std::ofstream nodeSelection(path);

	int index = 0;
	for (GLKPOSITION posNode = isoSurface->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)isoSurface->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp);
		nodeSelection << "v " << pp[0] << " " << pp[2] << " " << -pp[1] << std::endl;
		index++; node->SetIndexNo(index);
	}
	for (GLKPOSITION posFace = isoSurface->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(posFace);
		nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
	}
	nodeSelection.close();
}

void supportGeneration::output_initial_Polyline() {
	QMeshPatch* supportPoly_patch = NULL;
	for (GLKPOSITION Pos = m_supportRaySet->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* supportRay_Patch = (QMeshPatch*)m_supportRaySet->GetMeshList().GetNext(Pos);
		// 0 - support ray; 1 - un-remeshed convexHull
		if (supportRay_Patch->GetIndexNo() == 0) {
			supportPoly_patch = supportRay_Patch;
		}
	}
	std::string tetModel_support_polyline_dir 
		= "../DataSet/SUPPORT_SURFACE/" + this->m_model_name + "_support_polyline";
	this->_output_support_polyline(supportPoly_patch, tetModel_support_polyline_dir);
	std::cout << "Output support polyline for initial guess of support generation." << std::endl;
	std::cout << tetModel_support_polyline_dir << std::endl;
}

void supportGeneration::_output_support_polyline (QMeshPatch* supportPoly_patch, std::string path) {

	double pp[3];
	path += ".txt";
	std::ofstream str(path);

	for (GLKPOSITION posNode = supportPoly_patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)supportPoly_patch->GetNodeList().GetNext(posNode);

		node->GetCoord3D(pp[0], pp[1], pp[2]);
		str << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
	}
	str.close();
}