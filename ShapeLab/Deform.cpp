#include "Deform.h"
#include "PrincipleStressField.h"
#include "../GLKLib/GLKGeometry.h"

void Deform::initial(PolygenMesh* polygenMesh_TetModel) {

	m_tetPatch = (QMeshPatch*)polygenMesh_TetModel->GetMeshList().GetHead();
	m_tetPatch_Name = polygenMesh_TetModel->getModelName();

	this->_index_initial(m_tetPatch, true);
	this->_build_tetraSet_4SpeedUp(m_tetPatch);
	this->_moveModelup2ZeroHeight(m_tetPatch);  //finalize the node positions into Coord3D_last
	this->_record_neighbor_Tet();
	this->_compTetMeshVolumeMatrix(m_tetPatch);
	this->_detectBottomTet(m_bottom_height);
	this->_getNodeFaceTet_num();
	if (m_isHole_considered ) this->_getHole_index_face_tet();
	if (m_surfKeep_considered) this->_get_KeptSurf_tet();

	if (m_tetPatch_Name == "connector") {
		this->_setParameters(7, 1, 600, 3, 1, 3, 1);
		tensile_ratio = 0.5; compress_ratio = 0.0;
	}
	else if (m_tetPatch_Name == "topopt") {
		this->_setParameters(7, 1, 600, 3, 1, 3, 1);
		tensile_ratio = 0.3; compress_ratio = 0.1;
	}
	else if (m_tetPatch_Name == "clip") {
		this->_setParameters(7, 1, 600, 3, 1, 3, 1);
		tensile_ratio = 0.5; compress_ratio = 0.0;
	}
	if (m_tetPatch_Name == "connector2") {
		this->_setParameters(7, 1, 600, 3, 1, 3, 1);
		tensile_ratio = 0.5; compress_ratio = 0.0;
	}
	if (m_tetPatch_Name == "bracket") {
		this->_setParameters(7, 1, 600, 3, 1, 3, 1);
		tensile_ratio = 0.5; compress_ratio = 0.0;
	}

	std::printf("\n/***************************/\n\nDeformation case: \n");
	std::cout << "--> m_isHole_considered \t" << m_isHole_considered << std::endl;
	std::cout << "--> m_surfKeep_considered \t" << m_surfKeep_considered << std::endl;
	std::printf("\n/***************************/\n");


	std::printf("Finish initialization of Deform class.\n");
}

bool Deform::preProcess_4StrengthReinforcement() {

	PrincipleStressField* stressFieldComp = new PrincipleStressField(m_tetPatch);
	bool successful_Read = stressFieldComp->InputFEMResult(m_tetPatch_Name);
	if (successful_Read) {
		m_tetPatch->drawStressField = true;
		stressFieldComp->ComputeElementPrincipleStress();
		stressFieldComp->DetermineCriticalTensileandCompressRegion(tensile_ratio, compress_ratio);
		stressFieldComp->DetectSmallCriticalRegionandClear();
	}
	delete stressFieldComp;

	return successful_Read;
}

void Deform::runASAP_SR() {

	if(m_isHole_considered) this->_update_hole_direction();
	this->_initial_variables_and_space(false);

	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	for (int outerLoop = 0; outerLoop < m_outer_cycle; outerLoop++) {

		this->_calFabricationEnergy_SR();

		for (int innerLoop = 0; innerLoop < m_inner_cycle; innerLoop++) {
		// local rotation and scaling operation (frame_Local_new)

#pragma omp parallel
			{
#pragma omp for 
				for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
					QMeshTetra* Tetra = tetraPatch_elementSet[i];

					int fdx = Tetra->GetIndexNo();

					// the estimation of SVD and hole handle control only needs once
					if (innerLoop == 0) {

						// R1 
						Eigen::Matrix3d R1 = Eigen::Matrix3d::Zero(3, 3);
						this->_estimate_localRotation(Tetra, fdx, R1);

						Tetra->R_estimate = R1;
						frame_Local_new[fdx] = R1 * frame_Local_initial[fdx];
						//std::printf("Finish local frame rotation 1, shape drives.\n");

						if (Tetra->is_holeRegion) {

							Eigen::Vector3d current_HoleDir = Tetra->R_estimate * Tetra->holeDir;
							Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_Hole(current_HoleDir);

							Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
							frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
							//std::printf("Finish local frame rotation 2.1, hole drives.\n");
						}
					}
					else {
						// R2.2 --> Fabrication requirement: stress
						if (Tetra->isTensileorCompressSelect) {

							Eigen::Vector3d principal_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;
							Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_SR(principal_Stress_Dir);

							Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
							frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
							//std::printf("Finish local frame rotation 2.2, stress drives.\n");
						}
					}
				}
			}
			
			std::printf("Finish new local frame calculation.\n");

			this->_quaternionSmooth_SR();

			for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];
				frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
			}
			std::printf("Finish global quaternion smooth calculation.\n");

			this->_get_energy_innerLoop_SR();
		}

		this->_build_matrix(true);
		this->_solve_eqation();
		this->_extract_info();

		std::printf("End calculation of outer loop %d.\n\n", outerLoop);
	}
	std::printf("End calculation.\n");
}

void Deform::runASAP_SR_SQ() {

	this->runASAP_SR();
	if (!m_surfKeep_considered) return;
	this->postProcess();
	this->_initial_variables_and_space(true);
	//---------------------------------------------------------------------------------------------------------------
	// Main function of ASAP iteration
	this->_update_outerLoop_weights(15, 3, 0.4);
	for (int outerLoop = 0; outerLoop < m_outer_cycle_surfKeep; outerLoop++) {

		this->_calFabricationEnergy_SQ();
	
		// local rotation and scaling operation (frame_Local_new)
#pragma omp parallel
		{
#pragma omp for 
			for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
				QMeshTetra* Tetra = tetraPatch_elementSet[i];

				int fdx = Tetra->GetIndexNo();

				// the estimation of SVD
				// R1 
				Eigen::Matrix3d R1 = Eigen::Matrix3d::Zero(3, 3);
				this->_estimate_localRotation(Tetra, fdx, R1);

				Tetra->R_estimate = R1;
				frame_Local_new[fdx] = R1 * frame_Local_initial[fdx];
				//std::printf("Finish local frame rotation 1, shape drives.\n");
					
				// R2.3 --> Fabrication requirement: surf Kept
				if (Tetra->is_surfKeptRegion) {

					QMeshFace* keptFace = Tetra->kept_Face;
					Eigen::Matrix3d rotationMatrix = this->_cal_rotationMatrix_SQ(keptFace);
					Tetra->R_estimate = rotationMatrix * Tetra->R_estimate;
					frame_Local_new[fdx] = rotationMatrix * frame_Local_new[fdx];
					//std::printf("Finish local frame rotation 2.3, suf Keep drives.\n");
				}
			}
		}

		std::printf("Finish new local frame calculation.\n");

		this->_quaternionSmooth_SQ();

		for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
			QMeshTetra* Tetra = tetraPatch_elementSet[i];
			frame_Local_new[i] = Tetra->R_estimate * frame_Local_initial[i];
		}
		std::printf("Finish global quaternion smooth calculation.\n");	

		this->_build_matrix(false);
		this->_solve_eqation();
		this->_extract_info();

		std::printf("End calculation of outer loop %d.\n\n", outerLoop);
	}
	std::printf("End calculation.\n");
}

void Deform::postProcess() {

	this->_moveModel2Center();
	this->_record_deformed_position();
	this->_record_hightField();
}

void Deform::_setParameters(
	int inner_cycle, 
	double inner_critical_weight, double inner_smooth_weight,
	int outer_cycle, 
	double outer_criticalTet_weight, double outer_neighborScale_weight, double outer_regularScale_weight){

	m_inner_cycle = inner_cycle;
	m_inner_critical_weight = inner_critical_weight;
	m_inner_smooth_weight = inner_smooth_weight;

	m_outer_cycle = outer_cycle;
	m_outer_criticalTet_weight = outer_criticalTet_weight;
	m_outer_neighborScale_weight = outer_neighborScale_weight;
	m_outer_regularScale_weight = outer_regularScale_weight;
}

void Deform::_index_initial(QMeshPatch* patch, bool is_TetMesh) {

	//----initial the edge, node and face index, start from 0
	int index = 0;
	for (GLKPOSITION Pos = patch->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* Edge = (QMeshEdge*)patch->GetEdgeList().GetNext(Pos);
		Edge->SetIndexNo(index);
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		Face->SetIndexNo(index);
		Face->CalPlaneEquation(); // pre-compute the normal of face
		index++;
	}
	index = 0;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

		Node->SetIndexNo(index);
		index++;
	}
	// only tet mesh needs reorder the index of Tetra
	if (is_TetMesh) {
		index = 0;
		for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
			QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);

			Tetra->SetIndexNo(index);
			index++;
		}
	}
	std::cout << "Finish initializing mesh index." << std::endl;
}

void Deform::_build_tetraSet_4SpeedUp(QMeshPatch* patch) {

	tetraPatch_elementSet.resize(patch->GetTetraNumber());
	int index = 0;
	for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);
		tetraPatch_elementSet[index] = Tetra; index++;
	}

	// protect operation
	if (patch->GetTetraNumber() != index) std::cout << "Error: please check the num of tet mesh(materialSpace)! " << std::endl;

	std::cout << "Finish building tet set for SpeedUp." << std::endl;
}

void Deform::_moveModelup2ZeroHeight(QMeshPatch* patch) {

	// move to the Zero height (follow the operation in SIGGRAPH2018)
	double xmin, ymin, zmin, xmax, ymax, zmax;
	patch->ComputeBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
	std::cout << "the min y coord3D is " << ymin << " move it to Zero Height" << std::endl;
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		yy -= ymin;

		node->SetCoord3D(xx, yy, zz);
		node->SetCoord3D_last(xx, yy, zz);
	}
}

void Deform::_record_neighbor_Tet() {

	for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];
		std::vector< QMeshTetra* > Tetra_neighbSet;
		_detect_each_neighbor_Tet(Tetra_neighbSet, Tetra, false);
		Tetra->neighborCell = Tetra_neighbSet;
	}
	std::cout << "Finish recording neighbor Tet." << std::endl;
}

//neighb_type == true; // use node to detect neighbor
//neighb_type == false;// use face to detect neighbor
void Deform::_detect_each_neighbor_Tet(std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, bool neighb_type) {

	//using node to find neighbor tetrahedral 
	if (neighb_type) {
		for (int i = 0; i < 4; i++) {
			QMeshNode* thisNode = Tetra->GetNodeRecordPtr(i + 1);
			for (GLKPOSITION Pos = thisNode->GetTetraList().GetHeadPosition(); Pos;) {
				QMeshTetra* ConnectTetra = (QMeshTetra*)thisNode->GetTetraList().GetNext(Pos);

				if (ConnectTetra == Tetra) continue;
				bool exist_in_set = false;

				for (int j = 0; j < TetraSet.size(); j++) {
					if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) {
						exist_in_set = true; break;
					}
				}
				if (exist_in_set) continue;

				//the connected tetra has not being processed before
				TetraSet.push_back(ConnectTetra);
			}
		}
	}


	//using face to find neighbor tetrahedral 
	else {
		for (int i = 0; i < 4; i++) {
			QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);

			if (thisFace->GetLeftTetra() == NULL || thisFace->GetRightTetra() == NULL) continue;

			QMeshTetra* ConnectTetra = thisFace->GetRightTetra();
			if (Tetra == ConnectTetra) ConnectTetra = thisFace->GetLeftTetra();

			if (ConnectTetra == Tetra) continue;
			bool exist_in_set = false;

			for (int j = 0; j < TetraSet.size(); j++) {
				if (ConnectTetra->GetIndexNo() == TetraSet[j]->GetIndexNo()) {
					exist_in_set = true; break;
				}
			}
			if (exist_in_set) continue;

			//the connected tetra has not being processed before
			TetraSet.push_back(ConnectTetra);
		}
	}
}

void Deform::_compTetMeshVolumeMatrix(QMeshPatch* patch) {

	//-- This function calculate the volume matrix 
	//   for each tetrahedral elements and installed in formate
	/* [   b1 c1 d1
		   b2 c2 d2
		   b3 c3 d3
		   b4 c4 d4   ] */

	for (GLKPOSITION Pos = patch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)patch->GetTetraList().GetNext(Pos);

		Eigen::MatrixXd VolumeMatrix(4, 3);

		Eigen::Vector3d aa, bb, cc, dd, pp;
		Tet->CalCenterPos(pp(0), pp(1), pp(2));

		Tet->GetNodeRecordPtr(1)->GetCoord3D(aa(0), aa(1), aa(2));
		Tet->GetNodeRecordPtr(2)->GetCoord3D(bb(0), bb(1), bb(2));
		Tet->GetNodeRecordPtr(3)->GetCoord3D(cc(0), cc(1), cc(2));
		Tet->GetNodeRecordPtr(4)->GetCoord3D(dd(0), dd(1), dd(2));

		Eigen::Vector3d vap = pp - aa;
		Eigen::Vector3d vbp = pp - bb;

		Eigen::Vector3d vab = bb - aa;
		Eigen::Vector3d vac = cc - aa;
		Eigen::Vector3d vad = dd - aa;

		Eigen::Vector3d vbc = cc - bb;
		Eigen::Vector3d vbd = dd - bb;

		Eigen::Vector3d bd_bc = vbd.cross(vbc);
		Eigen::Vector3d ac_ad = vac.cross(vad);
		Eigen::Vector3d ad_ab = vad.cross(vab);
		Eigen::Vector3d ab_ac = vab.cross(vac);
		double volumeTet = Tet->CalVolume() * 6;

		VolumeMatrix.row(0) = bd_bc / volumeTet;
		VolumeMatrix.row(1) = ac_ad / volumeTet;
		VolumeMatrix.row(2) = ad_ab / volumeTet;
		VolumeMatrix.row(3) = ab_ac / volumeTet;

		Tet->VolumeMatrix = VolumeMatrix;
	}
}

void Deform::_detectBottomTet(double threshold) {

	//mark the bottom
	double minHight = 99999.0;
	for (GLKPOSITION pos = m_tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(pos);
		double pp[3] = { 0 }; node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < minHight) minHight = pp[1];
	}

	std::printf("The minume height: %f.\n", minHight);

	for (GLKPOSITION pos = m_tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(pos);
		double pp[3] = { 0 }; node->GetCoord3D(pp[0], pp[1], pp[2]);
		if (pp[1] < minHight + threshold) {
			node->isBottomNode = true;
		}
	}
	// mark bottom tet element
	int bottom_Num = 0;
	for (GLKPOSITION pos = m_tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(pos);
		tetra->isBottomTet = false;
		for (int i = 0; i < 4; i++) {
			if (tetra->GetNodeRecordPtr(i + 1)->isBottomNode) {
				tetra->isBottomTet = true;
				bottom_Num++;
				break;
			}
		}
	}
	std::printf("The bottom tet Num: %d.\n", bottom_Num);
	// clear isBottomNode flag
	for (GLKPOSITION pos = m_tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(pos);
		node->isBottomNode = false;
	}
}

void Deform::_getNodeFaceTet_num() {

	node_Num = m_tetPatch->GetNodeNumber();
	tet_Num = m_tetPatch->GetTetraNumber();
	face_Num = m_tetPatch->GetFaceNumber();
	neighborScale_face_num = 0;
	for (GLKPOSITION pos = m_tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(pos);

		if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL)	neighborScale_face_num++;
	}

	std::printf("All face num %d , neighborScale_face_num %d.\n", face_Num, neighborScale_face_num);
	std::printf("A matrix size %d x %d\n", (5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
}

void Deform::_initial_variables_and_space(bool is_useDeformed_position) {

	//---------------------------------------------------------------------------------------------------------------
	//apply for space of variables of inner loop
	frame_Local_new.resize(tet_Num);
	frame_Local_initial_inverse.resize(tet_Num);
	frame_Local_initial.resize(tet_Num);
	std::cout << "open up space for local frames." << std::endl;

	//---------------------------------------------------------------------------------------------------------------
	//pre-compute the initial frame and inverse of initla frame
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tet = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tet->GetIndexNo();
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
		double center[3] = { 0 };
		QMeshNode* nodes[4];

		for (int i = 0; i < 4; i++) {
			nodes[i] = Tet->GetNodeRecordPtr(i + 1);
			nodes[i]->GetCoord3D_last(P(0, i), P(1, i), P(2, i));
			if (is_useDeformed_position) {
				P(0, i) = nodes[i]->deformed_coord3D[0];
				P(1, i) = nodes[i]->deformed_coord3D[1];
				P(2, i) = nodes[i]->deformed_coord3D[2];
			}
			for (int j = 0; j < 3; j++) center[j] += P(j, i);
		} for (int j = 0; j < 3; j++) center[j] /= 4;
		for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) P(j, i) -= center[j];

		frame_Local_initial[fdx] = P;
		frame_Local_initial_inverse[fdx] = Eigen::MatrixXd::Zero(3, 4);
		frame_Local_initial_inverse[fdx] = (P.transpose()).completeOrthogonalDecomposition().pseudoInverse();
		//std::cout << frame_Local_initial[fdx] << std::endl << std::endl;
	}
	std::printf("compute local frame and its inverse.\n");
}

void Deform::_build_matrix(bool is_onlySR) {

	//---------------------------------------------------------------------------------------------------------------
	//apply for space of variables of outer loop
	vector_X_Pos_Scale.resize(3);
	vector_b.resize(3);
	for (int i = 0; i < 3; i++) {
		vector_X_Pos_Scale[i] = Eigen::VectorXd::Zero(node_Num + tet_Num);
		vector_b[i] = Eigen::VectorXd::Zero(5 * tet_Num + neighborScale_face_num);
	}

	matrix_A_4x.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
	matrix_A_transpose_4x.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
	matrix_A_4y.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
	matrix_A_transpose_4y.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));
	matrix_A_4z.resize((5 * tet_Num + neighborScale_face_num), (node_Num + tet_Num));
	matrix_A_transpose_4z.resize((node_Num + tet_Num), (5 * tet_Num + neighborScale_face_num));

	//fill A matrix for x,y,z
	//block 11 for x,y,z
	matrix_A_4x.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
	matrix_A_4y.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));
	matrix_A_4z.reserve(Eigen::VectorXi::Constant(5 * tet_Num + neighborScale_face_num, 1000));

	float c1 = -0.25, c2 = 0.75;
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);

		int fdx = Tetra->GetIndexNo() * 4;	int vdxArr[4];
		for (int i = 0; i < 4; i++) vdxArr[i] = Tetra->GetNodeRecordPtr(i + 1)->GetIndexNo();

		double weight = 1.0;

		if (is_onlySR) { 
			if (Tetra->isTensileorCompressSelect)	
				weight = m_outer_criticalTet_weight; 
		}
		else {
			if (Tetra->is_surfKeptRegion)
				weight = m_outer_criticalTet_weight;
		}

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (i == j) {
					matrix_A_4x.insert(fdx + j, vdxArr[i]) = c2 * weight;
					matrix_A_4y.insert(fdx + j, vdxArr[i]) = c2 * weight;
					matrix_A_4z.insert(fdx + j, vdxArr[i]) = c2 * weight;
				}
				else {
					matrix_A_4x.insert(fdx + j, vdxArr[i]) = c1 * weight;
					matrix_A_4y.insert(fdx + j, vdxArr[i]) = c1 * weight;
					matrix_A_4z.insert(fdx + j, vdxArr[i]) = c1 * weight;
				}
			}
		}
	}

	//block 12 for x,y,z
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);
		int fdx = Tetra->GetIndexNo();

		double weight = 1.0;
		if (is_onlySR) {
			if (Tetra->isTensileorCompressSelect)
				weight = m_outer_criticalTet_weight;
		}
		else {
			if (Tetra->is_surfKeptRegion)
				weight = m_outer_criticalTet_weight;
		}

		for (int i = 0; i < 4; i++) {
			matrix_A_4x.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](0, i); // x row
			matrix_A_4y.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](1, i); // y row
			matrix_A_4z.insert((fdx * 4 + i), (node_Num + fdx)) = -weight * frame_Local_new[fdx](2, i); // z row
		}
	}
	//block 21 equals to zero
	//block 22 for x,y,z
	int constraint_ind = 0;
	for (GLKPOSITION Pos = m_tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(Pos);

		if (Face->GetLeftTetra() == NULL || Face->GetRightTetra() == NULL) continue;

		matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_outer_neighborScale_weight;
		matrix_A_4x.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_outer_neighborScale_weight;

		matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_outer_neighborScale_weight;
		matrix_A_4y.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_outer_neighborScale_weight;

		matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetLeftTetra()->GetIndexNo()) = m_outer_neighborScale_weight;
		matrix_A_4z.insert(4 * tet_Num + constraint_ind, node_Num + Face->GetRightTetra()->GetIndexNo()) = -m_outer_neighborScale_weight;

		constraint_ind++;
	}
	//block 31 equals to zero
	//block 32 for x,y,z
	constraint_ind = 0;
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);
		int fdx = Tetra->GetIndexNo();

		matrix_A_4x.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_outer_regularScale_weight;
		matrix_A_4y.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_outer_regularScale_weight;
		matrix_A_4z.insert((4 * tet_Num + neighborScale_face_num + constraint_ind), (node_Num + fdx)) = m_outer_regularScale_weight;

		constraint_ind++;
	}

	std::cout << "Finish fill A matrix." << std::endl;

	//fill b vector for x,y,z
	//special case of regular term
	constraint_ind = 0;
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);

		for (int i = 0; i < 3; i++) {

			vector_b[i](4 * tet_Num + neighborScale_face_num + constraint_ind) = m_outer_regularScale_weight;
		}
		constraint_ind++;
	}

	std::cout << "Finish fill b vector." << std::endl;
}

void Deform::_solve_eqation() {

	long time = clock();

	// Factorize A matrix
	matrix_A_4x.makeCompressed();
	Eigen::SparseMatrix<double> matATA_4x(node_Num + tet_Num, node_Num + tet_Num);
	matrix_A_transpose_4x = matrix_A_4x.transpose();
	matATA_4x = matrix_A_transpose_4x * matrix_A_4x;
	Solver_ASAP_4x.compute(matATA_4x);

	matrix_A_4y.makeCompressed();
	Eigen::SparseMatrix<double> matATA_4y(node_Num + tet_Num, node_Num + tet_Num);
	matrix_A_transpose_4y = matrix_A_4y.transpose();
	matATA_4y = matrix_A_transpose_4y * matrix_A_4y;
	Solver_ASAP_4y.compute(matATA_4y);

	matrix_A_4z.makeCompressed();
	Eigen::SparseMatrix<double> matATA_4z(node_Num + tet_Num, node_Num + tet_Num);
	matrix_A_transpose_4z = matrix_A_4z.transpose();
	matATA_4z = matrix_A_transpose_4z * matrix_A_4z;
	Solver_ASAP_4z.compute(matATA_4z);

	std::printf("factorize materix A\n");

	// Solve x vector
	Eigen::VectorXd ATb_4x = matrix_A_transpose_4x * vector_b[0];
	vector_X_Pos_Scale[0] = Solver_ASAP_4x.solve(ATb_4x);
	// Solve y vector
	Eigen::VectorXd ATb_4y = matrix_A_transpose_4y * vector_b[1];
	vector_X_Pos_Scale[1] = Solver_ASAP_4y.solve(ATb_4y);
	// Solve z vector
	Eigen::VectorXd ATb_4z = matrix_A_transpose_4z * vector_b[2];
	vector_X_Pos_Scale[2] = Solver_ASAP_4z.solve(ATb_4z);
	
	std::printf("solve eqation finish, it spends %f s.\n", (double(clock() - time)) / CLOCKS_PER_SEC);
}

void Deform::_extract_info() {

	// Update vertex
	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);
		int idx = node->GetIndexNo();

		double xx, yy, zz;
		node->SetCoord3D(vector_X_Pos_Scale[0](idx), vector_X_Pos_Scale[1](idx), vector_X_Pos_Scale[2](idx));
		node->GetCoord3D(xx, yy, zz);
		//std::printf("New coordinate xx %f , yy %f , zz %f.\n", xx, yy, zz);
	}

	// record the scale value
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);
		int fdx = Tetra->GetIndexNo();

		for (int i = 0; i < 3; i++) {
			Tetra->scaleValue_vector[i] = vector_X_Pos_Scale[i](node_Num + fdx);
			//std::cout << vector_X_Pos_Scale[i](node_Num + fdx) << " ";
		}
		//std::cout << std::endl;
	}
}

void Deform::_estimate_localRotation(QMeshTetra* Tetra, int index, Eigen::Matrix3d& R) {

	double center[3] = { 0 };
	Tetra->CalCenterPos(center[0], center[1], center[2]);

	//This tP is each current frame in each tet.
	Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
	QMeshNode* nodes[4];
	for (int i = 0; i < 4; i++) {
		nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
		nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
	}
	for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

	Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
	Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);

	T = frame_Local_initial_inverse[index] * tP;
	T_transpose = T.transpose();

	//// Eigen SVD decomposition /////
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
	R = U * V.transpose();
}

void Deform::_quaternionSmooth_SR() {

	int critical_tet_num = 0;
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);
		if (Tet->isTensileorCompressSelect 
			|| Tet->isBottomTet 
			|| Tet->is_holeRegion
			)
			critical_tet_num++;
	}

	// Pre-compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(tet_Num + critical_tet_num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, tet_Num + critical_tet_num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(tet_Num + critical_tet_num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(tet_Num + critical_tet_num, 1000));
	//fill L matrix
	// Part 1: ws(1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	int keep_constraint_lineNum = 0;
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_inner_smooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo())
				= -m_inner_smooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->isBottomTet) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = m_inner_bottom_weight;
			keep_constraint_lineNum++;
		}
		else if (Tetra->is_holeRegion) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = m_inner_hole_weight;
			keep_constraint_lineNum++;
		}
		else if (Tetra->isTensileorCompressSelect) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = m_inner_critical_weight;
			keep_constraint_lineNum++;
		}

	}// finish Lap matrix build

	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	//std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < m_tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	keep_constraint_lineNum = 0;
	for (int m = 0; m < m_tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isBottomTet) {
			vector_matrix_b.row(keep_constraint_lineNum + tet_Num)
				<< Tetra->R_quaternion.w() * m_inner_bottom_weight
				, Tetra->R_quaternion.vec().transpose()* m_inner_bottom_weight;
			keep_constraint_lineNum++;
		}
		else if (Tetra->is_holeRegion) {
			vector_matrix_b.row(keep_constraint_lineNum + tet_Num)
				<< Tetra->R_quaternion.w() * m_inner_hole_weight
				, Tetra->R_quaternion.vec().transpose()* m_inner_hole_weight;
			keep_constraint_lineNum++;
		}
		else if (Tetra->isTensileorCompressSelect) {
			vector_matrix_b.row(keep_constraint_lineNum + tet_Num)
				<< Tetra->R_quaternion.w() * m_inner_critical_weight
				, Tetra->R_quaternion.vec().transpose()* m_inner_critical_weight;
			keep_constraint_lineNum++;
		}
	}
	//std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < m_tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		//if (Tetra->isOverhangTet == false) continue;
		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

void Deform::_quaternionSmooth_SQ() {

	int critical_tet_num = 0;
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos != nullptr;) {
		QMeshTetra* Tet = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);
		if (Tet->isBottomTet
			|| Tet->is_surfKeptRegion
			)
			critical_tet_num++;
	}

	// Pre-compute the Laplacian Rotation Smooth Matrix A
	// Define matrix for global quaternion smooth
	Eigen::SparseMatrix<double> matLap_A;	matLap_A.resize(tet_Num + critical_tet_num, tet_Num);
	Eigen::SparseMatrix<double> matLap_AT;	matLap_AT.resize(tet_Num, tet_Num + critical_tet_num);
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> GlobalQuatSmooth_Solver;
	Eigen::MatrixXd vector_Quat_x = Eigen::MatrixXd::Zero(tet_Num, 4);
	Eigen::MatrixXd vector_matrix_b = Eigen::MatrixXd::Zero(tet_Num + critical_tet_num, 4);
	//give memory to sparse matrix, to accerate the insert speed
	matLap_A.reserve(Eigen::VectorXi::Constant(tet_Num + critical_tet_num, 1000));
	//fill L matrix
	// Part 1: ws(1 -1/k -1/k ... 0)*(q1 q2 q3 ... 0)T = (0 0 0 ... 0)T
	// Part 2: (w1 0 0 ... 0)*(q1 q2 q3 ... 0)T = (w1* t1 w2* t2 ...)T
	int keep_constraint_lineNum = 0;
	for (int m = 0; m < tet_Num; m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];
		matLap_A.insert(m, m) = m_inner_smooth_weight;
		for (int i = 0; i < Tetra->neighborCell.size(); i++) {
			matLap_A.insert(m, Tetra->neighborCell[i]->GetIndexNo())
				= -m_inner_smooth_weight / Tetra->neighborCell.size();
		}

		if (Tetra->isBottomTet) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = m_inner_bottom_weight;
			keep_constraint_lineNum++;
		}
		else if (Tetra->is_surfKeptRegion) {
			matLap_A.insert(keep_constraint_lineNum + tet_Num, m) = m_inner_surfKeep_weight;
			keep_constraint_lineNum++;
		}

	}// finish Lap matrix build

	matLap_A.makeCompressed();
	matLap_AT = matLap_A.transpose();
	Eigen::SparseMatrix<double> matLap_ATA(tet_Num, tet_Num);
	matLap_ATA = matLap_AT * matLap_A;
	GlobalQuatSmooth_Solver.compute(matLap_ATA);
	//std::cout << "Fill Laplacian matrix and factorize it" << std::endl;

	//record the quaternion after local process
	for (int m = 0; m < m_tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		Eigen::Quaterniond quat(Tetra->R_estimate);
		Tetra->R_quaternion = quat;
	}
	//fill D matrix
	keep_constraint_lineNum = 0;
	for (int m = 0; m < m_tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		if (Tetra->isBottomTet) {
			vector_matrix_b.row(keep_constraint_lineNum + tet_Num)
				<< Tetra->R_quaternion.w() * m_inner_bottom_weight
				, Tetra->R_quaternion.vec().transpose()* m_inner_bottom_weight;
			keep_constraint_lineNum++;
		}
		else if (Tetra->is_surfKeptRegion) {
			vector_matrix_b.row(keep_constraint_lineNum + tet_Num)
				<< Tetra->R_quaternion.w() * m_inner_surfKeep_weight
				, Tetra->R_quaternion.vec().transpose()* m_inner_surfKeep_weight;
			keep_constraint_lineNum++;
		}
	}
	//std::cout << "Fill D matrix" << std::endl;
	//solve equation
	for (int n = 0; n < 4; n++) {
		Eigen::VectorXd Lap_ATb = matLap_AT * vector_matrix_b.col(n);
		vector_Quat_x.col(n) = GlobalQuatSmooth_Solver.solve(Lap_ATb);
	}
	//update the rotation matrix generated from result quaternion
	for (int m = 0; m < m_tetPatch->GetTetraNumber(); m++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[m];

		//if (Tetra->isOverhangTet == false) continue;
		Tetra->R_quaternion.w() = vector_Quat_x(m, 0);
		Eigen::Vector3d Qvector = { vector_Quat_x(m, 1),vector_Quat_x(m, 2),vector_Quat_x(m, 3) };
		Tetra->R_quaternion.vec() = Qvector;
		Tetra->R_estimate = Tetra->R_quaternion.normalized().toRotationMatrix();
	}
}

Eigen::Matrix3d Deform::_cal_rotationMatrix_SR(Eigen::Vector3d principal_Stress_Dir) {

	Eigen::Vector3d projectDir; // assume the printing direction is (0,1,0)
	projectDir << principal_Stress_Dir(0), 0.0, principal_Stress_Dir(2);
	projectDir.normalized();

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();

	rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(principal_Stress_Dir, projectDir);

	return rotationMatrix;
}

void Deform::_calFabricationEnergy_SR() {

	//Get the fabrication energy of reinforcement
	double critical_energy = 0.0;
	for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];

		int fdx = Tetra->GetIndexNo();
		if (Tetra->isTensileorCompressSelect) {
			double center[3] = { 0 };
			Tetra->CalCenterPos(center[0], center[1], center[2]);

			//This tP is current frame.
			Eigen::MatrixXd tP = Eigen::MatrixXd::Zero(4, 3);
			QMeshNode* nodes[4];
			for (int i = 0; i < 4; i++) {
				nodes[i] = Tetra->GetNodeRecordPtr(i + 1);
				nodes[i]->GetCoord3D(tP(i, 0), tP(i, 1), tP(i, 2));
			}
			for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) tP(i, j) -= center[j];

			Eigen::Matrix3d T = Eigen::Matrix3d::Zero(3, 3);
			Eigen::Matrix3d T_transpose = Eigen::Matrix3d::Zero(3, 3);
			Eigen::Matrix3d R = Eigen::Matrix3d::Zero(3, 3);

			T = frame_Local_initial_inverse[fdx] * tP;
			T_transpose = T.transpose();

			///// Get current tau_max by //// Eigen SVD decomposition /////
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(T_transpose, Eigen::ComputeThinU | Eigen::ComputeThinV);
			Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
			R = U * V.transpose();
			Eigen::Vector3d new_Stress_Dir = R * Tetra->tau_max;
			critical_energy += pow((acos(new_Stress_Dir.dot(printDir)) - PI / 2), 2);
		}
	}
	std::cout << "Fabrication energy (StrengthReinforcement) = " << critical_energy << std::endl;
}

void Deform::_calFabricationEnergy_SQ() {

	//Get the fabrication energy of surface quality
	double critical_energy = 0.0;
	for (GLKPOSITION Pos = m_tetPatch->GetTetraList().GetHeadPosition(); Pos;) {
		QMeshTetra* Tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(Pos);

		if (Tetra->is_surfKeptRegion) {

			QMeshFace* criticalFace = NULL;
			for (int i = 0; i < 4; i++) {
				QMeshFace* thisFace = Tetra->GetFaceRecordPtr(i + 1);
				if (thisFace->is_keptFace) {
					criticalFace = thisFace;
					break;
				}
			}
			if (criticalFace == NULL) std::cout << "ERROR! No quality protect face." << std::endl;

			Eigen::Vector3d normal;
			criticalFace->CalPlaneEquation();
			criticalFace->GetNormal(normal(0), normal(1), normal(2));

			//std::cout << "critcal normal" << normal.transpose() << std::endl;
			if (normal.dot(this->printDir) > 0)	critical_energy += acos(normal.dot(printDir));
			else critical_energy += acos(-normal.dot(this->printDir));

		}
	}
	std::cout << "Fabrication energy (SurfaceQuality) = " << critical_energy << std::endl;
}

void Deform::_get_energy_innerLoop_SR() {

	//Get the fabrication energy of reinforcement
	double critical_energy = 0.0;
	int criticalEle_num = 0;
	for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];

		if (Tetra->isTensileorCompressSelect) {
			Eigen::Vector3d new_Stress_Dir = Tetra->R_estimate * Tetra->tau_max;
			critical_energy += fabs(acos(new_Stress_Dir.dot(printDir)) - PI / 2);
			criticalEle_num++;
		}
	}

	std::cout << "Fabrication energy (StrengthReinforcement) = "
		<< (critical_energy / criticalEle_num) * 180.0 / PI << std::endl;
}

Eigen::Matrix3d Deform::_cal_rotationMatrix_Hole(Eigen::Vector3d current_HoleDir) {

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	if (current_HoleDir.dot(printDir) > 0)
		rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(current_HoleDir, this->printDir);
	else
		rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(current_HoleDir, -this->printDir);

	return rotationMatrix;
}

void Deform::_getHole_index_face_tet() {

	//transfer the isHandleDraw to is_holeRegion and initialize them
	for (GLKPOSITION Pos = m_tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(Pos);

		if (Face->inner) continue;

		if (Face->isHandleDraw) Face->hole_index = -1;
		else Face->hole_index = 0;
	}

	//get region index of holes
	bool allNodeChecked = true;
	int splitTime = 0;
	do {

		splitTime++;

		//-- Find start face
		for (GLKPOSITION Pos = m_tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(Pos);
			if (Face->inner) continue;

			if (Face->hole_index < 0) {
				Face->hole_index = splitTime; 
				break; 
			}
		}

		//-- Run flooding searching
		this->_holeRegion_flooding(m_tetPatch, splitTime);

		//-- Detect if all the node has been checked
		for (GLKPOSITION Pos = m_tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* Face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(Pos);
			if (Face->inner) continue;

			allNodeChecked = true;
			if (Face->hole_index < 0) { allNodeChecked = false; break; }
		}
	} while (!allNodeChecked);

	std::cout << "There are " << splitTime << " Hole(s)." << std::endl;
	holeNum = splitTime;

	//transfer the index to tetra
	for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];
		Tetra->is_holeRegion = false;
	}

	for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];

		for (int j = 0; j < 4; j++) {
			QMeshFace* oneFace = Tetra->GetFaceRecordPtr(j + 1);

			if ((oneFace->hole_index > 0) && (!oneFace->inner)) {
				Tetra->is_holeRegion = true;
				Tetra->hole_index = oneFace->hole_index;
				break;
			}
		}
	}

	// clear handle/fix_flag of nodes
	for (GLKPOSITION pos = m_tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(pos);
		node->isHandle = false;
	}
	// clear face handle/fix draw
	for (GLKPOSITION pos = m_tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(pos);

		face->isHandleDraw = false;
	}

}

void Deform::_holeRegion_flooding(QMeshPatch* patch, int index) {

	int StopFlag = 0;

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* thisFace = (QMeshFace*)patch->GetFaceList().GetNext(Pos);

		if (thisFace->inner) continue;

		if (thisFace->hole_index == index) {
			for (int i = 0; i < 3; i++) {

				QMeshEdge* thisEdge = thisFace->GetEdgeRecordPtr(i + 1);
				for (GLKPOSITION Pos_facesOfEdge = thisEdge->GetFaceList().GetHeadPosition(); Pos_facesOfEdge;) {
					QMeshFace* oneFaceOFedge = (QMeshFace*)thisEdge->GetFaceList().GetNext(Pos_facesOfEdge);

					if (oneFaceOFedge->inner || (!oneFaceOFedge->isHandleDraw)) continue;
					if (oneFaceOFedge == thisFace) continue;


					if (oneFaceOFedge->hole_index == index) continue;
					else {
						oneFaceOFedge->hole_index = index;
						StopFlag++;
					}
				}
			}
		}
	}
	//cout << "This iteration adding node number = " << StopFlag << endl;

	if (StopFlag > 0) _holeRegion_flooding(patch, index);
}

void Deform::_update_hole_direction() {

	for (int i = 0; i < holeNum; i++) {
		this->_cal_hole_direction(i + 1); //hole index start from 1
	}
}

void Deform::_cal_hole_direction(int whichHole) {

	std::vector<Eigen::Vector3d> nodeSet;
	for (GLKPOSITION Pos = m_tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(Pos);
		if (Face->inner) continue;

		if (Face->hole_index == whichHole) {

			Eigen::Vector3d center_coordinate = Eigen::Vector3d::Zero();
			Face->CalCenterPos(center_coordinate[0], center_coordinate[1], center_coordinate[2]);
			nodeSet.push_back(center_coordinate);
		}
	}

	//PCA to get central axis
	Eigen::MatrixXd X(nodeSet.size(), 3), C(3, 3);
	Eigen::MatrixXd vec, val;
	for (int i = 0; i < nodeSet.size(); i++) {
		X.row(i) = nodeSet[i];
	}

	Eigen::MatrixXd meanval = X.colwise().mean();
	Eigen::RowVector3d meanvecRow = meanval;
	X.rowwise() -= meanvecRow;
	C = X.adjoint() * X;
	C = C.array() / (X.rows() - 1);
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(C);
	vec = eig.eigenvectors();
	val = eig.eigenvalues();

	//std::cout << val << std::endl;
	//std::cout << vec << std::endl;

	std::ptrdiff_t max_index, min_index, mid_index;
	Eigen::Vector3d eigenvalueABS; for (int i = 0; i < 3; i++) eigenvalueABS(i) = abs(val(i));
	eigenvalueABS.maxCoeff(&max_index); //Max eigen value (ABS)
	eigenvalueABS.minCoeff(&min_index); //Min eigen value (ABS)
	for (int i = 0; i < 3; i++) {
		if (i == max_index || i == min_index) continue;
		mid_index = i;
	}

	// find the most different abs(eigenVale) hole's property
	double min_Abs_eigenValue = eigenvalueABS[min_index];
	double mid_Abs_eigenValue = eigenvalueABS[mid_index];
	double max_Abs_eigenValue = eigenvalueABS[max_index];
	Eigen::Vector3d centerAxis = Eigen::Vector3d::Zero();
	if ((mid_Abs_eigenValue - min_Abs_eigenValue) < (max_Abs_eigenValue - mid_Abs_eigenValue)) {
		centerAxis = vec.col(max_index); //vector
	}
	else {
		centerAxis = vec.col(min_index); //vector
	}

	
	for (int i = 0; i < m_tetPatch->GetTetraNumber(); i++) {
		QMeshTetra* Tetra = tetraPatch_elementSet[i];
		
		if (Tetra->is_holeRegion && Tetra->hole_index == whichHole) {

			Tetra->holeDir = centerAxis;
		}
	}
}

void Deform::_moveModel2Center() {

	double centerXx, centerYy, centerZz;
	double minYy = 99999999.99;
	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		if (minYy > yy) minYy = yy;
		centerXx += xx; centerYy += yy; centerZz += zz;
	}

	centerXx /= m_tetPatch->GetNodeNumber();
	centerYy /= m_tetPatch->GetNodeNumber();
	centerZz /= m_tetPatch->GetNodeNumber();

	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);

		xx -= centerXx;
		yy -= minYy;
		zz -= centerZz;

		node->SetCoord3D(xx, yy, zz);
		//node->SetCoord3D_last(xx, yy, zz);
	}
}

void Deform::_record_deformed_position() {

	for (GLKPOSITION Pos = m_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(Pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		node->deformed_coord3D << xx, yy, zz;
	}

}

void Deform::_record_hightField() {

	// Get height field (scalar field)
	double boundingBox[6]; double heightRange = 0.0;
	m_tetPatch->ComputeBoundingBox(boundingBox);
	heightRange = boundingBox[3] - boundingBox[2];
	for (GLKPOSITION pos = m_tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(pos);

		double xx, yy, zz;
		node->GetCoord3D(xx, yy, zz);
		node->scalarField_init = yy;
		node->scalarField = (yy - boundingBox[2]) / heightRange;
	}
}

void Deform::_get_KeptSurf_tet() {

	for (GLKPOSITION pos = m_tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(pos);
		face->is_keptFace = false;
	}

	// tetrahedral containing 1 critical face is defined as containProtectFace
	for (GLKPOSITION pos = m_tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(pos);

		tetra->is_surfKeptRegion = false;
		int critical_Face_NUM = 0;
		QMeshFace* critical_face = NULL;
		for (int i = 0; i < 4; i++) {

			QMeshFace* each_face = tetra->GetFaceRecordPtr(i + 1);
			if (each_face->isFixedDraw
				&& !(each_face->inner)) {
				critical_Face_NUM++;
				critical_face = each_face;
			}
		}

		if (critical_Face_NUM == 1) {
			tetra->is_surfKeptRegion = true;
			tetra->kept_Face = critical_face;
			critical_face->is_keptFace = true;
		}
	}

	// clear handle/fix_flag of nodes
	for (GLKPOSITION pos = m_tetPatch->GetNodeList().GetHeadPosition(); pos != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetPatch->GetNodeList().GetNext(pos);
		
		node->isFixed = false;
	}
	// clear face handle/fix draw
	for (GLKPOSITION pos = m_tetPatch->GetFaceList().GetHeadPosition(); pos != nullptr;) {
		QMeshFace* face = (QMeshFace*)m_tetPatch->GetFaceList().GetNext(pos);
		
		face->isFixedDraw = false;
	}

}

Eigen::Matrix3d Deform::_cal_rotationMatrix_SQ(QMeshFace* face) {

	face->CalPlaneEquation();
	Eigen::Vector3d normal;
	face->GetNormal(normal[0], normal[1], normal[2]);
	normal.normalize();

	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	if (normal.dot(printDir) > 0)
		rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, this->printDir);
	else
		rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(normal, -this->printDir);

	return rotationMatrix;
}

void Deform::_update_outerLoop_weights(double critical, double neighbor, double regular) {

	m_outer_criticalTet_weight = critical;
	m_outer_neighborScale_weight = neighbor;
	m_outer_regularScale_weight = regular;

	std::cout << "\n/********************************/\n\nOuter loop weights:";
	std::cout << "\n\n--> critical \t" << m_outer_criticalTet_weight
		<< "\n--> neighbor \t" << m_outer_neighborScale_weight
		<<"\n--> regular \t" << m_outer_regularScale_weight 
		<< "\n\n/********************************/\n";
}

void Deform::update_StreeField() {

	for (GLKPOSITION pos = m_tetPatch->GetTetraList().GetHeadPosition(); pos != nullptr;) {
		QMeshTetra* tetra = (QMeshTetra*)m_tetPatch->GetTetraList().GetNext(pos);

		if (tetra->isTensileorCompressSelect){
			tetra->tau_max = m_tetPatch->model_rotMat * tetra->tau_max;
			tetra->tau_mid = m_tetPatch->model_rotMat * tetra->tau_mid;
			tetra->tau_min = m_tetPatch->model_rotMat * tetra->tau_min;
		}
	}

}