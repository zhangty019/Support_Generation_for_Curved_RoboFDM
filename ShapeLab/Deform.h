#pragma once
#include "../QMeshLib/PolygenMesh.h"
#include <Eigen/PardisoSupport>

class Deform
{
public:
	Deform() {};
	~Deform() {};

	void initial(PolygenMesh* polygenMesh_TetModel);
	bool preProcess_4StrengthReinforcement();
	void update_StreeField();
	void runASAP_SR();
	void runASAP_SR_SQ();
	void postProcess();

private:
	void _setParameters(
		int inner_cycle, 
		double inner_critical_weight, double inner_smooth_weight,
		int outer_cycle,
		double outer_criticalTet_weight, 
		double outer_neighborScale_weight,
		double outer_regularScale_weight);
	void _index_initial(QMeshPatch* patch, bool is_TetMesh);
	void _build_tetraSet_4SpeedUp(QMeshPatch* patch);
	void _moveModelup2ZeroHeight(QMeshPatch* patch);
	void _record_neighbor_Tet();
	void _detect_each_neighbor_Tet(
		std::vector< QMeshTetra* >& TetraSet, QMeshTetra* Tetra, bool neighb_type);
	void _compTetMeshVolumeMatrix(QMeshPatch* patch);
	void _detectBottomTet(double threshold);

	void _getNodeFaceTet_num();
	void _initial_variables_and_space(bool is_useDeformed_position);
	void _build_matrix(bool is_onlySR);
	void _solve_eqation();
	void _extract_info();
	void _quaternionSmooth_SR();
	void _quaternionSmooth_SQ();
	void _estimate_localRotation(QMeshTetra* Tetra,	int index, Eigen::Matrix3d& R);
	Eigen::Matrix3d _cal_rotationMatrix_SR(Eigen::Vector3d principal_Stress_Dir);
	void _calFabricationEnergy_SR();
	void _get_energy_innerLoop_SR();
	void _calFabricationEnergy_SQ();


	Eigen::Matrix3d _cal_rotationMatrix_Hole(Eigen::Vector3d current_HoleDir);
	void _update_hole_direction();
	void _getHole_index_face_tet();
	void _cal_hole_direction(int whichHole); // start from 0
	void _holeRegion_flooding(QMeshPatch* patch, int index);

	void _moveModel2Center();
	void _record_deformed_position();
	void _record_hightField();

	void _get_KeptSurf_tet();
	Eigen::Matrix3d _cal_rotationMatrix_SQ(QMeshFace* face);

	void _update_outerLoop_weights(
		double critical, double neighbor, double regular);

private:
	int m_inner_cycle = 0;
	double m_inner_critical_weight = 0;
	double m_inner_smooth_weight = 0;

	//given
	double m_inner_bottom_weight = 15;
	double m_inner_hole_weight = 200;
	double m_inner_surfKeep_weight = 400;
	double m_outer_cycle_surfKeep = 2;
	double m_bottom_height = 1.0;
	Eigen::Vector3d printDir = { 0.0 ,1.0 ,0.0 };

	int m_outer_cycle = 0;
	double m_outer_criticalTet_weight = 0;
	double m_outer_neighborScale_weight = 0;
	double m_outer_regularScale_weight = 0;
	double tensile_ratio = 0;
	double compress_ratio = 0;

	QMeshPatch* m_tetPatch = NULL;
	std::vector<QMeshTetra*> tetraPatch_elementSet;
	std::string m_tetPatch_Name;

	//asap solution
	std::vector<Eigen::MatrixXd> frame_Local_new, frame_Local_initial_inverse, frame_Local_initial;
	std::vector<Eigen::VectorXd> vector_X_Pos_Scale, vector_b; //AX=b
	//
	Eigen::SparseMatrix<double> matrix_A_4x;
	Eigen::SparseMatrix<double> matrix_A_transpose_4x;
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4x;
	//
	Eigen::SparseMatrix<double> matrix_A_4y;
	Eigen::SparseMatrix<double> matrix_A_transpose_4y;
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4y;
	//
	Eigen::SparseMatrix<double> matrix_A_4z;
	Eigen::SparseMatrix<double> matrix_A_transpose_4z;
	Eigen::PardisoLDLT <Eigen::SparseMatrix<double>> Solver_ASAP_4z;
	//
	int node_Num = 0;
	int tet_Num = 0;
	int face_Num = 0;
	int neighborScale_face_num = 0;

	//hole related
	int holeNum = 0;
	bool m_isHole_considered = true;
	bool m_surfKeep_considered = true;
};