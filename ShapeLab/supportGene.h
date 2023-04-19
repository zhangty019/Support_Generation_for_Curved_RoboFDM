#pragma once
#include "../QMeshLib/PolygenMesh.h"

typedef struct qHullSet {
	int faceNum;	double* normalVec;	double* offset;
	int vertNum;	double* vertPos;
	unsigned int* faceTable;	//	Note that: the index starts from '1'
}QHULLSET;

class supportGene {

public:
	supportGene() {};
	~supportGene() {};

	void initial_4_envelope(
		QMeshPatch* tetPatch,
		QMeshPatch* platform_patch,
		PolygenMesh* supportEnvelope,
		PolygenMesh* supportRay,
		double tau, double expand_offset);
	void initial_Guess_SupportEnvelope();
	void compute_initialGuess_EnvelopeHull();

	void initial_4_supportSpace_Extraction(QMeshPatch* tetPatch, QMeshPatch* tetSupport);
	void generate_field_4_tetMeshes();
	void generate_field_4_tetMeshes_correction();
	void initial_4_treeGeneration(
		QMeshPatch* tetPatch, PolygenMesh* compatible_isoLayerSet, QMeshPatch* platform,
		PolygenMesh* tetModel_supportRay, double overhang_Deg);
	void organize_compatibleLayer_Index(PolygenMesh* compatible_isoLayerSet, int compatibleLayer_NUM);
	void generate_support_tree();
	void compute_Support_tree_Field();
	void build_tight_supportLayers();

private:

	void _index_initial(QMeshPatch* patch, bool is_TetMesh);

	QHULLSET* _mallocMemoryConvexHull(int faceNum, int vertNum);
	void _build_ConvexHull_mesh(QHULLSET* ConvexHULL, PolygenMesh* supportEnvelope);
	void _freeMemoryConvexHull(QHULLSET*& pConvexHull);

	void _transfer_ScalarField_2_supportSpace();
	void _transfer_VectorField_2_supportSpace();
	void _scalarField_4_supportSpace();
	double _get_init_scalarfield_of_NearestNode(
		QMeshPatch* init_tetMesh, QMeshNode* inquireNode_supportSpace);
	double _get_scalarfield_of_NearestNode(
		QMeshPatch* init_tetMesh, QMeshNode* inquireNode_supportSpace);

	Eigen::Vector3d _get_vectorfield_of_NearestFace(
		QMeshPatch* init_tetMesh, QMeshFace* inquireFace_supportSpace);
	void _vectorField_flooding_supportSpace(int loopTime);
	void _mark_bottomTet_flag(double threshold, QMeshPatch* tetMesh);
	void _compTetMeshVolumeMatrix(QMeshPatch* tetSupport);
	void _scalarField_2_vectorField(QMeshPatch* model, bool is_normalize);
	void _vectorField_2_scalarField(QMeshPatch* model, bool Up_Vector_Direction);

	void _markSupportFace();
	bool _is_single_OverHangface(QMeshFace* face);
	void _tracing_support_waterfall();
	void _cal_Ray_direction_onSourceNode(QMeshPatch* tet_Model);

	bool _find_targetNode(
		Eigen::Vector3d& oringinNode,
		Eigen::Vector3d& step_Direction,
		QMeshNode* Node,
		bool is_Merge);
	void _collect_Support_Polyline_fromTETsurface();

	void _build_support_tree();
	void _get_compatibleLayer_list(
		std::vector<std::vector<QMeshPatch*>>& compatibleLayer_matrix);
	bool _intersetTriangle(Eigen::Vector3d& orig, Eigen::Vector3d& dir,
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& insertP);
	void _compute_descend_Dir_hostNode(QMeshNode* hostNode, bool is_downWard);
	QMeshNode* _build_host_treeNodeEdge(QMeshNode* hostNode,
		const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
		QMeshPatch* supportRay_patch, int layer_height);
	bool _compute_descend_Dir_followNode(QMeshNode* followNode, QMeshNode* dropNode_hostNode);
	void _build_follow_treeNodeEdge(QMeshNode* followNode, QMeshNode* dropNode_hostNode,
		const std::vector<QMeshPatch*>& largeLayer_vector_m_1, bool is_merge,
		QMeshPatch* supportRay_patch, int layer_height);

	double _implicitDistance_Func_tree(Eigen::Vector3d& queryPnt);
	bool _lineIntersectSphere(Eigen::Vector3d& O, Eigen::Vector3d& T,
		Eigen::Vector3d& Center, double R, double& mu1, double& mu2);
	bool _planeCutSurfaceMesh_delete(QMeshPatch* surfaceMesh, bool Yup, double cutPlaneHeight);

	void _vectorField_flooding_supportSpace_unormalized(int loopTime);

	void _detect_overhang_edge();
	void _expand_shrink_overhang_region();
	void _vectorField_smooth(QMeshPatch* tetPatch);

public:

private:
	QMeshPatch* m_tetPatch = NULL;
	QMeshPatch* m_tetSupport = NULL;
	QMeshPatch* m_platform = NULL;
	PolygenMesh* m_layerSet = NULL;
	PolygenMesh* m_supportRay = NULL;
	PolygenMesh* m_supportEnvelope = NULL;

	double m_tau = 0.0;
	double m_bottom_Threshold = 1.0;
	double C = 1.0;
	double detectRadius_ratio = 1.5;	//detection radius ratio

	bool hostNode_decent_alongNormal = false; //true -> vertical

	double esp = -0.1;
	double pitch_grid = 3.2; // the grid pitch of the projected OV node set

	std::vector<std::vector<std::vector<QMeshNode*>>> vect3d;

	bool DEBUG = false;
	double trunk_height = 0.0;

	double m_expand_offset = 3.0;
};