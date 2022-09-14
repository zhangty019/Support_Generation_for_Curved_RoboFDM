#pragma once

class PolygenMesh;

typedef struct qHullSet {
	int faceNum;	double* normalVec;	double* offset;
	int vertNum;	double* vertPos;
	unsigned int* faceTable;	//	Note that: the index starts from '1'
}QHULLSET;

class supportGeneration {

public:
	supportGeneration() {};
	~supportGeneration() {};

	void initial(PolygenMesh* tetModel, PolygenMesh* layers, PolygenMesh* platform, PolygenMesh* supportRaySet,
		PolygenMesh* tight_supportLayerSet, PolygenMesh* toolpathSet_support, 
		PolygenMesh* toolpathSet_initial, std::string model_name);

	void initial(PolygenMesh* tetModel, PolygenMesh* platform, PolygenMesh* supportRaySet, std::string model_name);


	void build_SupportRays();
	void build_SupportRays_vertical();
	void build_Support_PolyLines();
	void build_Support_Polyline_fromTETsurface();
	void build_Support_Tree_fromTETsurface();
	void build_Support_Tree_fromTETsurface2();
	void build_Support_Tree_fromTETsurface3();

	void collect_SupportRays();
	void collect_SupportRays_vertical();
	void collect_Support_Polyline();
	void collect_Support_Polyline_fromTETsurface();

	void build_SupportSurface_MarchingCube();

	void compute_supportRayField();
	void compute_Support_polyLlne_Field();
	void compute_Support_tree_Field();

	void build_tight_supportLayers();
	void markSupportFace();
	void clearRay_insideModelTET_vertical();

	void toolPathCompute_support();
	void toolPathCompute_initial();
	void output_toolpath();

	void initial_Guess_SupportEnvelope();
	void computer_initialGuess_EnvelopeHull();

	bool supportGeneration::_lineIntersectSphere(Eigen::Vector3d& O, Eigen::Vector3d& T,
		Eigen::Vector3d& Center, double R, double& mu1, double& mu2);

	bool supportGeneration::_LineLineIntersection(Eigen::Vector3d& intersection,
		Eigen::Vector3d& p1, Eigen::Vector3d& v1,
		Eigen::Vector3d& p2, Eigen::Vector3d& v2);

	void output_OneSurfaceMesh(QMeshPatch* isoSurface, std::string path);
	void output_initial_Polyline();

private:

	bool _intersetTriangle(Eigen::Vector3d& orig, Eigen::Vector3d& dir,
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& insertP);

	/*bool supportGeneration::_lineIntersectSphere(Eigen::Vector3d& O, Eigen::Vector3d& T,
		Eigen::Vector3d& Center, double R, double& mu1, double& mu2);*/
	/*bool supportGeneration::_LineLineIntersection(Eigen::Vector3d& intersection,
		Eigen::Vector3d& p1, Eigen::Vector3d& v1,
		Eigen::Vector3d& p2, Eigen::Vector3d& v2);*/


	//constant detection radius
	double _implicitDistance_Func(Eigen::Vector3d& queryPnt);
	//adaptive detection radius
	double _implicitDistance_Func(Eigen::Vector3d& queryPnt, int& intersection_NUM);
	//grid plane pential field
	double _implicitDistance_Func(Eigen::Vector3d& queryPnt,	Eigen::ArrayXXd& planeSet);
	//polyline + speed up
	double _implicitDistance_Func_small_ployline(Eigen::Vector3d& queryPnt);
	//tree structure
	double _implicitDistance_Func_tree(Eigen::Vector3d& queryPnt);
	//tree structure with blend factors
	double _implicitDistance_Func_tree2(Eigen::Vector3d& queryPnt);
	// test
	double _implicitDistance_Func_tree3(Eigen::Vector3d& queryPnt);
	// record How many supportRay intersect with a sphere
	int _Intersection_NUM(Eigen::Vector3d& queryPnt); 

	void _output_SupportSurface(QMeshPatch* isoSurface, std::string path, bool isRun);
	void _smoothingIsoSurface();
	Eigen::Vector3d _calCandidateNormal(Eigen::Vector3d normal, double rad_ZrotateAngle, double rad_XtiltAngle);
	void _generate_normalSet_for_supportDetection(Eigen::Vector3d dir, std::vector<Eigen::Vector3d> dir_set);
	double _func_adaptive_ratio(int intersect_NUM);

	bool _inside_TET_Shell(Eigen::Vector3d midCoord3D);

	void _cal_Ray_direction(QMeshPatch* tet_Model);// calulate the direction of supportRay

	void _get_toolPath_list_output(std::vector<QMeshPatch*>& toolpath_list); // reorganize the toolpath list for output
	void _get_largeLayer_list(std::vector<std::vector<QMeshPatch*>>& largeLayer_list);

	// decide the intersection between neighbor initial layers(face normal / inlined face normal)
	bool _support_by_1st_below_initialLayer_normal(QMeshNode* top_node, QMeshPatch* below_initial_layer);
	bool _support_by_1st_below_initialLayer_incline(QMeshNode* top_node, QMeshPatch* below_initial_layer);

	// for waterfall-like polyline generation (one node trace continually)
	bool _find_targetNode(Eigen::Vector3d& oringinNode, Eigen::Vector3d& step_Direction, QMeshNode* Node,
		const std::vector<QMeshPatch*>& largeLayer_vector_q, bool decline_stepDir, Eigen::Vector3d& last_descend_dir);
	// for initial guess of envelope hull
	bool _find_targetNode(Eigen::Vector3d& oringinNode, Eigen::Vector3d& step_Direction, 
		QMeshNode* Node, bool decline_stepDir);
	// for tree-like support-line generation (one layer (find + merge))
	bool _find_targetNode(QMeshPatch* top_layer, const std::vector<QMeshPatch*>& bottom_large_2layer);
	bool _find_targetNode(Eigen::Vector3d oringinNode, Eigen::Vector3d step_Direction,
		QMeshNode* Node, const std::vector<QMeshPatch*>& largeLayer_vector_m, 
		Eigen::Vector3d& nearest_hostNode_coord3D, bool is_goto_center);

	bool _intersect_Line_Face(Eigen::Vector3d& oringinNode, Eigen::Vector3d& step_Direction,
		QMeshNode* Node, QMeshPatch* detect_layer, Eigen::Vector3d& last_step_Direction);
	bool _intersect_Line_Face(Eigen::Vector3d& intersetPnt, Eigen::Vector3d& oringinNode,
		Eigen::Vector3d& step_Direction, QMeshNode* Node, QMeshPatch* detect_layer);

	bool _pnt_2_layer(Eigen::Vector3d& queryPnt, QMeshPatch* detect_layer);

	void _get_step_direction_DownWard_OR_center(QMeshFace* Face, Eigen::Vector3d& treeNode_coord3D,
		Eigen::Vector3d& step_Direction,bool is_downWard);
	int _get_hostNode_on_1st_largeLayer_below(std::vector<SptTreeNode>& hostNode_1st_LB_Set,
		int host_NodeNum, const std::vector<QMeshPatch*>& largeLayer_vector_m_1);
	int _get_step_direction_tree(QMeshFace* Face, Eigen::Vector3d slave_node_coord3D, Eigen::Vector3d slave_node_Normal,
		Eigen::Vector3d& step_Direction, int dir_find_MODE, const std::vector<SptTreeNode>& hostNode_1st_LB_Set);
	int _get_index_nearest_hostNode_on_1st_largeLayer_below(
		Eigen::Vector3d slave_node_coord3D, const std::vector<SptTreeNode>& hostNode_1st_LB_Set);

	bool _get_dropNode_host_treeStructure(Eigen::Vector3d& drop_Node_host,
		Eigen::Vector3d& treeNode_coord3D, Eigen::Vector3d& step_Direction_hostNode,
		QMeshNode* sourceNode, const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
		bool is_continous_hostNode);
	bool _get_dropNode_follow_treeStructure(Eigen::Vector3d& drop_Node_host, Eigen::Vector3d& treeNode_coord3D,
		Eigen::Vector3d& step_Direction_hostNode, QMeshNode* sourceNode,
		const std::vector<QMeshPatch*>& largeLayer_vector_m_1, QMeshNode* host_sourceNode);
	void _get_step_direction_dropNode(Eigen::Vector3d& step_Direction, Eigen::Vector3d& belong_Face_Normal,
		Eigen::Vector3d& treeNode_coord3D, Eigen::Vector3d& drop_Node_host);

	//code reconstructure
	void _compute_descend_Dir_hostNode(QMeshNode* hostNode, bool is_downWard);
	QMeshNode* _build_host_treeNodeEdge(QMeshNode* hostNode,const std::vector<QMeshPatch*>& largeLayer_vector_m_1, 
		QMeshPatch* supportRay_patch, int layer_height);
	bool _compute_descend_Dir_followNode(QMeshNode* followNode, QMeshNode* dropNode_hostNode);
	void _build_follow_treeNodeEdge(QMeshNode* followNode, QMeshNode* dropNode_hostNode,
		const std::vector<QMeshPatch*>& largeLayer_vector_m_1,
		bool is_merge, QMeshPatch* supportRay_patch, int layer_height);

	//atom function of envelope hull compute
	QHULLSET* _mallocMemoryConvexHull(int faceNum, int vertNum);
	void _drawConvexHull(QHULLSET* ConvexHULL, PolygenMesh* supportRegion);
	void _freeMemoryConvexHull(QHULLSET*& pConvexHull);

	bool _checkSingleNode_inTETmodel(QMeshPatch* tet_Model, Eigen::Vector3d pp, double delta_expand);

	void _output_support_polyline(QMeshPatch* supportPoly_patch, std::string path);

	PolygenMesh* m_tetModel;
	PolygenMesh* m_layers;
	PolygenMesh* m_platform;
	PolygenMesh* m_supportRaySet;
	PolygenMesh* m_tight_supportLayerSet;
	PolygenMesh* m_toolpathSet_support;
	PolygenMesh* m_toolpathSet_initial;
	std::string  m_model_name;

	std::vector<QMeshPatch*> initial_layers;
	std::vector<QMeshPatch*> support_layers;

	// omp core number
	int CPUCoreNum = 16;
	// build_supportRay
	double bFaceExpand_ratio = 0.4;	// expand boundary face 30.0%
	//collect_SupportRays
	int needSupport_threshold = 0;	// the threshold used to indicate
									// whether certain layer needs supportRay
	//_implicitDistance_Func Parameters
	double C = 1.05;					// isoValue of potential field
	double r_i_MAX = 2.8;			// swell radius (constant/adaptive)
	double detectRadius_ratio = 1.5;// detection radius ratio		
									// detection radius = detectRadius_ratio * r_i_MAX;
	double threshold_bottom = 1.0;	// the threshold height of bottom link

	// face normal RAY
	/*  ^ r_i (energy radius)
	*3.5|___ a
	*	|	|\
	*	|	| \
	*   |	|  \
	*2.5|_ _|_ b\____
	*	|___|___|____>NUM
	*	0  100 550
	*/
	//Eigen::Vector2d a = { 100.0,3.5 };
	//Eigen::Vector2d b = { 550.0,2.5 };

	// TET surface RAY
	/*  ^ r_i (energy radius)
	*3.5|___ a
	*	|	|\
	*	|	| \
	*   |	|  \
	*2.5|_ _|_ b\____
	*	|___|___|____>NUM
	*	0   1  50
	*/
	Eigen::Vector2d a = { 2.0,6.0 };
	Eigen::Vector2d b = { 15.0,2.0 };

	// Marching Cube
	const int s = 120;				// number of vertices on the largest side
	double boxExpand_ratio = 0.2;	// expand bounding box 10%

	/******************** parameter set: ********************/
	//				bFaceExpand_ratio | needSupport_threshold |		C	 |	r_i	 |	 R	 |
	//bunnyhead				0.4					10					20		3.2		2.0
	//topo					0.4					10					20		3.2		2.0
	//yoga					0.4					20					20		2.0		3.0

	//Tet surface normal detection
	double tau = 20.0;				// support angle 25.0 bunny and topo
									// support angle 15.0 yoga
	int num = 6;					// grid plane div num
};