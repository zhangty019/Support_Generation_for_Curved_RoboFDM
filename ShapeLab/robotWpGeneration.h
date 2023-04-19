#pragma once
#include "../QMeshLib/PolygenMesh.h"

class robotWpGeneration {

public:
	robotWpGeneration() {};
	~robotWpGeneration() {};

	void initial(PolygenMesh* Slices, PolygenMesh* Waypoints, 
		PolygenMesh* CncPart, double width);
	void calDHW();
	void writeWp_4_robot(std::string GcodeDir);
	void singularityOpt();
	void modify_wp_normal(bool is_run);
	void build_Gcode_table(PolygenMesh* toolpathSet,
		Eigen::MatrixXf& Gcode_Table, int sIdx, int eIdx);

private:

	void _cal_Dist();
	void _initialSmooth(int loopTime);
	void _cal_Height();
	void _calDHW2E(QMeshPatch* patch);
	void _moveUp_4_addhision(double up_dist);

	std::vector<QMeshPatch*> _getJumpSection_patchSet(QMeshPatch* patch);
	double _safe_acos(double value);
	void _markSingularNode(QMeshPatch* patch);
	void _filterSingleSingularNode(QMeshPatch* patch);
	void _getSingularSec(QMeshPatch* patch, Eigen::MatrixXd& sectionTable);
	void _projectAnchorPoint(QMeshPatch* patch);
	void _getBCtable2(QMeshPatch* patch, Eigen::MatrixXd& B1C1table, Eigen::MatrixXd& B2C2table);
	void _motionPlanning3(
		QMeshPatch* patch, const Eigen::MatrixXd& sectionTable, const Eigen::MatrixXd& B1C1table,
		const Eigen::MatrixXd& B2C2table, Eigen::RowVector2d& prevBC);
	bool _chooseB1C1(
		const Eigen::RowVector2d& B1C1, const Eigen::RowVector2d& B2C2,
		Eigen::RowVector2d& prevBC);
	double _toLeft(
		const Eigen::RowVector2d& origin_p, const Eigen::RowVector2d& startPnt_q,
		const Eigen::RowVector2d& endPnt_s);
	void _getXYZ(QMeshPatch* patch);
	void _optimizationC(QMeshPatch* patch);
	void _verifyPosNor();
	double _getAngle3D(const Eigen::Vector3d& v1,
		const Eigen::Vector3d& v2, const bool in_degree);


	PolygenMesh* m_Slices = NULL;
	PolygenMesh* m_Waypoints = NULL;
	PolygenMesh* m_CncPart = NULL;

	double m_width = 0.0;

	//inner parameters
	int m_jump_detection_threshold = 5.0;
	int Core = 16;
	int layerNum = 40;	 // The number of detected bottom layer for height calculation
	bool is_planar_printing = false; //switch for the planar printing
	double ratio = 0.58;  // extrusion ratio
	double m_lambdaValue = 6.5;

	//post process
	double up_alpha = 0.2;
	Eigen::Vector3d up_dir = { 0.0,0.0,1.0 };

	//singularity opt
	Eigen::Vector3d abb_offset = { 0.0,0.0,900.0 };
};