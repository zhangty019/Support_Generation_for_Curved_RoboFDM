#pragma once

#include "PolygenMesh.h"

class meshOperation
{

public:
	meshOperation() {};
	~meshOperation() {};

public:

	void meshOperation::tetMeshGeneration_extract_SupportSpace(
		QMeshPatch* box, QMeshPatch* tetPatch);

private:

	void _extract_Surface_from_TetMesh(
		QMeshPatch* tetPatch, QMeshPatch* boundary_tetPatch);
	void _combineTwoSurfaceMesh(
		QMeshPatch* box, QMeshPatch* boundary_tetPatch, QMeshPatch* combinedMesh);
	void _tetMeshGeneration(
		QMeshPatch* inputMesh, QMeshPatch* outputMesh, std::string tetgenCommand);
	void _tetMeshGeneration_hollowed(
		QMeshPatch* solidTetMesh, QMeshPatch* boundary_tetPatch);
	void _hollowed_Tet_Support_Space_Output(
		QMeshPatch* solidTetMesh, std::string path);

private:

	bool _IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
		Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2);

	bool _calculatePointInsideMesh(QMeshPatch* target_mesh, Eigen::Vector3d& orig);

};

