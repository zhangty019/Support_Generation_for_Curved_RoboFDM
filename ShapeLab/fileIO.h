#pragma once
#include "../QMeshLib/PolygenMesh.h"

class fileIO
{
public:
	fileIO() {}
	~fileIO() {};

	void output_OBJ_Mesh(QMeshPatch* model, std::string path);
	int remove_allFile_in_Dir(std::string dirPath);
	void input_scalar_field(QMeshPatch* model, std::string path);
	void output_compatibleLayer_4_remesh(PolygenMesh* compatible_isoLayerSet, std::string path);
	void input_remeshed_compatibleLayer(PolygenMesh* compatible_isoLayerSet, std::string path);
	void outputIsoSurfaceSet(PolygenMesh* isoSurface, bool isSplit, std::string path, bool onORoff);
	void input_remeshed_init_and_slimSupport_layers(PolygenMesh* m_isoLayerSet, std::string path);
	void output_toolpath(PolygenMesh* toolPath, bool onORoff, bool is_Yup);
	void output_toolpath_UR5e(PolygenMesh* toolPath, std::string FileDir);
	int read_layer_toolpath_files(
		PolygenMesh* Slices, PolygenMesh* Waypoints, std::string Dir);

private:

	void _natSort(std::string dirctory, std::vector<std::string>& fileNameCell);
	bool _splitSingleSurfacePatch_detect(QMeshPatch* each_layer);
	void _splitSingleSurfacePatch_flooding(QMeshPatch* isoSurface, int index);
	void _splitSingleSurfacePatch_splitmesh(
		QMeshPatch* isoSurface, std::vector<QMeshPatch*>& isoSurfaceSet_splited);
	void _output_OneSurfaceMesh(QMeshPatch* eachLayer, std::string path);
	void _getFileName_Set(std::string dirctory, std::vector<std::string>& fileNameCell);
	int _getSplitNumber_with_CompatibleLayer_ind(
		int compatibleLayer_ind, PolygenMesh* toolPath, bool isSupport);
	QMeshPatch* _getToolpath_with_CompatibleLayer_ind(
		int compatibleLayer_ind, PolygenMesh* toolPath, bool find_Support);

	void _readWayPointData(PolygenMesh* Waypoints, std::string packName);
	void fileIO::_readSliceData(PolygenMesh* Slices, std::string packName);


	std::vector<std::string> wayPointFileCell;// Waypoints Dir Files
	std::vector<std::string> sliceSetFileCell;// Layers Dir Files
	int threshold_nodeNum = 0;
};
