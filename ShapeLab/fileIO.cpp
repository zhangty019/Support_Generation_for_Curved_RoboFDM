#include "fileIO.h"
#include <fstream>
#include "alphanum.hpp"
#include "dirent.h"
#include "io.h"

void fileIO::output_OBJ_Mesh(QMeshPatch* model, std::string path) {

	double pp[3];
	std::string tetPath = path + model->patchName + ".obj";
	std::ofstream str(tetPath);

	int index = 0;
	for (GLKPOSITION posNode = model->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)model->GetNodeList().GetNext(posNode);

		node->GetCoord3D(pp[0], pp[1], pp[2]);
		str << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++;
	}
	for (GLKPOSITION posFace = model->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)model->GetFaceList().GetNext(posFace);

		str << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
	}
	str.close();

	std::cout << "\nPath: " << tetPath << std::endl;
}

int fileIO::remove_allFile_in_Dir(std::string dirPath) {

    struct _finddata_t fb;   //find the storage structure of the same properties file.
    std::string path;
    intptr_t    handle;
    int  resultone;
    int   noFile;            // the tag for the system's hidden files

    noFile = 0;
    handle = 0;

    path = dirPath + "/*";

    handle = _findfirst(path.c_str(), &fb);

    //find the first matching file
    if (handle != -1)
    {
        //find next matching file
        while (0 == _findnext(handle, &fb))
        {
            // "." and ".." are not processed
            noFile = strcmp(fb.name, "..");

            if (0 != noFile)
            {
                path.clear();
                path = dirPath + "/" + fb.name;

                //fb.attrib == 16 means folder
                if (fb.attrib == 16)
                {
                    remove_allFile_in_Dir(path);
                }
                else
                {
                    //not folder, delete it. if empty folder, using _rmdir instead.
                    remove(path.c_str());
                }
            }
        }
        // close the folder and delete it only if it is closed. For standard c, using closedir instead(findclose -> closedir).
        // when Handle is created, it should be closed at last.
        _findclose(handle);
        return 0;
    }
}

void fileIO::input_scalar_field(QMeshPatch* model, std::string path) {

	char filename[1024];
	std::sprintf(filename, "%s", path.c_str());
	std::cout << "\Scalar Field is read from:\n" << filename << std::endl;

	FILE* fp;   char linebuf[256];  int i_temp = 0;
	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file 1!\n");
		printf("===============================================\n");
		return;
	}
	//get the line number!
	while (1) { // right read CODE !!!
		fgets(linebuf, 255, fp);
		if (feof(fp)) break;
		i_temp++;
	}
	fclose(fp);
	int line_Num = i_temp;
	Eigen::VectorXd scalarFieldSet = Eigen::VectorXd::Zero(line_Num);

	fp = fopen(filename, "r");
	if (!fp) {
		printf("===============================================\n");
		printf("Can not open the data file 2!\n");
		printf("===============================================\n");
	}
	float s_value;
	i_temp = 0;
	while (1) { // right read CODE !!!
		fgets(linebuf, 255, fp);
		if (feof(fp)) break;
		sscanf(linebuf, "%f\n", &s_value);
		scalarFieldSet[i_temp] = s_value;
		i_temp++;
	}
	fclose(fp);
	//std::cout << scalarFieldSet << std::endl;

	//check
	if (scalarFieldSet.size() == model->GetNodeNumber()) {

		double index = 0;
		for (GLKPOSITION posNode = model->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)model->GetNodeList().GetNext(posNode);
			//printf("node->GetIndexNo() = %d\n", node->GetIndexNo());
			node->scalarField = scalarFieldSet[index];
			index++;
		}
	}
	else {
		printf("file line number = %d, model node number %d \n", scalarFieldSet.size(), model->GetNodeNumber());
		printf("Error: the amount of node is not equal to model, please check!\n");
		return;
	}

}

void fileIO::output_compatibleLayer_4_remesh(PolygenMesh* compatible_isoLayerSet, std::string path) {

	std::string unremeshed_isoLayer_dir = path;

	for (GLKPOSITION posMesh = compatible_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_layer = (QMeshPatch*)compatible_isoLayerSet->GetMeshList().GetNext(posMesh);

		std::string modelORsupport = "M";
		if (each_layer->is_SupportLayer) modelORsupport = "S";

		//e.g. 23_C15_M.obj
		std::string LAYER_dir = unremeshed_isoLayer_dir
			+ std::to_string(each_layer->GetIndexNo()) + "_C"
			+ std::to_string(each_layer->compatible_layer_Index) + "_"
			+ modelORsupport;
		this->_output_OneSurfaceMesh(each_layer, LAYER_dir);
	}
	std::cout << "Finish output unmeshed layers into : " << unremeshed_isoLayer_dir << std::endl;
}


void fileIO::input_remeshed_compatibleLayer(PolygenMesh* compatible_isoLayerSet, std::string path) {

	std::string remeshed_isoLayer_dir = path;
	std::vector<std::string> remeshedLayer_FileCell;// File name table
	this->_natSort(remeshed_isoLayer_dir, remeshedLayer_FileCell);

	//read slice files and build mesh_patches
	char file_Dir[1024];
	for (int i = 0; i < remeshedLayer_FileCell.size(); i++)
	{
		sprintf(file_Dir, "%s%s%s", remeshed_isoLayer_dir.c_str(), "/", remeshedLayer_FileCell[i].data());
		//std::cout << file_Dir << std::endl;

		QMeshPatch* slice = new QMeshPatch;
		slice->SetIndexNo(compatible_isoLayerSet->GetMeshList().GetCount()); //index begin from 0
		compatible_isoLayerSet->GetMeshList().AddTail(slice);
		slice->patchName = remeshedLayer_FileCell[i].data();

		// is_SupportLayer
		std::string::size_type supportFlag = remeshedLayer_FileCell[i].find("S");
		if (supportFlag == std::string::npos)	slice->is_SupportLayer = false;
		else	slice->is_SupportLayer = true;

		// get compatible layer index
		std::string::size_type p = remeshedLayer_FileCell[i].find('_');
		std::string::size_type pp = remeshedLayer_FileCell[i].find('_', p + 2);
		slice->compatible_layer_Index = stoi(remeshedLayer_FileCell[i].substr(p + 2, pp - p - 1));
		//std::cout << "remeshedLayer_FileCell[i]: " << remeshedLayer_FileCell[i] << " layer index: " << slice->GetIndexNo()
		//	<< " layer compatible index: " << slice->compatible_layer_Index << std::endl;

		slice->inputOBJFile(file_Dir);
	}
}

void fileIO::_natSort(std::string dirctory, std::vector<std::string>& fileNameCell) {

	if (fileNameCell.empty() == false) return;

	DIR* dp;
	struct dirent* ep;
	std::string fullDir = dirctory;
	//cout << fullDir << endl;
	dp = opendir(fullDir.c_str());
	//dp = opendir("../Waypoints");

	if (dp != NULL) {
		while (ep = readdir(dp)) {
			//cout << ep->d_name << endl;
			if ((std::string(ep->d_name) != ".") && (std::string(ep->d_name) != "..")) {
				//cout << ep->d_name << endl;
				fileNameCell.push_back(std::string(ep->d_name));
			}
		}
		(void)closedir(dp);
	}
	else {
		perror("Couldn't open the directory");
	}
	//resort the files with nature order
	sort(fileNameCell.begin(), fileNameCell.end(), doj::alphanum_less<std::string>());

}

//output the curved layers with splited ones OR not
void fileIO::outputIsoSurfaceSet(PolygenMesh* isoSurface, bool isSplit, std::string path, bool onORoff) {

	if (onORoff == false) return;

	std::string unremeshed_isoLayer_dir = path;

	int layer_index = 0; std::string LAYER_dir;

	for (GLKPOSITION posMesh = isoSurface->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshPatch* each_layer = (QMeshPatch*)isoSurface->GetMeshList().GetNext(posMesh);

		std::vector<QMeshPatch*>  each_layer_splited;
		bool is_multiRegion = this->_splitSingleSurfacePatch_detect(each_layer);
		if (is_multiRegion && isSplit) {
			this->_splitSingleSurfacePatch_splitmesh(each_layer, each_layer_splited);
			for (int i = 0; i < each_layer_splited.size(); i++) {

				LAYER_dir = unremeshed_isoLayer_dir + std::to_string(layer_index)
					+ "_C" + std::to_string(each_layer->compatible_layer_Index);

				if (each_layer->is_SupportLayer) LAYER_dir += "_S";
				else  LAYER_dir += "_M";
				this->_output_OneSurfaceMesh(each_layer_splited[i], (LAYER_dir + "_" + std::to_string(i)));
				layer_index++;
			}
		}
		else {
			LAYER_dir = unremeshed_isoLayer_dir + std::to_string(layer_index)
				+ "_C" + std::to_string(each_layer->compatible_layer_Index);

			if (each_layer->is_SupportLayer) LAYER_dir += "_S";
			else  LAYER_dir += "_M";

			this->_output_OneSurfaceMesh(each_layer, (LAYER_dir + "_0"));
			layer_index++;
		}
	}
	std::cout << "Finish output layers into : " << path << std::endl;
}

bool fileIO::_splitSingleSurfacePatch_detect(QMeshPatch* each_layer) {
	//Function for detecting if single path have more than one closed region - by flooding method//

	/*---------------------------
	Detect if given a closed mesh
	---------------------------*/

	bool closedSurface = true;
	for (GLKPOSITION Pos = each_layer->GetEdgeList().GetHeadPosition(); Pos;) {
		QMeshEdge* edge = (QMeshEdge*)each_layer->GetEdgeList().GetNext(Pos);
		if (edge->GetLeftFace() == NULL || edge->GetRightFace() == NULL) { closedSurface = false; break; }
	}
	if (closedSurface) return false;

	/*-------------------------------
	give node split index by flooding
	-------------------------------*/
	bool allNodeChecked = true;
	int splitTime = 0;

	for (GLKPOSITION Pos = each_layer->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)each_layer->GetNodeList().GetNext(Pos);
		thisNode->splitIndex = -1;
	}

	do {
		splitTime++;

		//-- Find start node
		for (GLKPOSITION Pos = each_layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)each_layer->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex < 0) { thisNode->splitIndex = splitTime; break; }
		}

		//-- Run flooding searching
		this->_splitSingleSurfacePatch_flooding(each_layer, splitTime);

		//-- Detect if all the node has been checked
		for (GLKPOSITION Pos = each_layer->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)each_layer->GetNodeList().GetNext(Pos);
			allNodeChecked = true;
			if (thisNode->splitIndex < 0) { allNodeChecked = false; break; }
		}

	} while (!allNodeChecked);

	if (splitTime == 1) { 
		//std::cout << "This isoSurface contains single region" << std::endl; 
		return false; }
	else {

		/*----------------------------------------------------------------------------------
		if contains more than one region, save them back to the isoSurfaceSet_splited vector
		----------------------------------------------------------------------------------*/
		each_layer->splitIndex = splitTime;
		//std::cout << "This isoSurface contains " << splitTime << " regions." << std::endl;
		return true;
	}
}

void fileIO::_splitSingleSurfacePatch_flooding(QMeshPatch* isoSurface, int index) {
	int StopFlag = 0; 			QMeshNode* NeighboorNode;

	for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);

		if (thisNode->splitIndex == index) {
			for (int i = 0; i < thisNode->GetEdgeNumber(); i++) {

				QMeshEdge* thisEdge = thisNode->GetEdgeRecordPtr(i + 1);
				if (thisNode == thisEdge->GetStartPoint()) NeighboorNode = thisEdge->GetEndPoint();
				else NeighboorNode = thisEdge->GetStartPoint();

				if (NeighboorNode->splitIndex == index) continue;
				else {
					NeighboorNode->splitIndex = index;
					StopFlag++;
				}
			}
		}
	}
	//cout << "This iteration adding node number = " << StopFlag << endl;

	if (StopFlag > 0) _splitSingleSurfacePatch_flooding(isoSurface, index);
}

void fileIO::_splitSingleSurfacePatch_splitmesh(
	QMeshPatch* isoSurface, std::vector<QMeshPatch*>& isoSurfaceSet_splited) {
	for (int iter = 1; iter < isoSurface->splitIndex + 1; iter++) {

		QMeshPatch* patch = new QMeshPatch;
		patch->splitIndex = iter; patch->SetIndexNo(isoSurface->GetIndexNo());

		//build nodeNum and nodeTable
		float* nodeTable; int nodeNum = 0; 	double pp[3];

		for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex == iter) nodeNum++;
		}
		nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);

		int index = 0;
		for (GLKPOSITION Pos = isoSurface->GetNodeList().GetHeadPosition(); Pos;) {
			QMeshNode* thisNode = (QMeshNode*)isoSurface->GetNodeList().GetNext(Pos);
			if (thisNode->splitIndex == iter) {
				thisNode->GetCoord3D(pp[0], pp[1], pp[2]);
				for (int i = 0; i < 3; i++) nodeTable[index * 3 + i] = (float)pp[i];
				thisNode->splitPatchIndex = index; //VolumetoSurfaceIndex start from 0
				index++;
			}
		}

		//build faceNum and faceTable
		unsigned int* faceTable; int faceNum = 0;
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			for (int i = 0; i < 3; i++) {
				if (face->GetNodeRecordPtr(i)->splitIndex == iter) {
					face->splitIndex = iter; faceNum++; break;
				}
			}
		}
		faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);
		index = 0;
		for (GLKPOSITION Pos = isoSurface->GetFaceList().GetHeadPosition(); Pos;) {
			QMeshFace* face = (QMeshFace*)isoSurface->GetFaceList().GetNext(Pos);
			if (face->splitIndex == iter) {
				for (int i = 0; i < 3; i++)
					faceTable[index * 3 + i] = face->GetNodeRecordPtr(i)->splitPatchIndex;
				index++;
			}
		}

		patch->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);
		isoSurfaceSet_splited.push_back(patch);
	}
}

void fileIO::_output_OneSurfaceMesh(QMeshPatch* eachLayer, std::string path) {

	double pp[3];
	path += ".obj";
	std::ofstream nodeSelection(path);

	int index = 0;
	for (GLKPOSITION posNode = eachLayer->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
		QMeshNode* node = (QMeshNode*)eachLayer->GetNodeList().GetNext(posNode);
		node->GetCoord3D(pp[0], pp[1], pp[2]);
		nodeSelection << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
		index++; node->SetIndexNo(index);

	}
	for (GLKPOSITION posFace = eachLayer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
		QMeshFace* face = (QMeshFace*)eachLayer->GetFaceList().GetNext(posFace);
		nodeSelection << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
			<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
	}
	nodeSelection.close();
}

void fileIO::input_remeshed_init_and_slimSupport_layers(PolygenMesh* m_isoLayerSet, std::string path) {

	std::string remeshed_isoLayer_dir = path;
	std::vector<std::string> remeshedLayer_FileCell;// File name table
	this->_getFileName_Set(remeshed_isoLayer_dir, remeshedLayer_FileCell);

	//read slice files and build mesh_patches
	char file_Dir[1024];
	for (int i = 0; i < remeshedLayer_FileCell.size(); i++)
	{
		sprintf(file_Dir, "%s%s%s", remeshed_isoLayer_dir.c_str(), "/", remeshedLayer_FileCell[i].data());
		//cout << file_Dir << endl;

		QMeshPatch* slice = new QMeshPatch;
		slice->SetIndexNo(m_isoLayerSet->GetMeshList().GetCount()); //index begin from 0
		m_isoLayerSet->GetMeshList().AddTail(slice);
		slice->patchName = remeshedLayer_FileCell[i].data();

		// is_SupportLayer
		std::string::size_type supportFlag = remeshedLayer_FileCell[i].find("S");
		if (supportFlag == std::string::npos)	slice->is_SupportLayer = false;
		else	slice->is_SupportLayer = true;

		// 101_C81_M_1.obj
		// get compatible layer index
		std::string::size_type p = remeshedLayer_FileCell[i].find('_');
		std::string::size_type pp = remeshedLayer_FileCell[i].find('_', p + 2);
		slice->compatible_layer_Index = stoi(remeshedLayer_FileCell[i].substr(p + 2, pp - p - 1));

		std::string::size_type q = remeshedLayer_FileCell[i].find('_', pp + 2);
		std::string::size_type qq = remeshedLayer_FileCell[i].find('.');
		slice->splitIndex = stoi(remeshedLayer_FileCell[i].substr(q + 1, qq - 1));

		//std::cout << p << " " << pp << " " << q << " " << qq << std::endl;

		/*std::cout << "remeshedLayer_FileCell[i]: " << remeshedLayer_FileCell[i]
			<< " layer index: " << slice->GetIndexNo()
			<< " layer compatible index: " << slice->compatible_layer_Index
			<< " layer split index: " << slice->splitIndex << std::endl;*/

		slice->inputOBJFile(file_Dir);
	}
}

void fileIO::_getFileName_Set(std::string dirctory, std::vector<std::string>& fileNameCell) {

	if (fileNameCell.empty() == false) return;

	DIR* dp;
	struct dirent* ep;
	std::string fullDir = dirctory;
	//cout << fullDir << endl;
	dp = opendir(fullDir.c_str());
	//dp = opendir("../Waypoints");

	if (dp != NULL) {
		while (ep = readdir(dp)) {
			//cout << ep->d_name << endl;
			if ((std::string(ep->d_name) != ".") && (std::string(ep->d_name) != "..")) {
				//cout << ep->d_name << endl;
				fileNameCell.push_back(std::string(ep->d_name));
			}
		}
		(void)closedir(dp);
	}
	else {
		perror("Couldn't open the directory");
	}
	//resort the files with nature order
	std::vector<std::string> copy_fileNameCell = fileNameCell;
	for (int i = 0; i < copy_fileNameCell.size(); i++) {

		//std::cout << copy_fileNameCell[i] << std::endl;

		int layerFile_Ind = -1;
		std::string::size_type q = copy_fileNameCell[i].find('_');
		layerFile_Ind = stoi(copy_fileNameCell[i].substr(0, q));

		fileNameCell[layerFile_Ind] = copy_fileNameCell[i];
	}
}

void fileIO::output_toolpath(PolygenMesh* toolPath, bool onORoff, bool is_Yup) {

	if (onORoff == false) return;

	std::string TOOLPATH_waypoint_dir = "../DataSet/TOOL_PATH/";
	this->remove_allFile_in_Dir(TOOLPATH_waypoint_dir);
	std::string CURVED_layer_dir = "../DataSet/CURVED_LAYER/";
	this->remove_allFile_in_Dir(CURVED_layer_dir);

	std::vector<QMeshPatch*> toolpath_list;

	std::vector<QMeshPatch*> layer_list;

	int min_CompatibleLayer_ind = 1000000; int max_CompatibleLayer_ind = -1000000;
	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		if (each_toolpath->compatible_layer_Index > max_CompatibleLayer_ind)
			max_CompatibleLayer_ind = each_toolpath->compatible_layer_Index;

		if (each_toolpath->compatible_layer_Index < min_CompatibleLayer_ind)
			min_CompatibleLayer_ind = each_toolpath->compatible_layer_Index;
	}

	for (int i = min_CompatibleLayer_ind; i <= max_CompatibleLayer_ind; i++) {

		bool find_Support;
		if (toolpath_list.size() == 0) find_Support = true;
		else find_Support = toolpath_list.back()->is_SupportLayer;

		// get split num of layer/toolpath i
		int splitNum = _getSplitNumber_with_CompatibleLayer_ind(i, toolPath, find_Support);

		for (int j = 0; j < splitNum; j++) {
			QMeshPatch* founded_toolpath = _getToolpath_with_CompatibleLayer_ind(i, toolPath, find_Support);

			if (founded_toolpath != NULL) {
				toolpath_list.push_back(founded_toolpath);
				layer_list.push_back(founded_toolpath->attached_Layer);
				//std::cout << "founded_toolpath: " << founded_toolpath->compatible_layer_Index << std::endl;
			}
		}

		// inverse find type
		find_Support = !find_Support;

		// get split num of layer/toolpath i
		splitNum = _getSplitNumber_with_CompatibleLayer_ind(i, toolPath, find_Support);

		for (int j = 0; j < splitNum; j++) {
			QMeshPatch* founded_toolpath = _getToolpath_with_CompatibleLayer_ind(i, toolPath, find_Support);

			if (founded_toolpath != NULL) {
				toolpath_list.push_back(founded_toolpath);
				layer_list.push_back(founded_toolpath->attached_Layer);
				//std::cout << "founded_toolpath: " << founded_toolpath->compatible_layer_Index << std::endl;
			}
		}
	}

	//protection code
	int toolpath_Num = toolPath->GetMeshList().GetCount();
	std::cout << "toolpath_Num: " << toolpath_Num << std::endl;
	if (toolpath_list.size() != toolpath_Num) {
		std::cout << "toolpath_list.size(): " << toolpath_list.size() << std::endl;
		std::cout << "Error: toolpath number is not same with toolpath list. please check" << std::endl;
		return;
	}

	//output waypoints
	for (int i = 0; i < toolpath_list.size(); i++) {

		QMeshPatch* each_toolpath = toolpath_list[i];

		std::string eachToolpath_dir;
		if (each_toolpath->is_SupportLayer) {
			eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + "S.txt";
		}
		else {
			eachToolpath_dir = TOOLPATH_waypoint_dir + std::to_string(i) + ".txt";
		}

		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;

		std::ofstream toolpathFile(eachToolpath_dir);

		double pp[3]; double n[3];
		QMeshEdge* sEdge = (QMeshEdge*)each_toolpath->GetEdgeList().GetHead(); // the first Edge of Toolpath
		QMeshNode* sNode = sEdge->GetStartPoint();
		sNode->GetCoord3D(pp[0], pp[1], pp[2]); sNode->GetNormal(n[0], n[1], n[2]);
		if(is_Yup)
			toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;
		else
			toolpathFile << pp[0] << " " << -pp[2] << " " << pp[1] << " " << -n[0] << " " << n[2] << " " << -n[1] << std::endl;


		for (GLKPOSITION posEdge = each_toolpath->GetEdgeList().GetHeadPosition(); posEdge != nullptr;) {
			QMeshEdge* Edge = (QMeshEdge*)each_toolpath->GetEdgeList().GetNext(posEdge);

			//std::cout << "start Node " << Edge->GetStartPoint()->GetIndexNo() << " end Node " << Edge->GetEndPoint()->GetIndexNo() << std::endl;

			QMeshNode* eNode = Edge->GetStartPoint();
			if (eNode == sNode) eNode = Edge->GetEndPoint();

			eNode->GetCoord3D(pp[0], pp[1], pp[2]); eNode->GetNormal(n[0], n[1], n[2]);
			if (is_Yup)
				toolpathFile << pp[0] << " " << pp[1] << " " << pp[2] << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;
			else
				toolpathFile << pp[0] << " " << -pp[2] << " " << pp[1] << " " << -n[0] << " " << n[2] << " " << -n[1] << std::endl;

			sNode = eNode;
		}
		toolpathFile.close();
	}
	std::cout << "Finish output toolpath into : " << TOOLPATH_waypoint_dir << std::endl;

	//output initial and slimmed support layer
	for (int i = 0; i < layer_list.size(); i++) {

		QMeshPatch* each_layer = layer_list[i];

		std::string eachLayer_dir;
		if (each_layer->is_SupportLayer) {
			eachLayer_dir = CURVED_layer_dir + std::to_string(i) + "S.obj";
		}
		else {
			eachLayer_dir = CURVED_layer_dir + std::to_string(i) + ".obj";
		}

		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;

		std::ofstream layer_str(eachLayer_dir);
		int index = 0; double pp[3];
		for (GLKPOSITION posNode = each_layer->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)each_layer->GetNodeList().GetNext(posNode);
			node->GetCoord3D(pp[0], pp[1], pp[2]);
			if (is_Yup)
				layer_str << "v " << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
			else
				layer_str << "v " << pp[0] << " " << -pp[2] << " " << pp[1] << std::endl;
			index++; node->SetIndexNo(index);
		}
		for (GLKPOSITION posFace = each_layer->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
			QMeshFace* face = (QMeshFace*)each_layer->GetFaceList().GetNext(posFace);
			layer_str << "f " << face->GetNodeRecordPtr(0)->GetIndexNo()
				<< " " << face->GetNodeRecordPtr(1)->GetIndexNo()
				<< " " << face->GetNodeRecordPtr(2)->GetIndexNo() << std::endl;
		}
		layer_str.close();
	}
	std::cout << "Finish output layers into : " << CURVED_layer_dir << std::endl;
}

void fileIO::output_toolpath_UR5e(PolygenMesh* toolPath, std::string FileDir) {

	this->remove_allFile_in_Dir(FileDir);
	std::cout << "The toolpath will be skiped when the node number is less than " 
		<< threshold_nodeNum << "\nThe index is listed below:" << std::endl;

	//output waypoints
	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		if (each_toolpath->GetNodeNumber() < threshold_nodeNum) {
			std::cout << each_toolpath->patchName << "\n";
			continue; 
		}

		std::string eachToolpath_dir = FileDir + each_toolpath->patchName;

		//std::cout << "Output File: " << TOOLPATH_dir << std::endl;

		std::ofstream toolpathFile(eachToolpath_dir);

		double pp[3];
		for (GLKPOSITION posNode = each_toolpath->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
			QMeshNode* node = (QMeshNode*)each_toolpath->GetNodeList().GetNext(posNode);

			bool jump_flag = false;
			if (node->Jump_nextSecStart || node->GetIndexNo() == 0) jump_flag = true;

			double extrusion = node->m_E;
			if (node->GetIndexNo() == 0) extrusion = 0.0;
			if (each_toolpath->GetIndexNo() > 990) extrusion *= 0.7;

			toolpathFile 
				<< node->m_printPos[0] << " " 
				<< node->m_printPos[1] << " " 
				<< node->m_printPos[2] << " "
				<< node->m_printNor[0] << " " 
				<< node->m_printNor[1] << " " 
				<< node->m_printNor[2] << " "
				<< extrusion << " " 
				<< jump_flag << " " 
				<< !each_toolpath->is_SupportLayer 
				<< std::endl;
		}

		toolpathFile.close();
	}
	std::cout << "\nFinish output toolpath into : " << FileDir << std::endl;
}

int fileIO::_getSplitNumber_with_CompatibleLayer_ind(
	int compatibleLayer_ind, PolygenMesh* toolPath, bool isSupport) {

	int splitNumber = 0;

	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		//std::cout << "compatible ind = " << each_toolpath->compatible_layer_Index
		//	<< " support? " << each_toolpath->is_SupportLayer << std::endl;

		if (each_toolpath->compatible_layer_Index == compatibleLayer_ind) {
			if (each_toolpath->is_SupportLayer == isSupport) {

				splitNumber++;
			}
		}
	}

	return splitNumber;
}

QMeshPatch* fileIO::_getToolpath_with_CompatibleLayer_ind(
	int compatibleLayer_ind, PolygenMesh* toolPath, bool find_Support) {

	QMeshPatch* founded_toolpath = NULL;

	for (GLKPOSITION Pos = toolPath->GetMeshList().GetHeadPosition(); Pos;) {
		QMeshPatch* each_toolpath = (QMeshPatch*)toolPath->GetMeshList().GetNext(Pos);

		if (each_toolpath->compatible_layer_Index == compatibleLayer_ind) {
			if (each_toolpath->is_SupportLayer == find_Support) {
				if (!each_toolpath->isInstalled_toolpath) {
					founded_toolpath = each_toolpath;
					each_toolpath->isInstalled_toolpath = true;
					break;
				}
			}
		}
	}

	return founded_toolpath;
}

int fileIO::read_layer_toolpath_files(
	PolygenMesh* Slices, PolygenMesh* Waypoints, std::string Dir) {

	std::string PosNorFileDir = Dir + "/TOOL_PATH";
	std::string LayerFileDir = Dir + "/layer_Simplified";

	this->_natSort(PosNorFileDir, wayPointFileCell);
	this->_natSort(LayerFileDir, sliceSetFileCell);

	if (wayPointFileCell.size() != sliceSetFileCell.size()) {
		std::cout << "The file number of slics and toolpath is not the same, please check." << std::endl;
		return 0;
	}

	this->_readWayPointData(Waypoints, PosNorFileDir);
	this->_readSliceData(Slices, LayerFileDir);

	return wayPointFileCell.size();
}

void fileIO::_readWayPointData(PolygenMesh* Waypoints, std::string packName) {

	//read waypoint files and build mesh_patches
	char filename[1024];
	for (int i = 0; i < wayPointFileCell.size(); i++) {

		sprintf(filename, "%s%s%s", packName.c_str(), "/", wayPointFileCell[i].data());

		QMeshPatch* waypoint = new QMeshPatch;
		waypoint->SetIndexNo(Waypoints->GetMeshList().GetCount()); //index begin from 0
		Waypoints->GetMeshList().AddTail(waypoint);
		waypoint->patchName = wayPointFileCell[i].data();

		// isSupportLayer
		std::string::size_type supportFlag = wayPointFileCell[i].find("S");
		if (supportFlag == std::string::npos)	waypoint->is_SupportLayer = false;
		else	waypoint->is_SupportLayer = true;

		waypoint->inputPosNorFile(filename);
	}
	std::cout << "--> WayPoints Load Finish!" << std::endl;
}

void fileIO::_readSliceData(PolygenMesh* Slices, std::string packName) {

	//read slice files and build mesh_patches
	char filename[1024];
	for (int i = 0; i < sliceSetFileCell.size(); i++) {

		sprintf(filename, "%s%s%s", packName.c_str(), "/", sliceSetFileCell[i].data());

		QMeshPatch* layers = new QMeshPatch;
		layers->SetIndexNo(Slices->GetMeshList().GetCount()); //index begin from 0
		Slices->GetMeshList().AddTail(layers);
		layers->patchName = sliceSetFileCell[i].data();

		// isSupportLayer
		std::string::size_type supportFlag = sliceSetFileCell[i].find("S");
		if (supportFlag == std::string::npos)	layers->is_SupportLayer = false;
		else	layers->is_SupportLayer = true;

		layers->inputOBJFile(filename);
	}
	std::cout << "--> Slices Load Finish!" << std::endl;
}
