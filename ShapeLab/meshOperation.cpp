#include "meshOperation.h"
#include "tetgen.h"

#include <iostream>
#include <fstream>      // std::ifstream
#include <cstring>

void meshOperation::tetMeshGeneration_extract_SupportSpace(
    QMeshPatch* box, QMeshPatch* tetPatch) {

    //0. extract the boundary of tet model(*.obj)
    QMeshPatch* boundary_tetPatch = new QMeshPatch;
    _extract_Surface_from_TetMesh(tetPatch, boundary_tetPatch);
    //1. combine the boundary of tet model(*.obj) and box mesh(*.obj)
    QMeshPatch* combinedMesh = new QMeshPatch;
    this->_combineTwoSurfaceMesh(box, boundary_tetPatch, combinedMesh);
    //2. generate the tet mesh of boxEnvelope containing boundary of tet mesh
    QMeshPatch* solidTetMesh = new QMeshPatch;
    this->_tetMeshGeneration(combinedMesh, solidTetMesh, "Ya4.0");
    //3. extract support space by Boolean operation
    this->_tetMeshGeneration_hollowed(solidTetMesh, boundary_tetPatch);
    //4. output support space (hollowed)
    std::string hollowed_supportTet_dir = "../DataSet/TET_MODEL/" + tetPatch->patchName + "_supportSpace";
    this->_hollowed_Tet_Support_Space_Output(solidTetMesh, hollowed_supportTet_dir);

    boundary_tetPatch->ClearAll(); delete boundary_tetPatch;
    combinedMesh->ClearAll(); delete combinedMesh;
    solidTetMesh->ClearAll(); delete solidTetMesh;
}

////////////////////////////////////////////////////////////////////////////////

void meshOperation::_extract_Surface_from_TetMesh(QMeshPatch* tetPatch, QMeshPatch* boundary_tetPatch) {

    //clean the index of boundary node of tetMesh
    //get node number
    int index = 0;
    for (GLKPOSITION posNode = tetPatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
        QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(posNode);

        if (node->inner) continue;
        node->Volume2Surface_Ind = index; //start from 0
        index++;
    }
    
    int nodeNum = index;
    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    //build node table
    index = 0; double pp[3];
    for (GLKPOSITION posNode = tetPatch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
        QMeshNode* node = (QMeshNode*)tetPatch->GetNodeList().GetNext(posNode);

        if (node->inner) continue;
        node->GetCoord3D(pp[0], pp[1], pp[2]);
        for (int i = 0; i < 3; i++) nodeTable[3 * index + i] = pp[i];
        index++;
    }
    //get face number
    index = 0;
    for (GLKPOSITION posFace = tetPatch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
        QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(posFace);
        if (face->inner) continue;
        index++;
    }

    int faceNum = index;
    unsigned int* faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);
    //build face table
    index = 0;
    for (GLKPOSITION posFace = tetPatch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
        QMeshFace* face = (QMeshFace*)tetPatch->GetFaceList().GetNext(posFace);
        if (face->inner) continue;
        for (int i = 0; i < 3; i++)
            faceTable[index * 3 + i] = face->GetNodeRecordPtr(i)->Volume2Surface_Ind;
        index++;
    }

    boundary_tetPatch->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

    delete nodeTable;
    delete faceTable;
}

void meshOperation::_combineTwoSurfaceMesh(
    QMeshPatch* box, QMeshPatch* boundary_tetPatch, QMeshPatch* combinedMesh) {

    int nodeNum = box->GetNodeNumber() + boundary_tetPatch->GetNodeNumber();
    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);

    int faceNum = box->GetFaceNumber() + boundary_tetPatch->GetFaceNumber();
    unsigned int* faceTable = (unsigned int*)malloc(sizeof(unsigned int) * faceNum * 3);

    /* build node list */
    int nodeIndex = 0;  double pp[3];
    for (GLKPOSITION Pos = box->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)box->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex); //start from 0
        Node->GetCoord3D(pp[0], pp[1], pp[2]);
        for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
        nodeIndex++;
    }

    for (GLKPOSITION Pos = boundary_tetPatch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)boundary_tetPatch->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex);
        Node->GetCoord3D(pp[0], pp[1], pp[2]);
        for (int i = 0; i < 3; i++) nodeTable[3 * nodeIndex + i] = pp[i];
        nodeIndex++;
    }

    /* build face list */
    int faceIndex = 0;
    for (GLKPOSITION Pos = box->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)box->GetFaceList().GetNext(Pos);
        for (int i = 0; i < 3; i++)
            faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->GetIndexNo();
        faceIndex++;
    }

    for (GLKPOSITION Pos = boundary_tetPatch->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)boundary_tetPatch->GetFaceList().GetNext(Pos);
        for (int i = 0; i < 3; i++)
            faceTable[faceIndex * 3 + i] = thisFace->GetNodeRecordPtr(i)->GetIndexNo();
        faceIndex++;
    }

    combinedMesh->constructionFromVerFaceTable(nodeNum, nodeTable, faceNum, faceTable);

}

void meshOperation::_tetMeshGeneration(
    QMeshPatch* inputMesh, QMeshPatch* outputMesh, std::string tetgenCommand) {

    tetgenio in, out;
    tetgenio::facet* f;
    tetgenio::polygon* p;

    // All indices start from 0.
    //in.firstnumber = 1;

    /* fill node list - tetgen */
    in.numberofpoints = inputMesh->GetNodeNumber();
    in.pointlist = new REAL[in.numberofpoints * 3];

    int nodeIndex = 0;
    for (GLKPOSITION Pos = inputMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)inputMesh->GetNodeList().GetNext(Pos);
        Node->SetIndexNo(nodeIndex);
        Node->GetCoord3D(in.pointlist[3 * nodeIndex], in.pointlist[3 * nodeIndex + 1], in.pointlist[3 * nodeIndex + 2]);
        nodeIndex++;
    }

    /* fill face list - tetgen */
    in.numberoffacets = inputMesh->GetFaceNumber();
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];

    int faceIndex = 0;
    for (GLKPOSITION Pos = inputMesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* thisFace = (QMeshFace*)inputMesh->GetFaceList().GetNext(Pos);

        f = &in.facetlist[faceIndex];
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = NULL;

        p = &f->polygonlist[0];
        p->numberofvertices = 3;
        p->vertexlist = new int[p->numberofvertices];
        p->vertexlist[0] = thisFace->GetNodeRecordPtr(0)->GetIndexNo();
        p->vertexlist[1] = thisFace->GetNodeRecordPtr(1)->GetIndexNo();
        p->vertexlist[2] = thisFace->GetNodeRecordPtr(2)->GetIndexNo();

        faceIndex++;
    }

    const char* inputCommand = tetgenCommand.c_str();

    tetrahedralize((char*)inputCommand, &in, &out);

    int nodeNum = out.numberofpoints;
    float* nodeTable = (float*)malloc(sizeof(float) * nodeNum * 3);
    for (int i = 0; i < nodeNum * 3; i++) nodeTable[i] = out.pointlist[i];

    int tetraNum = out.numberoftetrahedra;
    unsigned int* tetTable = (unsigned int*)malloc(sizeof(unsigned int) * tetraNum * 4);
    for (int i = 0; i < tetraNum * 4; i++) tetTable[i] = out.tetrahedronlist[i];

    std::string path = "../DataSet/temp/tetOperation/newTet.tet";
    std::ofstream tetOutput(path);
    tetOutput << nodeNum << " vertices" << std::endl;
    tetOutput << tetraNum << " tets" << std::endl;
    for (int i = 0; i < nodeNum; i++) {
        tetOutput << nodeTable[i * 3] << " " << nodeTable[i * 3 + 1] << " " << nodeTable[i * 3 + 2] << std::endl;
    }
    for (int i = 0; i < tetraNum; i++) {
        tetOutput << "4 " << tetTable[i * 4] << " " << tetTable[i * 4 + 1] << " " << tetTable[i * 4 + 2] << " " << tetTable[i * 4 + 3] << std::endl;
    }
    tetOutput.close();

    delete nodeTable, tetTable;
    std::cout << "output newTet.tet into " << path << std::endl;
    const char* pathCharStar = path.c_str();
    outputMesh->inputTETFile((char*)pathCharStar, true);

}

void meshOperation::_tetMeshGeneration_hollowed(QMeshPatch* solidTetMesh, QMeshPatch* boundary_tetPatch) {

    //_build_tetraSet_4SpeedUp
    std::vector<QMeshTetra*> tetraSet_supportSpace(solidTetMesh->GetTetraNumber());
    int index = 0;
    for (GLKPOSITION Pos = solidTetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)solidTetMesh->GetTetraList().GetNext(Pos);
        tetraSet_supportSpace[index] = Tetra; index++;
    }
    // protect operation
    if (solidTetMesh->GetTetraNumber() != index)
        std::cout << "Error: please check the tet num of tet mesh(supportSpace)!" << std::endl;

    //detect the center of tet element of supportTET in/out the model surface
#pragma omp parallel   
    {
#pragma omp for 
        for (int i = 0; i < tetraSet_supportSpace.size(); i++) {
            QMeshTetra* each_Tetra = tetraSet_supportSpace[i];

            Eigen::Vector3d centerPos = Eigen::Vector3d::Zero();
            each_Tetra->CalCenterPos(centerPos[0], centerPos[1], centerPos[2]);
            each_Tetra->isIn_tetSurfaceMesh = this->_calculatePointInsideMesh(boundary_tetPatch, centerPos);
        }
    }

    //clear the is_tetSupportNode flag
    for (GLKPOSITION Pos = solidTetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)solidTetMesh->GetNodeList().GetNext(Pos);

        Node->is_tetSupportNode = false;
    }
    //mark the is_tetSupportNode
    for (GLKPOSITION Pos = solidTetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)solidTetMesh->GetTetraList().GetNext(Pos);

        if (Tetra->isIn_tetSurfaceMesh) continue;

        for (int nodeInd = 0; nodeInd < 4; nodeInd++) {
            Tetra->GetNodeRecordPtr(nodeInd + 1)->is_tetSupportNode = true;
        }
    }

    std::cout << "\nFinish boundary detection." << std::endl;
}

void meshOperation::_hollowed_Tet_Support_Space_Output(QMeshPatch* solidTetMesh, std::string path) {

    path += ".tet";
    // export files - hollowed tet Model file
    std::ofstream str(path);

    // record tet number
    int hollow_supportTet_NUM = 0;
    for (GLKPOSITION Pos = solidTetMesh->GetTetraList().GetHeadPosition(); Pos;) {
        QMeshTetra* Tetra = (QMeshTetra*)solidTetMesh->GetTetraList().GetNext(Pos);

        if (!Tetra->isIn_tetSurfaceMesh) hollow_supportTet_NUM++;
    }

    // record node number
    int hollow_supportNode_Ind = 0;
    for (GLKPOSITION Pos = solidTetMesh->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)solidTetMesh->GetNodeList().GetNext(Pos);

        if (Node->is_tetSupportNode) {

            Node->hollow_supportTet_Ind = hollow_supportNode_Ind;
            hollow_supportNode_Ind++;
        }
    }
    int hollow_supportNode_NUM = hollow_supportNode_Ind;

    str << hollow_supportNode_NUM << " vertices" << std::endl;
    str << hollow_supportTet_NUM << " tets" << std::endl;


    for (GLKPOSITION posNode = solidTetMesh->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
        QMeshNode* node = (QMeshNode*)solidTetMesh->GetNodeList().GetNext(posNode);

        if (!node->is_tetSupportNode) continue;

        double pp[3]; node->GetCoord3D(pp[0], pp[1], pp[2]);
        str << pp[0] << " " << pp[1] << " " << pp[2] << std::endl;
    }
    for (GLKPOSITION posTet = solidTetMesh->GetTetraList().GetHeadPosition(); posTet != nullptr;) {
        QMeshTetra* tet = (QMeshTetra*)solidTetMesh->GetTetraList().GetNext(posTet);

        if (tet->isIn_tetSurfaceMesh) continue;

        str << "4 " << tet->GetNodeRecordPtr(1)->hollow_supportTet_Ind
            << " " << tet->GetNodeRecordPtr(2)->hollow_supportTet_Ind
            << " " << tet->GetNodeRecordPtr(3)->hollow_supportTet_Ind
            << " " << tet->GetNodeRecordPtr(4)->hollow_supportTet_Ind << std::endl;
    }
    str.close();
    std::cout << "Output hollowed tet model for support generation." << std::endl;
}



bool meshOperation::_calculatePointInsideMesh(QMeshPatch* target_mesh, Eigen::Vector3d& orig) {

    Eigen::Vector3d dir = { 1.0,0.0,0.0 };
    int intersection_Time = 0;

    for (GLKPOSITION Pos = target_mesh->GetFaceList().GetHeadPosition(); Pos;) {
        QMeshFace* each_face = (QMeshFace*)target_mesh->GetFaceList().GetNext(Pos);

        double xx, yy, zz;
        each_face->GetNodeRecordPtr(0)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v0 = { xx,yy,zz };

        each_face->GetNodeRecordPtr(1)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v1 = { xx,yy,zz };

        each_face->GetNodeRecordPtr(2)->GetCoord3D(xx, yy, zz);
        Eigen::Vector3d v2 = { xx,yy,zz };


        if (this->_IntersectTriangle(orig, dir, v0, v1, v2))
            intersection_Time++;

    }
    //std::cout << "intersection Num " << intersection_Time << std::endl;
    if (intersection_Time % 2 != 0) {
        //std::cout << "in the mesh" << std::endl;
        return true;
    }
    else return false;
    //std::cout << "be out of mesh" << std::endl;
}

// Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection

bool meshOperation::_IntersectTriangle(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
    Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2)
{
    // E1
    Eigen::Vector3d E1 = v1 - v0;

    // E2
    Eigen::Vector3d E2 = v2 - v0;

    // P
    Eigen::Vector3d P = dir.cross(E2);

    // determinant
    float det = E1.dot(P);

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
    if (det < 0.0001f)
        return false;

    // Calculate u and make sure u <= 1
    double t, u, v;
    u = T.dot(P);
    if (u < 0.0f || u > det)
        return false;

    // Q
    Eigen::Vector3d Q = T.cross(E1);

    // Calculate v and make sure u + v <= 1
    v = dir.dot(Q);
    if (v < 0.0f || u + v > det)
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    t = E2.dot(Q);
    if (t < 0) return false;

    float fInvDet = 1.0f / det;
    t *= fInvDet;
    u *= fInvDet;
    v *= fInvDet;

    return true;
}
