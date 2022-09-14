// PMBody.cpp: implementation of the PMBody class.
//
//////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_DEPRECATE

#include <math.h>
#include <memory.h>

#include "PolygenMesh.h"

#include <QDebug>

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

PolygenMesh::PolygenMesh(mesh_type type)
{
    ClearAll();
    m_drawListID=-1;
    m_bVertexNormalShading=false;
    isTransparent = false;
    m_drawListNumber = 6;
    meshType = type;
}

PolygenMesh::~PolygenMesh()
{
    ClearAll();
    if (m_drawListID!=-1) glDeleteLists(m_drawListID, m_drawListNumber);
}

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

void PolygenMesh::CompBoundingBox(double boundingBox[])
{
    GLKPOSITION PosMesh;
    GLKPOSITION Pos;
    double xx,yy,zz;

    boundingBox[0]=boundingBox[2]=boundingBox[4]=1.0e+32;
    boundingBox[1]=boundingBox[3]=boundingBox[5]=-1.0e+32;

    for(PosMesh=meshList.GetHeadPosition();PosMesh!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(PosMesh));
        for(Pos=mesh->GetNodeList().GetHeadPosition();Pos!=NULL;) {
            QMeshNode *node=(QMeshNode *)(mesh->GetNodeList().GetNext(Pos));
            node->GetCoord3D(xx,yy,zz);

            if (xx<boundingBox[0]) boundingBox[0]=xx;
            if (xx>boundingBox[1]) boundingBox[1]=xx;
            if (yy<boundingBox[2]) boundingBox[2]=yy;
            if (yy>boundingBox[3]) boundingBox[3]=yy;
            if (zz<boundingBox[4]) boundingBox[4]=zz;
            if (zz>boundingBox[5]) boundingBox[5]=zz;
        }
    }
}

void PolygenMesh::DeleteGLList()
{
    if (m_drawListID!=-1) {
        glDeleteLists(m_drawListID, m_drawListNumber);
        m_drawListID=-1;
    }
}

void PolygenMesh::BuildGLList(bool bVertexNormalShading)
{
    if (m_drawListID!=-1) glDeleteLists(m_drawListID, m_drawListNumber);
    m_drawListID = glGenLists(m_drawListNumber);

    _buildDrawShadeList(bVertexNormalShading);
    _buildDrawMeshList();
    _buildDrawNodeList();
    _buildDrawProfileList();
    _buildDrawFaceNormalList();
    _buildDrawNodeNormalList();
    computeRange();
}

void PolygenMesh::_buildDrawShadeList(bool bVertexNormalShading)
{
    GLKPOSITION Pos;
    GLKPOSITION PosFace;
    GLKPOSITION PosNode;
    QMeshFace* face;
    QMeshNode* node;
    QMeshPatch* mesh;
    double xx, yy, zz, dd;		float rr, gg, bb;
    int k, i, num, meshIndex;
    glNewList(m_drawListID, GL_COMPILE);

    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);

	//drawOriginalCoordinate();
    if (isTransparent) {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    /*------------- Draw FACE (CAD_PARTS) -------------*/
    if (this->meshType == CAD_PARTS) {
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            float rr, gg, bb;
            glBegin(GL_TRIANGLES);


            for (GLKPOSITION PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
                QMeshFace* face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

                rr = gg = bb = 0.8f;
                if (mesh->patchName == "floor") { rr = 0.05; gg = 0.05; bb = 0.05; }
                glColor3f(rr, gg, bb);
                this->drawSingleFace(face);
            }
            glEnd();
        }
    }

    /*------------- Draw FACE (TET) -------------*/
    if (this->meshType == TET) {
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));
            float rr, gg, bb;

            glBegin(GL_TRIANGLES);
            for (GLKPOSITION PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
                QMeshFace* face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

                // the inner face is not visulize 
                if (face->inner == true) continue;

                rr = 0.8; gg = 0.8; bb = 0.8;

                if (face->needSupport) { rr = 0.8; gg = 0.0; bb = 0.0; }
                glColor3f(rr, gg, bb);
                this->drawSingleFace(face);

            }
            glEnd();
        }
    }

    /*------------- Draw FACE (CURVED_LAYER) -------------*/
    if (this->meshType == CURVED_LAYER) {
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            if (mesh->drawThisPatch == false) continue;

            float rr, gg, bb;
            glBegin(GL_TRIANGLES);


            for (GLKPOSITION PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
                QMeshFace* face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

                _changeValueToColor(mesh->GetIndexNo(), rr, gg, bb);
                if (mesh->is_SupportLayer && this->getModelName() == "Layers") { rr = gg = bb = 0.8; }
                if (this->getModelName() == "Tight_supportLayerSet") { rr = gg = bb = 0.9; }

                if (face->support_treeNode_cell.size() != 0)
                { 
                    _changeValueToColor(10.0, 0.0, (double)face->support_treeNode_cell.size(), rr, gg, bb);
                }
                

                glColor3f(rr, gg, bb);
                this->drawSingleFace(face);
            }
            glEnd();
        }
    }

    if (this->meshType == SUPPORT_RAY) {
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            if (mesh->drawThisPatch == false) continue;

            float rr, gg, bb;
            glBegin(GL_TRIANGLES);


            for (GLKPOSITION PosFace = (mesh->GetFaceList()).GetHeadPosition(); PosFace != NULL;) {
                QMeshFace* face = (QMeshFace*)((mesh->GetFaceList()).GetNext(PosFace));

                rr = 0.85; gg = 0.35; bb = 0.65;

                glColor3f(rr, gg, bb);
                this->drawSingleFace(face);
            }
            glEnd();
        }
    }

    if (isTransparent) {
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }

    glEndList();
}

void PolygenMesh::_changeValueToColor(int nType, float & nRed, float & nGreen, float & nBlue)
{
    float color[][3]={
        {220,20,60},
        {107,200,35},
        {30,144,255},
        {255,105,180},
        {244,164,96},
        {176,196,222},
        {255,100,70},
        {128,255,128},
        {128,128,255},
        {255,255,128},
        {0,128,0},
        {255,128,255},
        {255,214,202},
        {128,128,192},
        {255,165,0}, //orange
        {255,128,192},
//		{39, 64, 139},//RoyalBlue
        {128,128,64},
        {0,255,255},
        {238,130,238},//violet
        {220,220,220},//gainsboro
        {188, 143, 143}, // rosy brown
        {46, 139, 87},//sea green
        {210, 105, 30 },//chocolate
        {237, 150, 100},
        {100, 149, 237},//cornflower blue
        {243, 20, 100},
        // 26th
        {0,0,0}
    };

//	printf("%d ",nType);
    nRed=color[nType%25][0]/255.0f;
    nGreen=color[nType%25][1]/255.0f;
    nBlue=color[nType%25][2]/255.0f;
}

void PolygenMesh::_buildDrawMeshList()
{
  
    GLKPOSITION Pos;
    GLKPOSITION PosEdge;
    float rr, gg, bb;

    if (meshList.GetCount()==0) return;

    glNewList(m_drawListID+1, GL_COMPILE);
    glDisable(GL_LIGHTING);
    
    glLineWidth(2.0);
    QMeshEdge* edge;
    QMeshPatch* mesh;
    double xx, yy, zz;

    /*------------- Draw EDGE (CAD_PARTS) -------------*/
    if (this->meshType == CAD_PARTS) {
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            if (mesh->patchName == "floor") continue;

            glBegin(GL_LINES);
            glLineWidth(1.0);
            rr = 0.3; gg = 0.3; bb = 0.3;

            for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
                QMeshEdge* edge = (QMeshEdge*)((mesh->GetEdgeList()).GetNext(PosEdge));

                // only show the boundary eadge
                Eigen::Vector3d normal_Lface, normal_Rface;
                edge->GetLeftFace()->GetNormal(normal_Lface(0), normal_Lface(1), normal_Lface(2));
                normal_Lface = normal_Lface.normalized();
                edge->GetRightFace()->GetNormal(normal_Rface(0), normal_Rface(1), normal_Rface(2));
                normal_Rface = normal_Rface.normalized();

                double radCOS = normal_Lface.dot(normal_Rface);
                double angle = ROTATE_TO_DEGREE(acos(radCOS));

                if (angle > 45.0) {
                    rr = 0.1; gg = 0.1; bb = 0.1;
                    glColor3f(rr, gg, bb);
                    this->drawSingleEdge(edge);
                }
            }
            glEnd();
        }
    }
    /*------------- Draw EDGE (TET) -------------*/
    if (this->meshType == TET && false) {
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            glBegin(GL_LINES);
            glLineWidth(2.0);
            rr = 0.75; gg = 0.75; bb = 0.75;

            for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
                QMeshEdge* edge = (QMeshEdge*)((mesh->GetEdgeList()).GetNext(PosEdge));

                if (edge->inner) continue;

                glColor3f(rr, gg, bb);
                

                this->drawSingleEdge(edge);
            }
            glEnd();
        }
    }

    /*------------- Draw EDGE (CURVED_LAYER)-------------*/
    if (this->meshType == CURVED_LAYER) {
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            if (mesh->drawThisPatch == false) continue;

            glBegin(GL_LINES);
            glLineWidth(1.0);

            for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
                QMeshEdge* edge = (QMeshEdge*)((mesh->GetEdgeList()).GetNext(PosEdge));
                rr = 0.75; gg = 0.75; bb = 0.75;

                glColor3f(rr, gg, bb);
                this->drawSingleEdge(edge);
            }
            glEnd();
        }
    }

    /*------------- Draw EDGE (SUPPORT_RAY)-------------*/
    if (this->meshType == SUPPORT_RAY) {
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            //std::cout << "aa" << std::endl;
            if (mesh->drawThisPatch == false) continue;
            //std::cout << "bb" << std::endl;
            glLineWidth(1.5);
            glBegin(GL_LINES);

            for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
                QMeshEdge* edge = (QMeshEdge*)((mesh->GetEdgeList()).GetNext(PosEdge));

                //_changeValueToColor(log10(mesh->max_segment_NUM), -0.001, log10(edge->treeEdge_radius), rr, gg, bb);
                //_changeValueToColor(mesh->max_height, -0.001, edge->treeEdge_height, rr, gg, bb);
                _changeValueToColor(sqrt((double)mesh->max_branch_NUM), 0.99, sqrt((double)edge->treeEdge_branch_NUM), rr, gg, bb);


                if (!edge->GetEndPoint()->isUseful_Node_SupportRay
                    || !edge->GetStartPoint()->isUseful_Node_SupportRay) continue;

                glColor3f(rr, gg, bb);
                this->drawSingleEdge(edge);
            }
            glEnd();
        }
    }

    /*------------- Draw SURFACE MESH (TOOL_PATH)-------------*/
    if (this->meshType == TOOL_PATH) {
        glLineWidth(2.0);
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            if (mesh->drawThisPatch == false) continue;

            glBegin(GL_LINES);

            for (GLKPOSITION PosEdge = (mesh->GetEdgeList()).GetHeadPosition(); PosEdge != NULL;) {
                QMeshEdge* edge = (QMeshEdge*)((mesh->GetEdgeList()).GetNext(PosEdge));
                rr = 0.3; gg = 0.3; bb = 0.3;

                _changeValueToColor(mesh->GetIndexNo(), rr, gg, bb);
                // show only before the subsumple of waypoints
                if (edge->isConnectEdge == true) { rr = 1.0; gg = 0.0; bb = 0.0; }

                glColor3f(rr, gg, bb);
                this->drawSingleEdge(edge);
            }
            glEnd();
        }
    }

    glEndList();
}

void PolygenMesh::_buildDrawNodeList()
{
    if (meshList.GetCount() == 0) return;

    glNewList(m_drawListID + 2, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    float rr, gg, bb;

    /*------------- Draw Node (CURVED_LAYER)-------------*/
    if (this->meshType == CURVED_LAYER) {
        glPointSize(3.0);
        glBegin(GL_POINTS);
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            if (mesh->drawThisPatch == false) continue;


            for (GLKPOSITION PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
                QMeshNode* node = (QMeshNode*)((mesh->GetNodeList()).GetNext(PosNode));

                rr = 0.65; gg = 0.65; bb = 0.65;
                if (node->need_Support) { rr = 0.15; gg = 0.95; bb = 0.15; }
                if (mesh->is_SupportLayer && mesh->drawSupportField) {

                    if (node->implicitSurface_value >= 0) {
                        rr = 1.0; gg = bb = 0.0;
                    }
                    else {
                        rr = gg = bb = 0.0;
                    }
                }

                if(node->GetAttribFlag(0)) { rr = gg = 0.0; bb = 1.0; }

                //Debug use
                if (node->isHighlight) { rr = 1.0; gg = 0.0; bb = 0.0; }
                if(node->polyline_node.size()!=0) { rr = 0.0; gg = 1.0; bb = 1.0; }
                //_changeValueToColor(1.0, 0.0, node->boundaryValue, rr, gg, bb);

                glColor3f(rr, gg, bb);
                drawSingleNode(node);
            }
        }
        glEnd();
    }

    /*------------- Draw Node (TET)-------------*/
    if (this->meshType == TET) {
        glPointSize(4.0);
        glBegin(GL_POINTS);
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));
            for (GLKPOSITION PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
                QMeshNode* node = (QMeshNode*)((mesh->GetNodeList()).GetNext(PosNode));

                rr = 0.85; gg = 0.85; bb = 0.85;

                if (node->inner) continue;
                if (node->need_Support) { rr = 0.0; gg = 1.0; bb = 0.0; }

                if(node->isHighlight) { rr = 0.0; gg = 0.0; bb = 1.0; }
                
                glColor3f(rr, gg, bb);

                drawSingleNode(node);
            }
        }
        glEnd();
    }

    /*------------- Draw Node (UNDEFINED) -------------*/
    if (this->meshType == UNDEFINED || this->meshType == CAD_PARTS) {
        glPointSize(1.0);
        glBegin(GL_POINTS);
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));
            for (GLKPOSITION PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
                QMeshNode* node = (QMeshNode*)((mesh->GetNodeList()).GetNext(PosNode));

                rr = 0.25; gg = 0.25; bb = 0.25;
                glColor3f(rr, gg, bb);

                drawSingleNode(node);
            }
        }
        glEnd();
    }

    /*------------- Draw Node (SUPPORT_RAY) -------------*/
    if (this->meshType == SUPPORT_RAY) {
        glPointSize(2.0);
        glBegin(GL_POINTS);
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            if (mesh->drawThisPatch == false) continue;

            for (GLKPOSITION PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
                QMeshNode* node = (QMeshNode*)((mesh->GetNodeList()).GetNext(PosNode));

                if (!node->isUseful_Node_SupportRay) continue;

                //if (!node->isHighlight) continue;

                rr = 0.05; gg = 0.05; bb = 0.95;
                if (node->isHighlight) { rr = 1.0; gg = 0.0; bb = 0.0; }

                //for tree-like structure debug
                /*if (node->is_Host) { rr = 1.0; gg = 0.0; bb = 0.0; }
                if (node->isOringin) { rr = 0.15; gg = 0.95; bb = 0.15; }
                if (node->is_virtual_Host) { rr = 0.0; gg = 1.0; bb = 0.0; }
                else if (!node->is_Processed) { rr = 0.0; gg = 0.0; bb = 0.0; }*/
                glColor3f(rr, gg, bb);

                drawSingleNode(node);
            }
        }
        glEnd();
    }

    /*------------- Draw Node (TOOL_PATH) -------------*/
    if (this->meshType == TOOL_PATH) {
        glPointSize(5.0);
        glBegin(GL_POINTS);
        for (GLKPOSITION Pos = meshList.GetHeadPosition(); Pos != NULL; ) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(Pos));

            if (mesh->drawThisPatch == false) continue;

            for (GLKPOSITION PosNode = (mesh->GetNodeList()).GetHeadPosition(); PosNode != NULL;) {
                QMeshNode* node = (QMeshNode*)((mesh->GetNodeList()).GetNext(PosNode));

                if (node->resampleChecked == false) continue;

                _changeValueToColor(mesh->GetIndexNo() + 1, rr, gg, bb);

                if (node->GetIndexNo() == 0) { rr = 1.0f; gg = 0.0f; bb = 0.0f; }

                glColor3f(rr, gg, bb);
                drawSingleNode(node);
            }
        }
        glEnd();
    }

    glEndList();
}

void PolygenMesh::_buildDrawProfileList()
{
    if (meshList.GetCount() == 0) return;

    glNewList(m_drawListID + 3, GL_COMPILE);
    glLineWidth(1.5);
    //glDisable(GL_LIGHTING);

    if (this->meshType == TET) {
        for (GLKPOSITION PosMesh = meshList.GetHeadPosition(); PosMesh != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(PosMesh));

            double edgeLength = 0;
            for (GLKPOSITION Pos = mesh->GetEdgeList().GetHeadPosition(); Pos != NULL;) {
                QMeshEdge* Edge = (QMeshEdge*)(mesh->GetEdgeList().GetNext(Pos));
                edgeLength += Edge->CalLength();
            }
            edgeLength /= mesh->GetEdgeNumber(); // average edge length    

            for (GLKPOSITION Pos = mesh->GetTetraList().GetHeadPosition(); Pos;) {
                QMeshTetra* Tet = (QMeshTetra*)mesh->GetTetraList().GetNext(Pos);

                if (Tet->GetIndexNo() % 20 != 0) continue;

                glLineWidth(0.5);

                double x, y, z;
                double length = edgeLength; // give the average edge length for vector length
                float  rr, gg, bb;
                double n[3];

                Tet->CalCenterPos(x, y, z);
                for (int i = 0; i < 3; i++) n[i] = Tet->vectorField(i);

                double n_length = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                if (n_length < 0.001) {
                    //std::cout << "error Vector Field value, Index: " << Tet->GetIndexNo() << std::endl;
                    continue; // not draw the short field vectors
                }

                rr = 0.31f, gg = 0.58f, bb = 0.81f;// default color
                length *= 1.5;

                //---- Draw Array (BODY) ----//
                glColor3f(rr, gg, bb);
                glBegin(GL_LINES);
                glVertex3d(x, y, z); glVertex3d(x + n[0] * length, y + n[1] * length, z + n[2] * length);
                glEnd();

                //---- Draw Array (TIP) ----//
                double tipColor[3] = { rr,gg,bb };
                double endPoint[3] = { x + n[0] * length, y + n[1] * length, z + n[2] * length };
                drawSingleArrayTip(endPoint, n, length, tipColor);
            }
        }
    }

    glEndList();
}

void PolygenMesh::_buildDrawFaceNormalList()
{
    if (meshList.GetCount()==0) return;
    if (this->meshType == CAD_PARTS) return;
    if (this->meshType == SUPPORT_RAY) return;
    if (this->meshType == TOOL_PATH) return;

    glNewList(m_drawListID+4, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glColor3f(0.5, 0.0, 0.5);

    glLineWidth(1.0);

    if (this->meshType == TET) {

        glBegin(GL_LINES);
        for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));
            QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetHead();
            double length = edge->CalLength();
            //std::cout << "Face normal length is: " << length << std::endl;
            for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos != NULL;) {
                QMeshFace* face = (QMeshFace*)(mesh->GetFaceList().GetNext(Pos));

                // only show the surface normals
                if (face->GetLeftTetra() != NULL && face->GetRightTetra() != NULL) continue;

                double x, y, z, nx, ny, nz;
                face->CalCenterPos(x, y, z);
                face->CalPlaneEquation();
                face->GetNormal(nx, ny, nz);
                glVertex3d(x, y, z);
                glVertex3d(x + nx * length, y + ny * length, z + nz * length);
            }
        }
        glEnd();

    }
    else if (this->meshType == CURVED_LAYER) {
        glBegin(GL_LINES);
        for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));

            if (mesh->drawThisPatch == false) continue;

            double length = 1.0;

            for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos != NULL;) {
                QMeshFace* face = (QMeshFace*)(mesh->GetFaceList().GetNext(Pos));
               /* double x, y, z;
                face->CalCenterPos(x, y, z);
                glVertex3d(x, y, z);
                glVertex3d(x + face->m_desiredNormal[0] * length, y + face->m_desiredNormal[1] * length, z + face->m_desiredNormal[2] * length);
                */
                double x, y, z, nx, ny, nz;
                face->CalCenterPos(x, y, z);
                face->CalPlaneEquation();
                face->GetNormal(nx, ny, nz);
                glVertex3d(x, y, z);
                glVertex3d(x + nx * length, y + ny * length, z + nz * length);
            }
        }
        glEnd();
    }
    else {
        glBegin(GL_LINES);
        for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));

            QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetHead();
            double length = edge->CalLength();
            for (GLKPOSITION Pos = mesh->GetFaceList().GetHeadPosition(); Pos != NULL;) {
                QMeshFace* face = (QMeshFace*)(mesh->GetFaceList().GetNext(Pos));
                double x, y, z, nx, ny, nz;
                face->CalCenterPos(x, y, z);
                face->CalPlaneEquation();
                face->GetNormal(nx, ny, nz);
                glVertex3d(x, y, z);
                glVertex3d(x + nx * length, y + ny * length, z + nz * length);
            }
        }
        glEnd();
    }

    glEndList();
}

void PolygenMesh::_buildDrawNodeNormalList()
{
    if (meshList.GetCount()==0) return;
    if (this->meshType == CAD_PARTS) return;
    if (this->meshType == SUPPORT_RAY) return;

    glNewList(m_drawListID + 5, GL_COMPILE);
    glDisable(GL_LIGHTING);

    glColor3f(0.0, 0.5, 0.0);

    glLineWidth(1.0);

    if (this->meshType == CURVED_LAYER) {

        //if (this->getModelName() != "Layers") return;

        glBegin(GL_LINES);
        for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));

            if (mesh->drawThisPatch == false) continue;

            double length = 1.0;
            for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos != NULL;) {
                QMeshNode* node = (QMeshNode*)(mesh->GetNodeList().GetNext(Pos));
                /*double x, y, z;
                node->GetCoord3D(x, y, z);
                glVertex3d(x, y, z);
                glVertex3d(x + node->m_desiredNormal[0] * length, y + node->m_desiredNormal[1] * length, z + node->m_desiredNormal[2] * length);
                */
                double x, y, z, n[3];
                node->GetCoord3D(x, y, z);
                node->CalNormal(n);
                node->GetNormal(n[0], n[1], n[2]);

                glVertex3d(x, y, z);
                glVertex3d(x + n[0] * length, y + n[1] * length, z + n[2] * length);
            }
        }
        glEnd();
    }
    else if (this->meshType == TOOL_PATH) {

        glBegin(GL_LINES);
        for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));

            if (mesh->drawThisPatch == false) continue;

            double length = 1.0;
            for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos != NULL;) {
                QMeshNode* node = (QMeshNode*)(mesh->GetNodeList().GetNext(Pos));
                double x, y, z, n[3];
                node->GetCoord3D(x, y, z);
                node->GetNormal(n[0], n[1], n[2]);
                glVertex3d(x, y, z);
                glVertex3d(x + n[0] * length, y + n[1] * length, z + n[2] * length);
            }
        }
        glEnd();

    }
    else{
        glBegin(GL_LINES);
        for (GLKPOSITION meshPos = meshList.GetHeadPosition(); meshPos != NULL;) {
            QMeshPatch* mesh = (QMeshPatch*)(meshList.GetNext(meshPos));

            QMeshEdge* edge = (QMeshEdge*)mesh->GetEdgeList().GetHead();
            double length = edge->CalLength();
            //double length = 1.0;
            for (GLKPOSITION Pos = mesh->GetNodeList().GetHeadPosition(); Pos != NULL;) {
                QMeshNode* node = (QMeshNode*)(mesh->GetNodeList().GetNext(Pos));
                double x, y, z;
                node->GetCoord3D(x, y, z);
                double n[3];
                node->CalNormal(n);

                glVertex3d(x, y, z);
                glVertex3d(x + n[0] * length, y + n[1] * length, z + n[2] * length);
            }
        }
        glEnd();
    }
    
    glEndList();
}

void PolygenMesh::drawOriginalCoordinate() {
	double axisLeng = 100.0;
	glLineWidth(5.0);
	glBegin(GL_LINES);

	// X-axis - Red Color
	glColor3f(1.0, 0.0, 0.0); glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(axisLeng, 0.0, 0.0);

	// Y-axis - green Color
	glColor3f(0.0, 1.0, 0.0);
	glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, axisLeng, 0.0);

	// Z-axis - black Color
	glColor3f(0.0, 0.0, 0.0);
	glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 0.0, axisLeng);

	//glColor3f(1, 0.1, 0.1);glVertex3d(0.0, 0.0, 0.0);
	//glVertex3d(-7.72805,- 11.4536,- 3.48036); //voxel searching debug

	glEnd();

	
}

void PolygenMesh::drawShade()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID);
}

void PolygenMesh::drawMesh()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+1);
}

void PolygenMesh::drawNode()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+2);
}

void PolygenMesh::drawProfile()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+3);
}

void PolygenMesh::drawFaceNormal()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+4);
}

void PolygenMesh::drawNodeNormal()
{
    if (meshList.IsEmpty()) {glDeleteLists(m_drawListID, m_drawListNumber); m_drawListID=-1; return;}
    glCallList(m_drawListID+5);
}

void PolygenMesh::ClearAll()
{
    GLKPOSITION Pos;

    for(Pos=meshList.GetHeadPosition();Pos!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(Pos));
        delete mesh;
    }
    meshList.RemoveAll();
}

void PolygenMesh::computeRange()
{
    double range=0.0,ll,xx,yy,zz;
    GLKPOSITION Pos;
    GLKPOSITION PosNode;

    for(Pos=meshList.GetHeadPosition();Pos!=NULL;) {
        QMeshPatch *mesh=(QMeshPatch *)(meshList.GetNext(Pos));
        for(PosNode=(mesh->GetNodeList()).GetHeadPosition();PosNode!=NULL;) {
            QMeshNode *node=(QMeshNode *)((mesh->GetNodeList()).GetNext(PosNode));

            node->GetCoord3D(xx,yy,zz);
            ll=xx*xx+yy*yy+zz*zz;

            if (ll>range) range=ll;
        }
    }

    m_range=(float)(sqrt(range));
}

void PolygenMesh::_changeValueToColor(double maxValue, double minValue, double Value,
                                 float & nRed, float & nGreen, float & nBlue)
{
//	Value=fabs(Value);

    if (Value<=minValue)
    {
        nRed=0.0;
        nGreen=0.0;
        nBlue=0.0;
        return;
    }

    if ((maxValue-minValue)<0.000000000001)
    {
        nRed=0.0;
        nGreen=0.0;
        nBlue=1.0;
        return;
    }

    double temp=(Value-minValue)/(maxValue-minValue);

//    nRed=(float)(1.0-temp);	nGreen=(float)(1.0-temp); nBlue=(float)(1.0-temp);	return;

    if (temp>0.75)
    {
        nRed=1;
        nGreen=(float)(1.0-(temp-0.75)/0.25);
        if (nGreen<0) nGreen=0.0f;
        nBlue=0;
        return;
    }
    if (temp>0.5)
    {
        nRed=(float)((temp-0.5)/0.25);
        nGreen=1;
        nBlue=0;
        return;
    }
    if (temp>0.25)
    {
        nRed=0;
        nGreen=1;
        nBlue=(float)(1.0-(temp-0.25)/0.25);
        return;
    }
    else
    {
        nRed=0;
        nGreen=(float)(temp/0.25);
        nBlue=1;
    }

//    double t1,t2,t3;
//    t1=0.75;
//    t2=0.5;
//    t3=0.25;
//    if (temp>t1)
//    {
//        nRed=1;
//        nGreen=0.8-(float)(temp-t1)/(1-t1)*0.42;
//        if (nGreen<0.38) nGreen=0.38f;
//        nBlue=0.62-(float)(temp-t1)/(1-t1)*0.4;
//        if (nBlue<0.22) nBlue=0.22;
//        return;
//    }
//    if (temp>t2)
//    {
//        nRed=1;
//        nGreen=1.0-(float)(temp-t2)/(t1-t2)*0.2;
//        if (nGreen<0.8) nGreen=0.8f;
//        nBlue=0.75-(float)(temp-t2)/(t1-t2)*0.13;
//        if (nBlue<0.62) nBlue=0.62f;
//        return;
//    }
//    if (temp>t3)
//    {
//        nRed=(float)(temp-t3)/(t2-t3)*0.31+0.69;
//        if (nRed>1.0) nRed=1.0f;
//        nGreen=(float)(temp-t3)/(t2-t3)*0.09+0.91;
//        if (nGreen>1.0) nGreen=1.0f;
//        nBlue=0.95-(float)(temp-t3)/(t2-t3)*0.2;
//        if (nBlue<0.75) nBlue=0.75f;
//        return;
//    }
//    else
//    {
//        nRed=(float)temp/t3*0.47+0.22;
//        if (nRed>0.69) nRed=0.69f;
//        nGreen=(float)temp/t3*0.53+0.38;
//        if (nGreen>0.91) nGreen=0.91f;
//        nBlue=1.0-(float)temp/t3*0.05;
//        if (nBlue<0.95) nBlue=0.95f;
//        return;
//    }
}

void PolygenMesh::ImportOBJFile(char *filename, std::string modelName)
{
    QMeshPatch *newMesh = new QMeshPatch;
    if (newMesh->inputOBJFile(filename)){
        meshList.AddTail(newMesh);
        computeRange();
        setModelName(modelName);
    }
    else
        delete newMesh;
}

void PolygenMesh::ImportTETFile(char *filename, std::string modelName)
{
	QMeshPatch *newMesh = new QMeshPatch;
	if (newMesh->inputTETFile(filename, false)) {
		meshList.AddTail(newMesh);
		computeRange();
		setModelName(modelName);
	}
	else
		delete newMesh;
}

void PolygenMesh::drawSingleEdge(QMeshEdge* edge) {

    double xx, yy, zz;
    edge->GetStartPoint()->GetCoord3D(xx, yy, zz);
    glVertex3d(xx, yy, zz);
    edge->GetEndPoint()->GetCoord3D(xx, yy, zz);
    glVertex3d(xx, yy, zz);

}

void PolygenMesh::drawSingleNode(QMeshNode* node) {

    double nx, ny, nz, xx, yy, zz;
    node->GetNormal(nx, ny, nz);
    node->GetCoord3D(xx, yy, zz);
    glNormal3d(nx, ny, nz);
    glVertex3d(xx, yy, zz);

}

void PolygenMesh::drawSingleFace(QMeshFace* face) {

    double xx, yy, zz, dd;
    for (int i = 0; i < 3; i++) {
        QMeshNode* node = face->GetNodeRecordPtr(i);

        if (m_bVertexNormalShading) {
            double normal[3];
            node->CalNormal(normal); glNormal3dv(normal);
        }
        else {
            face->CalPlaneEquation();
            face->GetPlaneEquation(xx, yy, zz, dd);
            glNormal3d(xx, yy, zz);
        }

        node->GetCoord3D(xx, yy, zz);
        glVertex3d(xx, yy, zz);
    }
}

void PolygenMesh::drawSingleArrayTip(double pp[3], double dir[3], double arrowLength, double tipColor[3]) {

    double bone_hight = 0.3 * arrowLength;
    double bone_radius = 0.06 * arrowLength;

    Eigen::Vector3d endPP = { pp[0],pp[1],pp[2] };

    Eigen::Vector3d A = { dir[0],dir[1],dir[2] };
    Eigen::Vector3d B = { 0, 1.0, 0 };

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = Eigen::Quaterniond().setFromTwoVectors(B, A);

    Eigen::Vector3d pp1 = { bone_radius * sin(0) , 0.0 , bone_radius * cos(0) };
    Eigen::Vector3d pp2 = { bone_radius * sin(120 * 3.14 / 180) , 0.0 , bone_radius * cos(120 * 3.14 / 180) };
    Eigen::Vector3d pp3 = { bone_radius * sin(240 * 3.14 / 180) , 0.0 , bone_radius * cos(240 * 3.14 / 180) };
    Eigen::Vector3d ppCenter = { 0.0, bone_hight, 0.0 };

    pp1 = rotationMatrix * pp1 + endPP;
    pp2 = rotationMatrix * pp2 + endPP;
    pp3 = rotationMatrix * pp3 + endPP;
    ppCenter = rotationMatrix * ppCenter + endPP;

    glBegin(GL_TRIANGLES);

    glColor3f(tipColor[0], tipColor[1], tipColor[2]);

    glVertex3d(pp1(0), pp1(1), pp1(2));
    glVertex3d(pp2(0), pp2(1), pp2(2));
    glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

    glVertex3d(pp2(0), pp2(1), pp2(2));
    glVertex3d(pp3(0), pp3(1), pp3(2));
    glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

    glVertex3d(pp3(0), pp3(1), pp3(2));
    glVertex3d(pp1(0), pp1(1), pp1(2));
    glVertex3d(ppCenter(0), ppCenter(1), ppCenter(2));

    glEnd();
}