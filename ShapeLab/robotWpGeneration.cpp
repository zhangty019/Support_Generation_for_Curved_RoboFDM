#include "robotWpGeneration.h"
#include "../ThirdPartyDependence/PQPLib/PQP.h"
#include "GLKGeometry.h"

void robotWpGeneration::initial(
    PolygenMesh* Slices, PolygenMesh* Waypoints, PolygenMesh* CncPart, double width) {

    m_Slices = Slices;
    m_Waypoints = Waypoints;
    m_CncPart = CncPart;
    m_width = width; // toolpath width

    this->_moveUp_4_addhision(0.3);
}

void robotWpGeneration::calDHW() {

    this->_cal_Dist();
    this->_initialSmooth(10);
    this->_cal_Height();

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* layer = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        this->_calDHW2E(layer);
    }
}

//rotate the normal to up a bit (temp)
void robotWpGeneration::modify_wp_normal(bool is_run) {

    if (!is_run) return;

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* layer = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

            Eigen::Vector3d original_normal = Node->m_printNor;

            Eigen::Vector3d new_normal = (1.0 - up_alpha) * original_normal + up_alpha * up_dir;
            new_normal.normalize();

            Node->m_printNor = new_normal;
            Node->SetNormal(new_normal[0], new_normal[1], new_normal[2]);
        }
    }
    std::cout << "\n\n--> Rotate the normal towards Z dir.\n\n" << std::endl;
}

void robotWpGeneration::_moveUp_4_addhision(double up_dist) {

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* wp_patch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        double xx, yy, zz;

        for (GLKPOSITION Pos = wp_patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* node = (QMeshNode*)wp_patch->GetNodeList().GetNext(Pos);

            node->GetCoord3D(xx, yy, zz);
            node->SetCoord3D(xx, yy, zz + up_dist);
            node->m_printPos << xx, yy, zz + up_dist;
        }
    }

    for (GLKPOSITION Pos = m_Slices->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* layer = (QMeshPatch*)m_Slices->GetMeshList().GetNext(Pos);

        double xx, yy, zz;

        for (GLKPOSITION Pos = layer->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* node = (QMeshNode*)layer->GetNodeList().GetNext(Pos);

            node->GetCoord3D(xx, yy, zz);
            node->SetCoord3D(xx, yy, zz + up_dist);
        }
    }

    std::cout << " --> Move wp and layers up " << up_dist << "mm." << std::endl;
}

void robotWpGeneration::_cal_Dist() {

    //std::cout << "------------------------------------------- Waypoint Distance Calculation Running ..." << std::endl;
    //long time = clock();

    std::cout << "--> largeJ_Length: " << m_jump_detection_threshold << std::endl;

#pragma omp parallel
    {
#pragma omp for  
        for (int omptime = 0; omptime < Core; omptime++) {

            for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
                QMeshPatch* layer = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

                if (layer->GetIndexNo() % Core != omptime) continue;

                for (GLKPOSITION nodePos = layer->GetNodeList().GetHeadPosition(); nodePos;) {
                    QMeshNode* Node = (QMeshNode*)layer->GetNodeList().GetNext(nodePos);

                    double D = 0.0;
                    int lines = layer->GetNodeNumber();
                    if (Node->GetIndexNo() == (lines - 1)) { D = 0.0; }
                    else {

                        GLKPOSITION nextPos = layer->GetNodeList().Find(Node)->next;
                        QMeshNode* nextNode = (QMeshNode*)layer->GetNodeList().GetAt(nextPos);

                        D = (Node->m_printPos - nextNode->m_printPos).norm();

                        if (D > m_jump_detection_threshold) {
                            D = 0.0;								// inject the D to the Node/startPnt of Edge
                            Node->Jump_preSecEnd = true;			// end of prev section
                            nextNode->Jump_nextSecStart = true;		// start of next section
                        }
                    }
                    Node->m_DHW(0) = D;
                }
            }
        }
    }
    //printf("TIMER -- Distance Calculation takes %ld ms.\n", clock() - time);
    std::cout << "--> Waypoint Distance Calculation Finish!" << std::endl;
}

void robotWpGeneration::_initialSmooth(int loopTime) {

    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        int patch_NodeNum = WayPointPatch->GetNodeNumber();
        std::vector<bool> fix_Flag(patch_NodeNum);
        std::vector<Eigen::Vector3d> NodeNormal_temp(patch_NodeNum); // [Nx Ny Nz fix_flag]

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            /*fixed at first / end / jump_start / jump_end points*/
            if (Node->GetIndexNo() == 0 || Node->GetIndexNo() == patch_NodeNum - 1
                || Node->Jump_preSecEnd || Node->Jump_nextSecStart) {
                fix_Flag[Node->GetIndexNo()] = true;
            }
            else { fix_Flag[Node->GetIndexNo()] = false; }

            NodeNormal_temp[Node->GetIndexNo()] = Node->m_printNor;

        }

        //smooth normal by (n-1) + X*(n) + (n+1)
        for (int loop = 0; loop < loopTime; loop++) {
            for (int i = 0; i < fix_Flag.size(); i++) {

                if (fix_Flag[i] == false) {
                    NodeNormal_temp[i] = (NodeNormal_temp[i - 1] + 0.5 * NodeNormal_temp[i] + NodeNormal_temp[i + 1]).normalized();
                }

            }
        }

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            Node->m_printNor = NodeNormal_temp[Node->GetIndexNo()];
            Node->SetNormal(Node->m_printNor(0), Node->m_printNor(1), Node->m_printNor(2));
        }
    }
    std::cout << "--> Initial Smooth Finish!" << std::endl;
}

void robotWpGeneration::_cal_Height() {

    //std::cout << "------------------------------------------- Waypoint Height Calculation Running ..." << std::endl;
    //long time = clock();

    // get the patch polygenMesh_PrintPlatform
    QMeshPatch* patch_PrintPlatform = NULL;
    for (GLKPOSITION posMesh = m_CncPart->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* thisPatch = (QMeshPatch*)m_CncPart->GetMeshList().GetNext(posMesh);

        if (thisPatch->patchName == "patch_platform") {
            patch_PrintPlatform = thisPatch;
            break;
        }
    }

    if (patch_PrintPlatform == NULL) {
        std::cout << "patch_PrintPlatform is NULL, please check." << std::endl;
        return;
    }

#pragma omp parallel
    {
#pragma omp for  
        for (int omptime = 0; omptime < Core; omptime++) {

            // topLayer --> layer on the highest place [travel head to tail]
            for (GLKPOSITION Pos = m_Slices->GetMeshList().GetHeadPosition(); Pos;) {
                QMeshPatch* topLayer = (QMeshPatch*)m_Slices->GetMeshList().GetNext(Pos); // order: get data -> pnt move

                if (topLayer->GetIndexNo() % Core != omptime) continue;

                std::vector<QMeshPatch*> bottomLayers;

                bottomLayers.push_back(patch_PrintPlatform);
                // construct a bottomLayers[i] to store the point of bottom layers for every toplayer
                for (GLKPOSITION beforePos = m_Slices->GetMeshList().Find(topLayer)->prev; beforePos;) {
                    QMeshPatch* beforePatch = (QMeshPatch*)m_Slices->GetMeshList().GetPrev(beforePos);

                    bottomLayers.push_back(beforePatch);
                    if (bottomLayers.size() > layerNum) break;
                }


                //--build PQP model
                std::vector<PQP_Model*> bLayerPQP;
                bLayerPQP.resize(bottomLayers.size());
                for (int i = 0; i < bottomLayers.size(); i++) {
                    if (bottomLayers[i]->GetNodeNumber() < 3) continue;
                    // build PQP model for bottom layers
                    PQP_Model* pqpModel = new PQP_Model();
                    pqpModel->BeginModel();  int index = 0;
                    PQP_REAL p1[3], p2[3], p3[3];

                    for (GLKPOSITION Pos = bottomLayers[i]->GetFaceList().GetHeadPosition(); Pos;) {
                        QMeshFace* Face = (QMeshFace*)bottomLayers[i]->GetFaceList().GetNext(Pos);

                        Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
                        Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
                        Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

                        pqpModel->AddTri(p1, p2, p3, index);
                        index++;

                    }
                    pqpModel->EndModel();
                    bLayerPQP[i] = pqpModel;
                }//--build PQP model END

                int layerIndex = topLayer->GetIndexNo();

                GLKPOSITION WayPointPatch_Pos = m_Waypoints->GetMeshList().FindIndex(layerIndex);
                QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetAt(WayPointPatch_Pos);

                for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
                    QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

                    double minHeight = 99999.99;
                    for (int i = 0; i < bottomLayers.size(); i++) {
                        if (bottomLayers[i]->GetNodeNumber() < 3) continue;

                        PQP_DistanceResult dres; dres.last_tri = bLayerPQP[i]->last_tri;
                        PQP_REAL p[3];
                        p[0] = Node->m_printPos(0); p[1] = Node->m_printPos(1); p[2] = Node->m_printPos(2);
                        PQP_Distance(&dres, bLayerPQP[i], p, 0.0, 0.0);

                        double Height = dres.Distance(); // Height of this layer
                        //int minTriIndex = dres.last_tri->id;	// closest triangle
                        if (minHeight > Height) minHeight = Height;
                    }
                    //cout << minHeight << endl;
                    Node->m_DHW(1) = minHeight;
                }

                //	free memory
                for (int i = 0; i < bottomLayers.size(); i++) { delete bLayerPQP[i]; }
            }
        }
    }

    //std::printf("TIMER -- Height Calculation takes %ld ms.\n", clock() - time);
    std::cout << "--> Waypoint Height Calculation Finish!" << std::endl;
}

void robotWpGeneration::_calDHW2E(QMeshPatch* patch) {

    // E = E + ratio * height * length * width;
    double D, H, W;

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        D = Node->m_DHW(0);
     
        H = Node->m_DHW(1);
   
        Node->m_DHW(2) = m_width;
        W = Node->m_DHW(2);

        Node->m_E = ratio * H * D * W;
    }

    double E = 0.0;
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
        E = E + Node->m_E;
        Node->m_E = E;
    }
}

void robotWpGeneration::singularityOpt() {

    std::cout << "------------------------------------------- XYZBCE Calculation running ... " << std::endl;
    long time = clock();

#pragma omp parallel
    {
#pragma omp for  
        for (int omptime = 0; omptime < Core; omptime++) {

            for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
                QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

                if (WayPointPatch->GetIndexNo() % Core != omptime) continue;

                std::vector<QMeshPatch*> layerJumpPatchSet = _getJumpSection_patchSet(WayPointPatch);

                Eigen::RowVector2d prevBC = { 0.0,0.0 };
                // give the message of BC for the first Node (only one)
                for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
                    QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

                    // solve 1
                    prevBC(0) = ROTATE_TO_DEGREE(-_safe_acos(Node->m_printNor(2)));
                    prevBC(1) = ROTATE_TO_DEGREE(-atan2(Node->m_printNor(1), Node->m_printNor(0)));

                    // solve 2
                    double C2temp = prevBC(1) + 180.0;
                    if (C2temp > 180.0)	C2temp -= 360.0; // control the range of C2 into the (-180,180]

                    //// prevBC always BC1 in each sub_patch
                    //if (fabs(C2temp) < fabs(prevBC(1))) {
                    //    prevBC(0) = -prevBC(0);
                    //    prevBC(1) = C2temp;
                    //}

                    break;
                }
                //cout << "prevBC: " << prevBC << endl;

                for (int Index = 0; Index < layerJumpPatchSet.size(); Index++) {
                    //1.0 find the singularity waypoints
                    _markSingularNode(layerJumpPatchSet[Index]);
                    //1.1 filter single singular waypoint (XXOXX -> XXXXX)
                    _filterSingleSingularNode(layerJumpPatchSet[Index]);

                    Eigen::MatrixXd sectionTable, B1C1table, B2C2table;
                    //2.0 get the range of singularity Sections
                    _getSingularSec(layerJumpPatchSet[Index], sectionTable);
                    //2.1 project normal to the singular region boundary and check
                    _projectAnchorPoint(layerJumpPatchSet[Index]);

                    //3. calculate the 2 solves baced on kinematics of CNC
                    _getBCtable2(layerJumpPatchSet[Index], B1C1table, B2C2table);
                    //4. Main singularity optimization algorithm
                    _motionPlanning3(layerJumpPatchSet[Index], sectionTable, B1C1table, B2C2table, prevBC);
                    //5. reset steps: CNC XYZ calculation
                    _getXYZ(layerJumpPatchSet[Index]);
                }

                //aim to eliminate the -pi to pi sharp change
                _optimizationC(WayPointPatch);
            }
        }
    }

    std::cout << "-------------------------------------------" << std::endl;
    std::printf("TIMER -- XYZBCE Calculation takes %ld ms.\n", clock() - time);
    std::cout << "------------------------------------------- XYZBCE Calculation Finish!\n " << std::endl;

    this->_verifyPosNor();
}

std::vector<QMeshPatch*> robotWpGeneration::_getJumpSection_patchSet(QMeshPatch* patch) { 

    // Get the Jump section Num
    int JumpPatchNum = 1;
    for (GLKPOSITION Pos_Node = patch->GetNodeList().GetHeadPosition(); Pos_Node;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos_Node);

        if (Node->Jump_nextSecStart == true) JumpPatchNum++;
    }

    // molloc the space for each jumpPatch
    std::vector<QMeshPatch*> layerJumpPatchSet(JumpPatchNum);
    for (int i = 0; i < JumpPatchNum; i++) {
        layerJumpPatchSet[i] = new QMeshPatch();
        layerJumpPatchSet[i]->rootPatch_jumpPatch = patch;
    }

    // Add node into each JumpPatch
    int Jump_PatchIndex = 0;
    int JumpPatch_NodeIndex = 0;
    for (GLKPOSITION Pos_Node = patch->GetNodeList().GetHeadPosition(); Pos_Node;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos_Node);

        if (Node->Jump_nextSecStart == true) {
            Jump_PatchIndex++;
            JumpPatch_NodeIndex = 0;
        }

        layerJumpPatchSet[Jump_PatchIndex]->GetNodeList().AddTail(Node);
        Node->Jump_SecIndex = JumpPatch_NodeIndex;
        JumpPatch_NodeIndex++;
    }
    //std::cout << "-----------------------------------" << std::endl;
    //std::cout << "--> Split ToolPath into JumpSection" << std::endl;

    return layerJumpPatchSet;
}

double robotWpGeneration::_safe_acos(double value) {
    if (value <= -1.0) {
        return PI;
    }
    else if (value >= 1.0) {
        return 0;
    }
    else {
        return acos(value);
    }
}

void robotWpGeneration::_markSingularNode(QMeshPatch* patch) {

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        Eigen::Vector2d Cspece_Coord;
        // Cal C space Coordinate : Cspece = Nx/Nz, Ny/Nz;
        Cspece_Coord << Node->m_printNor(0) / Node->m_printNor(2),
            Node->m_printNor(1) / Node->m_printNor(2);

        double R = Cspece_Coord.norm();
        double radLambda = DEGREE_TO_ROTATE(m_lambdaValue);

        if (R < tan(radLambda)) Node->isSingularNode = true;
    }
}

void robotWpGeneration::_filterSingleSingularNode(QMeshPatch* patch) {

    //protect
    if (patch->GetNodeNumber() < 4) return;

    std::vector<QMeshNode*> nodeSet(patch->GetNodeNumber());
    std::vector<bool> kept_Singular_Flag(patch->GetNodeNumber());

    int tempIndex = 0;
    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        nodeSet[tempIndex] = Node;
        kept_Singular_Flag[tempIndex] = Node->isSingularNode;
        tempIndex++;
    }


    // remove OXX ... XOX ... XXO
    for (int i = 0; i < kept_Singular_Flag.size(); i++) {

        if (i == 0) {
            if (kept_Singular_Flag[i] == false && kept_Singular_Flag[i + 1] == true && kept_Singular_Flag[i + 2] == true) {
                nodeSet[i]->isSingularNode = true;
            }
        }
        else if (i == (kept_Singular_Flag.size() - 1)) {
            if (kept_Singular_Flag[i - 2] == true && kept_Singular_Flag[i - 1] == true && kept_Singular_Flag[i] == false) {
                nodeSet[i]->isSingularNode = true;
            }
        }
        else {
            if (kept_Singular_Flag[i - 1] == true && kept_Singular_Flag[i] == false && kept_Singular_Flag[i + 1] == true) {
                nodeSet[i]->isSingularNode = true;
            }
        }
    }

    // remove XOOX
    if (patch->GetNodeNumber() < 5) return;
    for (int i = 0; i < kept_Singular_Flag.size(); i++) {
        kept_Singular_Flag[i] = nodeSet[i]->isSingularNode;
    }
    for (int i = 0; i < kept_Singular_Flag.size() - 3; i++) {

        if (kept_Singular_Flag[i] == true
            && kept_Singular_Flag[i + 1] == false
            && kept_Singular_Flag[i + 2] == false
            && kept_Singular_Flag[i + 3] == true) {
            nodeSet[i + 1]->isSingularNode = true;
            nodeSet[i + 2]->isSingularNode = true;
        }
    }
    // remove XOOOX
    if (patch->GetNodeNumber() < 6) return;
    for (int i = 0; i < kept_Singular_Flag.size(); i++) {
        kept_Singular_Flag[i] = nodeSet[i]->isSingularNode;
    }
    for (int i = 0; i < kept_Singular_Flag.size() - 4; i++) {

        if (kept_Singular_Flag[i] == true
            && kept_Singular_Flag[i + 1] == false
            && kept_Singular_Flag[i + 2] == false
            && kept_Singular_Flag[i + 3] == false
            && kept_Singular_Flag[i + 4] == true) {
            nodeSet[i + 1]->isSingularNode = true;
            nodeSet[i + 2]->isSingularNode = true;
            nodeSet[i + 3]->isSingularNode = true;
        }
    }
}

void robotWpGeneration::_getSingularSec(QMeshPatch* patch, Eigen::MatrixXd& sectionTable) {

    int lines = patch->GetNodeNumber();
    std::vector<int> srtPntIndTable, endPntIndTable;

    for (int i = 0; i < lines - 1; i++) {

        GLKPOSITION Node_Pos = patch->GetNodeList().FindIndex(i);
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetAt(Node_Pos);

        GLKPOSITION nextNode_Pos = patch->GetNodeList().FindIndex(i)->next;
        QMeshNode* nextNode = (QMeshNode*)patch->GetNodeList().GetAt(nextNode_Pos);


        if ((Node->isSingularNode == false && nextNode->isSingularNode == true)
            || (Node->isSingularNode == true && Node->Jump_SecIndex == 0)) {
            srtPntIndTable.push_back(Node->Jump_SecIndex);
            Node->isSingularSecStartNode = true;
        }
        if ((Node->isSingularNode == true && nextNode->isSingularNode == false)
            || (nextNode->isSingularNode == true && nextNode->Jump_SecIndex == lines - 1)) {
            endPntIndTable.push_back(nextNode->Jump_SecIndex);
            nextNode->isSingularSecEndNode = true;
        }
    }

    if (srtPntIndTable.size() == endPntIndTable.size()) sectionTable.resize(srtPntIndTable.size(), 2);
    else std::cout << "ERROR : srtPntIndTable.size() != endPntIndTable.size()" << std::endl;

    for (int i = 0; i < srtPntIndTable.size(); i++) {
        sectionTable(i, 0) = srtPntIndTable[i];
        sectionTable(i, 1) = endPntIndTable[i];
    }
    //std::cout << "sectionTable:\n"<<sectionTable << std::endl;
}

void robotWpGeneration::_projectAnchorPoint(QMeshPatch* patch) {

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        if (Node->isSingularSecStartNode == true && Node->isSingularSecEndNode == true) {
            std::cout << "Error: the normal of anchor point cannot move to the boundary of singular region" << std::endl;
            std::cout << "Error: as one normal cannot move to double directions" << std::endl;
        }

        if (Node->GetIndexNo() == 0 || Node->GetIndexNo() == (patch->GetNodeNumber() - 1)) continue;


        if (Node->isSingularSecStartNode == true || Node->isSingularSecEndNode == true) {

            Eigen::Vector3d m_printNor_before = Node->m_printNor;

            double anchor_Nz = cos(DEGREE_TO_ROTATE(m_lambdaValue));
            double temp_k = Node->m_printNor(1) / Node->m_printNor(0);
            double anchor_Nx = sqrt((1 - anchor_Nz * anchor_Nz) / (1 + temp_k * temp_k));
            if (Node->m_printNor(0) < 0.0) anchor_Nx = -anchor_Nx;
            double anchor_Ny = anchor_Nx * temp_k;

            Node->SetNormal(anchor_Nx, anchor_Ny, anchor_Nz);
            Node->SetNormal_last(anchor_Nx, anchor_Ny, anchor_Nz);
            Node->m_printNor << anchor_Nx, anchor_Ny, anchor_Nz;

            //cal the angle of before and after of anchor normal
            if (false) {
                double change = ROTATE_TO_DEGREE(
                    _safe_acos(
                        m_printNor_before.dot(Node->m_printNor)
                        / m_printNor_before.norm() / Node->m_printNor.norm()));
                std::cout << " the angle of before and after of anchor normal is " << change << std::endl;
            }
        }
    }
}

void robotWpGeneration::_getBCtable2(QMeshPatch* patch, Eigen::MatrixXd& B1C1table, Eigen::MatrixXd& B2C2table) {

    int lines = patch->GetNodeNumber();

    B1C1table = Eigen::MatrixXd::Zero(lines, 2);	B2C2table = Eigen::MatrixXd::Zero(lines, 2);

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        int i = Node->Jump_SecIndex;

        // solve 1
        // B1 deg // -acos(Nz)
        B1C1table(i, 0) = ROTATE_TO_DEGREE(-_safe_acos(Node->m_printNor(2)));
        // C1 deg //atan2(Nx, Ny)
        B1C1table(i, 1) = ROTATE_TO_DEGREE(-atan2(Node->m_printNor(1), Node->m_printNor(0)));

        // solve 2
        // B2 deg // acos(Nz)
        B2C2table(i, 0) = -B1C1table(i, 0);
        // C2 deg //atan2(Ny, Nx) +/- 180
        double C2temp = B1C1table(i, 1) + 180.0;
        if (C2temp > 180.0)C2temp -= 360.0; // control the range of C2 into the (-180,180]
        B2C2table(i, 1) = C2temp;

        // only use solve 2
        // modified by tianyu 10/04/2022
        //B1C1table(i, 0) = B2C2table(i, 0);
        //B1C1table(i, 1) = B2C2table(i, 1);
    }
}

void robotWpGeneration::_motionPlanning3(
    QMeshPatch* patch, const Eigen::MatrixXd& sectionTable, const Eigen::MatrixXd& B1C1table,
    const Eigen::MatrixXd& B2C2table, Eigen::RowVector2d& prevBC) {

    int lines = patch->GetNodeNumber();
    Eigen::MatrixXd BC_Matrix(lines, 2); BC_Matrix = Eigen::MatrixXd::Zero(lines, 2);
    std::vector<int> solveFlag(lines);// 1 -> solve 1 // 2 -> solve 2
    // std::vector<int> insertNum(lines);// insertNum for large BC change at the beginning point
    int sectionNumber = 0;
    int sectionAmount = sectionTable.rows();

    int i = 0;

    while (i < lines) {
        //all points of current path are OUT of the sigularity region
        if (sectionAmount == 0) {

            if (_chooseB1C1(B1C1table.row(i), B2C2table.row(i), prevBC)) {
                prevBC << B1C1table.row(i);
                solveFlag[i] = 1;
            }
            else {
                prevBC << B2C2table.row(i);
                solveFlag[i] = 2;
            }

            BC_Matrix.row(i) = prevBC;
            i = i + 1;
        }
        else {
            Eigen::RowVector2d tempBC;
            //all points of current path are IN the sigularity region
            if (i == sectionTable(sectionNumber, 0) && i == 0 && sectionTable(sectionNumber, 1) == (lines - 1)) {
                for (int secLineIndex = i; secLineIndex < lines; secLineIndex++) {

                    tempBC = { 0.0, prevBC(1) };
                    prevBC = tempBC;
                    solveFlag[secLineIndex] = 1;
                    BC_Matrix.row(secLineIndex) = prevBC;
                }
                i = lines;
            }
            // start from the singularity region (end in singularity region or not)
            else if (i == sectionTable(sectionNumber, 0) && i == 0 && sectionTable(sectionNumber, 1) != (lines - 1)) {

                int secEndIndex = sectionTable(sectionNumber, 1);

                for (int secLineIndex = i; secLineIndex < secEndIndex; secLineIndex++) {

                    if (_chooseB1C1(B1C1table.row(secEndIndex), B2C2table.row(secEndIndex), prevBC)
                        || (secLineIndex == 0) // this can make sure start from solution 1
                        ) {
                        tempBC << B1C1table.row(secEndIndex);
                        solveFlag[secLineIndex] = 1;
                    }
                    else {
                        tempBC << B2C2table.row(secEndIndex);
                        solveFlag[secLineIndex] = 2;
                    }

                    prevBC = tempBC;
                    BC_Matrix.row(secLineIndex) = prevBC;

                }

                i = secEndIndex;
                if (sectionNumber != (sectionAmount - 1))	sectionNumber++;
            }
            // end in the singularity region / finish path
            else if (i == sectionTable(sectionNumber, 0) && i != 0 && sectionTable(sectionNumber, 1) == (lines - 1)) {

                int secStartIndex = sectionTable(sectionNumber, 0);

                for (int secLineIndex = i; secLineIndex < lines; secLineIndex++) {

                    if (_chooseB1C1(B1C1table.row(secStartIndex), B2C2table.row(secStartIndex), prevBC)) {
                        tempBC << B1C1table.row(secStartIndex);
                        solveFlag[secLineIndex] = 1;
                    }
                    else {
                        tempBC << B2C2table.row(secStartIndex);
                        solveFlag[secLineIndex] = 2;
                    }

                    prevBC = tempBC;
                    BC_Matrix.row(secLineIndex) = prevBC;
                }

                i = lines;
            }
            // path passes through the sigularity region
            else if (i == sectionTable(sectionNumber, 0) && i != 0 && sectionTable(sectionNumber, 1) != (lines - 1)) {

                // give the message to anchor point (start point)
                int secStartIndex = sectionTable(sectionNumber, 0);
                if (_chooseB1C1(B1C1table.row(secStartIndex), B2C2table.row(secStartIndex), prevBC)) {
                    prevBC << B1C1table.row(secStartIndex);
                    solveFlag[secStartIndex] = 1;
                }
                else {
                    prevBC << B2C2table.row(secStartIndex);
                    solveFlag[secStartIndex] = 2;
                }

                // record the deg_BC of secStart point
                Eigen::RowVector2d startPntBC = prevBC;

                // decide the solve of End point
                int secEndIndex = sectionTable(sectionNumber, 1);
                int pointNum = secEndIndex - secStartIndex;

                double rad_end_B1 = DEGREE_TO_ROTATE(B1C1table(secEndIndex, 0));	double rad_end_C1 = DEGREE_TO_ROTATE(B1C1table(secEndIndex, 1));
                double rad_end_B2 = DEGREE_TO_ROTATE(B2C2table(secEndIndex, 0));	double rad_end_C2 = DEGREE_TO_ROTATE(B2C2table(secEndIndex, 1));
                double rad_start_B = DEGREE_TO_ROTATE(startPntBC(0));				double rad_start_C = DEGREE_TO_ROTATE(startPntBC(1));

                Eigen::Vector2d v_start_C = { cos(rad_start_C),sin(rad_start_C) };
                Eigen::Vector2d v_end_C1 = { cos(rad_end_C1),sin(rad_end_C1) };
                Eigen::Vector2d v_end_C2 = { cos(rad_end_C2),sin(rad_end_C2) };
                //compute the actural angle of 2 vectors
                double rad_end_C1_start_C = _safe_acos(v_end_C1.dot(v_start_C));		double rad_end_B1_start_B = rad_end_B1 - rad_start_B;
                double rad_end_C2_start_C = _safe_acos(v_end_C2.dot(v_start_C));		double rad_end_B2_start_B = rad_end_B2 - rad_start_B;
                //get rad_C/B_start_end
                double rad_C_start_end = 0.0;
                double rad_B_start_end = 0.0;
                Eigen::Vector2d v_end_C = { 0.0,0.0 };

                int solveFlag_passThrough = 0; // 1 -> solve 1 // 2 -> solve 2

                if ((rad_end_C1_start_C) <= (rad_end_C2_start_C)) {
                    rad_C_start_end = rad_end_C1_start_C;
                    rad_B_start_end = rad_end_B1_start_B;
                    v_end_C = v_end_C1;
                    solveFlag_passThrough = 1;
                }
                else {
                    rad_C_start_end = rad_end_C2_start_C;
                    rad_B_start_end = rad_end_B2_start_B;
                    v_end_C = v_end_C2;
                    solveFlag_passThrough = 2;
                }

                //decide the rotation direction of C axis
                double sign = _toLeft({ 0.0,0.0 }, v_start_C, v_end_C);

                //get tha delta Angel of deg_B/C
                double C_delta_Angle = ROTATE_TO_DEGREE(rad_C_start_end) / pointNum;
                double B_delta_Angle = ROTATE_TO_DEGREE(rad_B_start_end) / pointNum;

                unsigned int times = 0;
                for (int secLineIndex = secStartIndex; secLineIndex < secEndIndex; secLineIndex++) {

                    tempBC(0) = startPntBC(0) + times * B_delta_Angle;
                    tempBC(1) = startPntBC(1) + sign * times * C_delta_Angle;

                    prevBC = tempBC;

                    if (prevBC(1) > 180.0) prevBC(1) -= 360.0;
                    if (prevBC(1) < -180.0) prevBC(1) += 360.0;

                    solveFlag[secLineIndex] = solveFlag_passThrough;
                    BC_Matrix.row(secLineIndex) = prevBC;

                    times++;
                }

                i = secEndIndex;

                if (sectionNumber != (sectionAmount - 1))	sectionNumber = sectionNumber + 1;

            }
            // other points out of the singularity region
            else {

                if (_chooseB1C1(B1C1table.row(i), B2C2table.row(i), prevBC)) {

                    prevBC << B1C1table.row(i);
                    solveFlag[i] = 1;
                }
                else {
                    prevBC << B2C2table.row(i);
                    solveFlag[i] = 2;
                }

                BC_Matrix.row(i) = prevBC;
                i = i + 1;
            }
        }
    }

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        int nodeIndex = Node->Jump_SecIndex;
        Node->m_XYZBC(3) = BC_Matrix(nodeIndex, 0); //deg
        Node->m_XYZBC(4) = BC_Matrix(nodeIndex, 1); //deg

        Node->solveSeclct = solveFlag[nodeIndex];
        //cout << Node->solveSeclct << endl;
    }
}

bool robotWpGeneration::_chooseB1C1(
    const Eigen::RowVector2d& B1C1, const Eigen::RowVector2d& B2C2, Eigen::RowVector2d& prevBC) {

    double rad_B1 = DEGREE_TO_ROTATE(B1C1(0));	double rad_C1 = DEGREE_TO_ROTATE(B1C1(1));
    double rad_B2 = DEGREE_TO_ROTATE(B2C2(0));	double rad_C2 = DEGREE_TO_ROTATE(B2C2(1));
    double rad_Bp = DEGREE_TO_ROTATE(prevBC(0)); double rad_Cp = DEGREE_TO_ROTATE(prevBC(1));

    Eigen::Vector2d v_Cp = { cos(rad_Cp),sin(rad_Cp) };
    Eigen::Vector2d v_C1 = { cos(rad_C1),sin(rad_C1) };
    Eigen::Vector2d v_C2 = { cos(rad_C2),sin(rad_C2) };
    //compute the actural angle

    double rad_v_C1_v_Cp = _safe_acos(v_C1.dot(v_Cp));		double rad_B1_rad_Bp = fabs(rad_B1 - rad_Bp);
    double rad_v_C2_v_Cp = _safe_acos(v_C2.dot(v_Cp));		double rad_B2_rad_Bp = fabs(rad_B2 - rad_Bp);

    bool isB1C1 = true;

    if ((rad_v_C1_v_Cp + rad_B1_rad_Bp) > (rad_v_C2_v_Cp + rad_B2_rad_Bp)) {
        isB1C1 = false;

        //std::cout << "----------------------------\n use 2 solve" << std::endl;
        //std::cout << "B1C1 = " << B1C1 << std::endl;
        //std::cout << "B2C2 = " << B2C2 << std::endl;
        //std::cout << "prevBC = " << prevBC << std::endl;
        //std::cout << "rad_v_C1_v_Cp = " << rad_v_C1_v_Cp << std::endl;
        //std::cout << "rad_B1_rad_Bp = " << rad_B1_rad_Bp << std::endl;
        //std::cout << "rad_v_C2_v_Cp = " << rad_v_C2_v_Cp << std::endl;
        //std::cout << "rad_B2_rad_Bp = " << rad_B2_rad_Bp << std::endl;
    }
    return isB1C1;
}

double robotWpGeneration::_toLeft(
    const Eigen::RowVector2d& origin_p, const Eigen::RowVector2d& startPnt_q, const Eigen::RowVector2d& endPnt_s) {

    double Area2 = origin_p(0) * startPnt_q(1) - origin_p(1) * startPnt_q(0)
        + startPnt_q(0) * endPnt_s(1) - startPnt_q(1) * endPnt_s(0)
        + endPnt_s(0) * origin_p(1) - endPnt_s(1) * origin_p(0);

    double isLeft = -1.0;
    if (Area2 > 0.0) isLeft = 1.0;

    return isLeft;
}

void robotWpGeneration::_getXYZ(QMeshPatch* patch) {

    for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

        Eigen::Vector4d node_Pos = Eigen::Vector4d::Zero();
        node_Pos 
            << Node->m_printPos(0), 
               Node->m_printPos(1), 
               Node->m_printPos(2), 
               1.0;

        Eigen::Matrix4d pxpypz_Offset_XYZ;
        pxpypz_Offset_XYZ 
            << 1, 0, 0, abb_offset[0],
               0, 1, 0, abb_offset[1],
               0, 0, 1, abb_offset[2],
               0, 0, 0, 1;

        Eigen::Vector4d temp_4d = Eigen::Vector4d::Zero(); 
        temp_4d = pxpypz_Offset_XYZ * node_Pos;

        Node->m_XYZBC(0) = temp_4d[0];
        Node->m_XYZBC(1) = temp_4d[1];
        Node->m_XYZBC(2) = temp_4d[2];
    }
}

void robotWpGeneration::_optimizationC(QMeshPatch* patch) {

    for (int loop = 0; loop < 50; loop++) {

        double threshhold = 180.0;

        for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)patch->GetNodeList().GetNext(Pos);

            double C = Node->m_XYZBC(4); // deg

            if (Node->GetIndexNo() == 0) continue;
            GLKPOSITION prevPos = patch->GetNodeList().Find(Node)->prev;
            QMeshNode* prevNode = (QMeshNode*)patch->GetNodeList().GetAt(prevPos);
            double preC = prevNode->m_XYZBC(4);

            if (C - preC < -threshhold) {
                C = C + 360;
            }
            else if (C - preC > threshhold) {
                C = C - 360;
            }
            else {}

            Node->m_XYZBC(4) = C;
        }
    }
}

void robotWpGeneration::_verifyPosNor() {

    std::cout << "------------------------------------------- PosNor verification running ... " << std::endl;
    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            double rad_B = DEGREE_TO_ROTATE(Node->m_XYZBC(3));
            double rad_C = DEGREE_TO_ROTATE(Node->m_XYZBC(4));

            double finalNx = -cos(rad_C) * sin(rad_B);
            double finalNy = sin(rad_B) * sin(rad_C);
            double finalNz = cos(rad_B);

            Eigen::Vector3d finalNormal = { finalNx, finalNy, finalNz };
            double angle = _getAngle3D(finalNormal, Node->m_printNor, true);
            //if (angle >= 0.0001) {
            if (angle > (m_lambdaValue * 2 + 1.0)) {
                //if (Node->isSingularNode) cout << "this is a singular node";
                std::cout << "--------------------------------" << std::endl;
                std::cout << "Layer " << WayPointPatch->GetIndexNo() << " Point Index " << Node->GetIndexNo() << std::endl;
                std::cout << "The angle is " << angle << std::endl;
                std::cout << "final Normal\n" << finalNormal.transpose() << std::endl;
                std::cout << "print Normal\n" << Node->m_printNor.transpose() << std::endl;
            }

            //update the normal after singular optimization
            Node->SetNormal(finalNormal(0), finalNormal(1), finalNormal(2));// for show the final normal on the GUI
            Node->SetNormal_last(finalNormal(0), finalNormal(1), finalNormal(2));
            Node->m_printNor = finalNormal;
        }

    }

    std::cout << "------------------------------------------- PosNor verification Finish!\n" << std::endl;

}

double robotWpGeneration::_getAngle3D(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const bool in_degree) {

    //compute the actural angle
    double rad = v1.normalized().dot(v2.normalized());//dot product
    if (rad < -1.0)
        rad = -1.0;
    else if (rad > 1.0)
        rad = 1.0;
    return (in_degree ? _safe_acos(rad) * 180.0 / PI : _safe_acos(rad));
}

void robotWpGeneration::build_Gcode_table(PolygenMesh* toolpathSet, Eigen::MatrixXf& Gcode_Table, int sIdx, int eIdx) {

    m_Waypoints = toolpathSet;

    //protect the input data
    if (sIdx < 0) sIdx = 0;    if (eIdx < 0) eIdx = 0;

    int wp_num = m_Waypoints->GetMeshList().GetCount();

    if (sIdx >= wp_num) sIdx = wp_num - 1;    if (eIdx > wp_num) eIdx = wp_num - 1;

    if (sIdx > eIdx) {
        int _temp = sIdx;
        sIdx = eIdx;
        eIdx = _temp;
    }

    //get the line number of table
    int lines = 0;
    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < sIdx) continue;
        if (WayPointPatch->GetIndexNo() > eIdx) continue;

        lines += WayPointPatch->GetNodeNumber();
    }

    Gcode_Table = Eigen::MatrixXf::Zero(lines, 6);
    lines = 0;
    for (GLKPOSITION Pos = m_Waypoints->GetMeshList().GetHeadPosition(); Pos;) {
        QMeshPatch* WayPointPatch = (QMeshPatch*)m_Waypoints->GetMeshList().GetNext(Pos);

        if (WayPointPatch->GetIndexNo() < sIdx) continue;
        if (WayPointPatch->GetIndexNo() > eIdx) continue;

        for (GLKPOSITION Pos = WayPointPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)WayPointPatch->GetNodeList().GetNext(Pos);

            Gcode_Table.row(lines) << 
                Node->m_XYZBC(0), 
                Node->m_XYZBC(1), 
                Node->m_XYZBC(2), 
                Node->m_XYZBC(3), 
                Node->m_XYZBC(4), 
                WayPointPatch->GetIndexNo();
            lines++;
        }        
    }

    std::cout << "Value range of X axis: [" << Gcode_Table.col(0).maxCoeff()
        << ", " << Gcode_Table.col(0).minCoeff() << "]" << std::endl;
    std::cout << "Value range of Y axis: [" << Gcode_Table.col(1).maxCoeff()
        << ", " << Gcode_Table.col(1).minCoeff() << "]" << std::endl;
    std::cout << "Value range of Z axis: [" << Gcode_Table.col(2).maxCoeff()
        << ", " << Gcode_Table.col(2).minCoeff() << "]" << std::endl;
    std::cout << "Value range of B axis: [" << Gcode_Table.col(3).maxCoeff()
        << ", " << Gcode_Table.col(3).minCoeff() << "]" << std::endl;
    std::cout << "Value range of C axis: [" << Gcode_Table.col(4).maxCoeff()
        << ", " << Gcode_Table.col(4).minCoeff() << "]" << std::endl;

    std::cout << "------------------------------------------- Gcode Load Finish!" << std::endl;
}

