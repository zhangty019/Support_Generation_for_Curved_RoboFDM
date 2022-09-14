#include "stdafx.h"

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>
#include <QtDebug>
#include <QDesktopWidget>
#include <QCoreApplication>
#include <QMimeData>
#include <QTreeView>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include <QScreen>
#include <QStyleFactory>
#include <fstream>

#include "../GLKLib/GLKCameraTool.h"
#include "../GLKLib/InteractiveTool.h"
#include "../GLKLib/GLKMatrixLib.h"
#include "../GLKLib/GLKGeometry.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshTetra.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

#include "alphanum.hpp"
#include <dirent.h>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	QApplication::setStyle(QStyleFactory::create("Fusion"));

    signalMapper = new QSignalMapper(this);
    addToolBar(ui->toolBar);
    addToolBar(ui->navigationToolBar);
    addToolBar(ui->selectionToolBar);

    createTreeView();
    createActions();

    pGLK = new GLKLib();
    ui->horizontalLayout->addWidget(pGLK);
    ui->horizontalLayout->setMargin(0);
    pGLK->setFocus();

    pGLK->clear_tools();
    pGLK->set_tool(new GLKCameraTool(pGLK,ORBITPAN));
	
	//connect timer with timer function
	//connect(&Gcode_timer, SIGNAL(timeout()), this, SLOT(doTimerGcodeMoving()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::createActions()
{
    // file IO
    connect(ui->actionOpen, SIGNAL(triggered(bool)), this, SLOT(open()));
    connect(ui->actionSave, SIGNAL(triggered(bool)), this, SLOT(save()));
	connect(ui->actionSaveSelection, SIGNAL(triggered(bool)), this, SLOT(saveSelection()));
	connect(ui->actionReadSelection, SIGNAL(triggered(bool)), this, SLOT(readSelection()));

    // navigation
    connect(ui->actionFront, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBack, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionTop, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBottom, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionLeft, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionRight, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionIsometric, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_In, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Out, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_All, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Window, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionFront, 0);
    signalMapper->setMapping (ui->actionBack, 1);
    signalMapper->setMapping (ui->actionTop, 2);
    signalMapper->setMapping (ui->actionBottom, 3);
    signalMapper->setMapping (ui->actionLeft, 4);
    signalMapper->setMapping (ui->actionRight, 5);
    signalMapper->setMapping (ui->actionIsometric, 6);
    signalMapper->setMapping (ui->actionZoom_In, 7);
    signalMapper->setMapping (ui->actionZoom_Out, 8);
    signalMapper->setMapping (ui->actionZoom_All, 9);
    signalMapper->setMapping (ui->actionZoom_Window, 10);

    // view
    connect(ui->actionShade, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionMesh, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionProfile, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionFaceNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNodeNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionShade, 20);
    signalMapper->setMapping (ui->actionMesh, 21);
    signalMapper->setMapping (ui->actionNode, 22);
    signalMapper->setMapping (ui->actionProfile, 23);
    signalMapper->setMapping (ui->actionFaceNormal, 24);
    signalMapper->setMapping (ui->actionNodeNormal, 25);
    ui->actionShade->setChecked(true);

    connect(ui->actionShifttoOrigin, SIGNAL(triggered(bool)), this, SLOT(shiftToOrigin()));

    // select
    connect(ui->actionSelectNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectEdge, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectFace, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectFix, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectHandle, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));

	signalMapper->setMapping (ui->actionSelectNode, 30);
    signalMapper->setMapping (ui->actionSelectEdge, 31);
    signalMapper->setMapping (ui->actionSelectFace, 32);
	signalMapper->setMapping(ui->actionSelectFix, 33);
	signalMapper->setMapping(ui->actionSelectHandle, 34);


    connect (signalMapper, SIGNAL(mapped(int)), this, SLOT(signalNavigation(int)));

	//Button
    connect(ui->pushButton_readRobot_model, SIGNAL(released()), this, SLOT(import_CAD_RoboSYS()));


    connect(ui->pushButton_readGcodeSourceData, SIGNAL(released()), this, SLOT(readSliceData()));
    connect(ui->pushButton_Comp_initialGuess_envelopSupport, SIGNAL(released()), this, SLOT(compute_initial_Guess_SupportEnvelope()));
    
    connect(ui->pushButton_buildSupportRaySet, SIGNAL(released()), this, SLOT(build_SupportRAY()));
    connect(ui->pushButton_buildSupportLayerSet, SIGNAL(released()), this, SLOT(build_SupportMesh()));
    connect(ui->pushButton_buildSupportToolpathSet, SIGNAL(released()), this, SLOT(get_CurvedToolpath()));
    connect(ui->pushButton_output_Toolpath, SIGNAL(released()), this, SLOT(output_Toolpath()));

	connect(ui->pushButton_ShowAllLayers, SIGNAL(released()), this, SLOT(viewAll_Layers()));
	connect(ui->spinBox_ShowLayerIndex, SIGNAL(valueChanged(int)), this, SLOT(change_LayerDisplay()));
    connect(ui->radioButton_showRayOrSurface, SIGNAL(released()), this, SLOT(showRayOrSurface()));
    connect(ui->checkBox_draw_LargeISOlayers, SIGNAL(released()), this, SLOT(show_ISO_Layer()));
    connect(ui->radioButton_tightSupportLayerDraw, SIGNAL(released()), this, SLOT(showTightSupportSurface()));
    connect(ui->radioButton_deselect_origin, SIGNAL(released()), this, SLOT(deSelect_origin()));

    connect(ui->pushButton_TestFunc, SIGNAL(released()), this, SLOT(test_func()));
}

void MainWindow::open()
{
    QString filenameStr = QFileDialog::getOpenFileName(this, tr("Open File,"), "..//DataSet//TET//", tr(""));
    QFileInfo fileInfo(filenameStr);
    QString fileSuffix = fileInfo.suffix();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    
    if (QString::compare(fileSuffix,"obj") == 0){
        PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);
        polygenMesh->ImportOBJFile(filename,modelName);
        polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
        pGLK->AddDisplayObj(polygenMesh,true);
        polygenMeshList.AddTail(polygenMesh);
    }

	else if (QString::compare(fileSuffix, "tet") == 0) {
		PolygenMesh *polygenMesh = new PolygenMesh(TET);
		std::cout << "Input tetrahedral mesh from: " << filename << std::endl;
		std::cout << "The model name is '" << modelName << "'" << std::endl;
		polygenMesh->ImportTETFile(filename, modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);
	}

    updateTree();

    shiftToOrigin();
    pGLK->refresh(true);
}

void MainWindow::save()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (!polygenMesh)
		return;
	QString filenameStr = QFileDialog::getSaveFileName(this, tr("OBJ File Export,"), "..", tr("OBJ(*.obj)"));
	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();

	if (QString::compare(fileSuffix, "obj") == 0) {
		QFile exportFile(filenameStr);
		if (exportFile.open(QFile::WriteOnly | QFile::Truncate)) {
			QTextStream out(&exportFile);
			for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
				for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
					QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
					double xx, yy, zz;
					node->GetCoord3D(xx, yy, zz);
					float r, g, b;
					node->GetColor(r, g, b);
					out << "v " << xx << " " << yy << " " << zz << " " << node->value1 << endl;
				}
				for (GLKPOSITION posFace = patch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
					QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(posFace);
					out << "f " << face->GetNodeRecordPtr(0)->GetIndexNo() << " " << face->GetNodeRecordPtr(1)->GetIndexNo() << " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
				}
			}
		}
		exportFile.close();
	}
}

void MainWindow::saveSelection()
{
	//printf("%s exported\n", Model->ModelName);

	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();
	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char output_filename[256];
	strcpy(output_filename, "..\\selection_file\\");
	strcat(output_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(output_filename, filetype);

	ofstream nodeSelection(output_filename);
	if (!nodeSelection)
		cerr << "Sorry!We were unable to build the file NodeSelect!\n";
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		nodeSelection << CheckNode->GetIndexNo() << ":";
		//for the selection of fixing part
		if (CheckNode->isFixed == true) nodeSelection << "1:";
		else nodeSelection << "0:";
		//for the selection of hard part
		if (CheckNode->isHandle == true) nodeSelection << "1:" << endl;
		else nodeSelection << "0:" << endl;
	}

	nodeSelection.close();
	printf("Finish output selection \n");
}

void MainWindow::readSelection()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();

	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char input_filename[256];
	strcpy(input_filename, "..\\selection_file\\");
	strcat(input_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(input_filename, filetype);

	ifstream nodeSelect(input_filename);
	if (!nodeSelect)
		cerr << "Sorry!We were unable to open the file!\n";
	vector<int> NodeIndex(patch->GetNodeNumber()), checkNodeFixed(patch->GetNodeNumber()), checkNodeHandle(patch->GetNodeNumber());
	//string line;
	int LineIndex1 = 0;
	string sss;
	while (getline(nodeSelect, sss)){
		const char * c = sss.c_str();
		sscanf(c, "%d:%d:%d", &NodeIndex[LineIndex1], &checkNodeFixed[LineIndex1], &checkNodeHandle[LineIndex1]);
		LineIndex1++;
	}

	nodeSelect.close();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (checkNodeFixed[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isFixed = true;
		if (checkNodeHandle[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isHandle = true;
	}

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isHandle == true &&
			Face->GetNodeRecordPtr(1)->isHandle == true &&
			Face->GetNodeRecordPtr(2)->isHandle == true)
			Face->isHandleDraw = true;
		else Face->isHandleDraw = false;

		if (Face->GetNodeRecordPtr(0)->isFixed == true &&
			Face->GetNodeRecordPtr(1)->isFixed == true &&
			Face->GetNodeRecordPtr(2)->isFixed == true)
			Face->isFixedDraw = true;
		else Face->isFixedDraw = false;
	}
	printf("Finish input selection \n");
	pGLK->refresh(true);

}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
	//QMouseEvent *e = (QMouseEvent*)event;
	//QPoint pos = e->pos();
	//cout << "Mouse position updated" << endl;
	//double wx, wy, wz;
	//pGLK->screen_to_wcl(100.0, 100.0, wx, wy, wz);
	//ui->CorrdinateMouse->setText(QString("X = %1").arg(wx));

	//QString text;
	//text = QString("%1 X %2").arg(event->pos().x()).arg(event->pos().y());
	///** Update the info text */
	//ui->statusBar->showMessage(text);
}

void MainWindow::signalNavigation(int flag)
{
    if (flag <= 10)
        pGLK->setNavigation(flag);
    if (flag >=20 && flag <=25){
        pGLK->setViewModel(flag-20);
        switch (flag) {
        case 20:
            ui->actionShade->setChecked(pGLK->getViewModel(0));
            break;
        case 21:
            ui->actionMesh->setChecked(pGLK->getViewModel(1));
            break;
        case 22:
            ui->actionNode->setChecked(pGLK->getViewModel(2));
            break;
        case 23:
            ui->actionProfile->setChecked(pGLK->getViewModel(3));
            break;
        case 24:
            ui->actionFaceNormal->setChecked(pGLK->getViewModel(4));
            break;
        case 25:
            ui->actionNodeNormal->setChecked(pGLK->getViewModel(5));
            break;
        }
    }
    if (flag==30 || flag==31 || flag==32 || flag == 33 || flag == 34){
        InteractiveTool *tool;
        switch (flag) {
        case 30:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NODE, ui->boxDeselect->isChecked());
            break;
        case 31:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), EDGE, ui->boxDeselect->isChecked());
            break;
        case 32:
            tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FACE, ui->boxDeselect->isChecked());
            break;
		case 33:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FIX, ui->boxDeselect->isChecked());
			break;
		case 34:
			tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NHANDLE, ui->boxDeselect->isChecked());
			break;
        }
        pGLK->set_tool(tool);
    }
}

void MainWindow::shiftToOrigin()
{
    
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls())
        event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent *event)
{
    QString filenameStr;
    foreach (const QUrl &url, event->mimeData()->urls())
        filenameStr = url.toLocalFile();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    int i = 0;
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
        std::string name = (polygen->getModelName()).substr(0,(polygen->getModelName()).find(' '));
        if (name == modelName)
            i++;
    }
    if (i > 0)
        modelName += " "+std::to_string(i);

	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();
	if (QString::compare(fileSuffix, "obj") == 0) {
		polygenMesh->ImportOBJFile(filename, modelName);
	}
	else if (QString::compare(fileSuffix, "tet") == 0) {
		polygenMesh->ImportTETFile(filename, modelName);
        polygenMesh->meshType = TET;
	}
	polygenMesh->m_bVertexNormalShading = false;	
    polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(polygenMesh,true);
    polygenMeshList.AddTail(polygenMesh);
    
    updateTree();
}

void MainWindow::createTreeView()
{
    treeModel = new QStandardItemModel();
    ui->treeView->setModel(treeModel);
    ui->treeView->setHeaderHidden(true);
    ui->treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    ui->treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->treeView->expandAll();
}

void MainWindow::updateTree()
{
    treeModel->clear();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        QStandardItem *modelListItem = new QStandardItem(modelName);
        modelListItem->setCheckable(true);
        modelListItem->setCheckState(Qt::Checked);
        treeModel->appendRow(modelListItem);
    }
	pGLK->refresh(true);
}

PolygenMesh *MainWindow::getSelectedPolygenMesh()
{
    if (!treeModel->hasChildren())
        return nullptr;
    QModelIndex index = ui->treeView->currentIndex();
    QString selectedModelName = index.data(Qt::DisplayRole).toString();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        if (QString::compare(selectedModelName,modelName) == 0)
            return polygenMesh;
    }
    return nullptr;
}

void MainWindow::on_treeView_clicked(const QModelIndex &index)
{
    ui->treeView->currentIndex();
    QStandardItem *modelListItem = treeModel->itemFromIndex(index);
    ui->treeView->setCurrentIndex(index);
    PolygenMesh *polygenMesh = getSelectedPolygenMesh();
    if (modelListItem->checkState() == Qt::Checked)
        polygenMesh->bShow = true;
    else
        polygenMesh->bShow = false;
    pGLK->refresh(true);
}

void MainWindow::import_CAD_RoboSYS() {

    std::vector<std::string> robotPartsName;
    std::string cad_Dir_Robot = "../DataSet/CAD/Robot";
    _get_FileName(cad_Dir_Robot, robotPartsName, true, false);

    std::vector<std::string> positionerPartsName;
    std::string cad_Dir_Postioner = "../DataSet/CAD/Positioner";
    _get_FileName(cad_Dir_Postioner, positionerPartsName, true, false);

    //for (int i = 0; i < robotPartsName.size(); i++) {
    //    std::cout << robotPartsName[i] << std::endl;
    //}
    //for (int i = 0; i < positionerPartsName.size(); i++) {
    //    std::cout << positionerPartsName[i] << std::endl;
    //}

    //read CAD files and build mesh_patches
    char filename[1024];
    // isBuilt
    if (_detectPolygenMesh(CAD_PARTS, "Robot", false) != NULL) return;
    PolygenMesh* robotModelSet = _buildPolygenMesh(CAD_PARTS, "Robot");
    // robot
    for (int i = 0; i < robotPartsName.size(); i++) {

        std::sprintf(filename, "%s%s%s", cad_Dir_Robot.c_str(), "/", robotPartsName[i].c_str());
        cout << filename << endl;

        QMeshPatch* cadPatch = new QMeshPatch;
        cadPatch->SetIndexNo(robotModelSet->GetMeshList().GetCount()); //index begin from 0
        robotModelSet->GetMeshList().AddTail(cadPatch);
        cadPatch->inputOBJFile(filename);
        cadPatch->patchName = robotPartsName[i].data();

    }

    // isBuilt
    if (_detectPolygenMesh(CAD_PARTS, "Positioner", false) != NULL) return;
    PolygenMesh* positionerModelSet = _buildPolygenMesh(CAD_PARTS, "Positioner");
    // positioner
    for (int i = 0; i < positionerPartsName.size(); i++) {

        std::sprintf(filename, "%s%s%s", cad_Dir_Postioner.c_str(), "/", positionerPartsName[i].c_str());
        cout << filename << endl;

        QMeshPatch* cadPatch = new QMeshPatch;
        cadPatch->SetIndexNo(positionerModelSet->GetMeshList().GetCount()); //index begin from 0
        positionerModelSet->GetMeshList().AddTail(cadPatch);
        cadPatch->inputOBJFile(filename);
        cadPatch->patchName = positionerPartsName[i].data();

    }

    // isBuilt
    if (_detectPolygenMesh(CAD_PARTS, "Floor", false) != NULL) return;
    PolygenMesh* floorSet = _buildPolygenMesh(CAD_PARTS, "Floor");
    // floor
    std::sprintf(filename, "%s", "../DataSet/CAD/floor.obj");
    cout << filename << endl;
    QMeshPatch* cadPatch = new QMeshPatch;
    cadPatch->SetIndexNo(floorSet->GetMeshList().GetCount()); //index begin from 0
    floorSet->GetMeshList().AddTail(cadPatch);
    cadPatch->inputOBJFile(filename);
    cadPatch->patchName = "floor";

    /**** Postion transformation ****/
    Eigen::Vector3d tr(1459.306, -26.9, 2.591); // X - Y - Z 
    cout << endl << "********** EulerAngle **********" << endl;
    //initial (RPY, x-roll, then y-pitch, next z-yaw)
    Eigen::Vector3d ea(89.87, -0.1, 0.073); // Z - Y - X
    for (int i = 0; i < 3; i++) {
        ea[i] = DEGREE_TO_ROTATE(ea[i]);
    }

    //Euler Angel -> rotation matrix
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    cout << "rotation matrix3 =\n" << rotation_matrix3 << endl;

    //move positioner
    for (GLKPOSITION Postioner_Pos = positionerModelSet->GetMeshList().GetHeadPosition(); Postioner_Pos;) {
        QMeshPatch* Postioner_Patch = (QMeshPatch*)positionerModelSet->GetMeshList().GetNext(Postioner_Pos);
        for (GLKPOSITION Pos = Postioner_Patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* node = (QMeshNode*)Postioner_Patch->GetNodeList().GetNext(Pos);

            Eigen::Vector3d pp;
            node->GetCoord3D(pp[0], pp[1], pp[2]);
            pp = rotation_matrix3 * pp + tr;
            node->SetCoord3D(pp[0], pp[1], pp[2]);
        }
    }


    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "-------------------------------------------- CNC Load Finish!" << std::endl;

}
//// only get file name in dirctory and push_back into fileNameCell
//void MainWindow::_get_FileName(string dirctory, vector<string>& fileNameCell) {
//
//    if (fileNameCell.empty() == false) return;
//
//    DIR* dp;
//    struct dirent* ep;
//    string fullDir = dirctory;
//    //cout << fullDir << endl;
//    dp = opendir(fullDir.c_str());
//
//    if (dp != NULL) {
//        while (ep = readdir(dp)) {
//            //cout << ep->d_name << endl;
//            if ((string(ep->d_name) != ".") && (string(ep->d_name) != "..")) {
//                //cout << ep->d_name << endl;
//                fileNameCell.push_back(string(ep->d_name));
//            }
//        }
//        (void)closedir(dp);
//    }
//    else {
//        perror("Couldn't open the directory");
//    }
//}

// true - use polygen type define <-> false - use polygen name define
PolygenMesh* MainWindow::_detectPolygenMesh(mesh_type type, std::string name, bool detectType) {

    PolygenMesh* detectedMesh = NULL;
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (detectType == true) {
            if (thispolygenMesh->meshType == type) {
                detectedMesh = thispolygenMesh; break;
            }
        }
        else {
            if (thispolygenMesh->getModelName() == name) {
                detectedMesh = thispolygenMesh; break;
            }
        }
    }
    return detectedMesh;

}

PolygenMesh* MainWindow::_buildPolygenMesh(mesh_type type, std::string name) {

    PolygenMesh* newMesh = new PolygenMesh(type);
    newMesh->setModelName(name);
    polygenMeshList.AddTail(newMesh);
    pGLK->AddDisplayObj(newMesh, true);
    newMesh->BuildGLList(newMesh->m_bVertexNormalShading);
    updateTree();
    return newMesh;

}

//compute initial Guess SupportEnvelope
void MainWindow::compute_initial_Guess_SupportEnvelope() {
    this->_read_platform();
    this->_read_TET_vectorField();
    std::printf("------------------------------------------- TET and Vfield Load Finish!\n");

    PolygenMesh* tetModel = _detectPolygenMesh(TET, "tet_fabrication", true);
    PolygenMesh* platform = _detectPolygenMesh(CAD_PARTS, "Platform", false);
    PolygenMesh* supportRaySet = _buildPolygenMesh(SUPPORT_RAY, "Support_Ray");

    if (tetModel == NULL || platform == NULL || supportRaySet == NULL)
        std::cout << "There is no needed PolygenMesh, please check" << std::endl;

    supportGene = new supportGeneration();
    supportGene->initial(tetModel, platform, supportRaySet, (ui->lineEdit_SorceDataDir->text()).toStdString());
    supportGene->initial_Guess_SupportEnvelope();//get support node
    supportGene->collect_Support_Polyline_fromTETsurface();// collect support node and edge
    supportGene->computer_initialGuess_EnvelopeHull();// build convex hull for initial guess of support
    supportGene->output_initial_Polyline();

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

// read layers
void MainWindow::readSliceData() {

    char filename[1024];
    //read platform firstly
    this->_read_platform();
    //read iso-Layers
    std::vector<std::string> initLayer_Name;
    std::string initLayer_Dir = "../DataSet/LAYERS/" + (ui->lineEdit_SorceDataDir->text()).toStdString();
    _get_FileName(initLayer_Dir, initLayer_Name, false, false);

    PolygenMesh* sliceSet = NULL;
    sliceSet = _detectPolygenMesh(CURVED_LAYER, "Layers", false);

    if (sliceSet == NULL) {

        sliceSet = _buildPolygenMesh(CURVED_LAYER, "Layers");

        //read slice files and build mesh_patches
        for (int i = 0; i < initLayer_Name.size(); i++){

            std::sprintf(filename, "%s%s%s", initLayer_Dir.c_str(), "/", initLayer_Name[i].data());
            //cout << filename << endl;

            QMeshPatch* slice = new QMeshPatch;
            slice->SetIndexNo(sliceSet->GetMeshList().GetCount()); //index begin from 0
            sliceSet->GetMeshList().AddTail(slice);
            slice->patchName = initLayer_Name[i].data();
            if (slice->patchName.find("S") != string::npos) {
                slice->is_SupportLayer = true;
                slice->largeLayer_Index = stoi(initLayer_Name[i].substr(0, initLayer_Name[i].size() - 5)) / 100; //100 S.obj
            }
            else {
                slice->is_SupportLayer = false;
                slice->largeLayer_Index = stoi(initLayer_Name[i].substr(0, initLayer_Name[i].size() - 4)) / 100; //5900 .obj
            }
            slice->inputOBJFile(filename);

            _modifyCoord(slice);//modifyCoord upY->upZ (only node)

            //pre-calculate the face normal for layers and save into "m_desiredNormal"
            //control the m_desiredNormal to be downward
            int count_Z_up_node = 0;
            for (GLKPOSITION Pos = slice->GetFaceList().GetHeadPosition(); Pos != NULL;) {
                QMeshFace* face = (QMeshFace*)(slice->GetFaceList().GetNext(Pos));
                double n[3];
                face->CalPlaneEquation();
                face->GetNormal(n[0], n[1], n[2]);
                if (n[2] >= 0.0)    count_Z_up_node++;
            }
            bool flip_slice_normal = false;
            // if almost normal upward, we need to flip them
            if ((double)count_Z_up_node / slice->GetFaceNumber() > 0.2) {flip_slice_normal = true;}

            for (GLKPOSITION Pos = slice->GetFaceList().GetHeadPosition(); Pos != NULL;) {
                QMeshFace* face = (QMeshFace*)(slice->GetFaceList().GetNext(Pos));
                double n[3]; double A, B, C, D;
                face->GetPlaneEquation(n[0], n[1], n[2], D);
                if (flip_slice_normal) {
                    for (int i = 0; i < 3; i++) { face->m_desiredNormal[i] = -n[i]; }
                    face->m_desired_D = -D;
                }
                else {
                    for (int i = 0; i < 3; i++) {face->m_desiredNormal[i] =  n[i];}
                    face->m_desired_D = D;
                }
            }
            //pre-calculate the node normal for layers
            for (GLKPOSITION Pos = slice->GetNodeList().GetHeadPosition(); Pos != NULL;) {
                QMeshNode* node = (QMeshNode*)(slice->GetNodeList().GetNext(Pos));

                double n[3];
                node->CalNormal(n);
                if (flip_slice_normal) {
                    for (int i = 0; i < 3; i++) {node->m_desiredNormal[i] = -n[i];}
                }
                else {
                    for (int i = 0; i < 3; i++) {node->m_desiredNormal[i] =  n[i];}
                }
            }       
        }
    }

    //give base movement 
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "bunny_cut" ||
        (ui->lineEdit_SorceDataDir->text()).toStdString() == "CSquare_cut" ||
        (ui->lineEdit_SorceDataDir->text()).toStdString() == "yoga_cut") {
        ui->doubleSpinBox_Zmove->setValue(0.5);
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "topo_cut") {
        ui->doubleSpinBox_Zmove->setValue(6.5);
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "arch_cut") {
        ui->doubleSpinBox_Zmove->setValue(15.5);
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "dome_cut") {
        ui->doubleSpinBox_Zmove->setValue(-4.5);
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "bridgeSmall_cut") {
        ui->doubleSpinBox_Zmove->setValue(10.5);
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "helmetSmall_cut") {
        ui->doubleSpinBox_Zmove->setValue(20.5);
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "airbus_topopt_cut") {
        ui->doubleSpinBox_Zmove->setValue(5.5);
    }


    // Move iso-layers(Xmove, Ymove, Zmove);
    for (GLKPOSITION patchPos = sliceSet->GetMeshList().GetHeadPosition(); patchPos;) {
        QMeshPatch* slice_patch = (QMeshPatch*)sliceSet->GetMeshList().GetNext(patchPos);
        for (GLKPOSITION Pos = slice_patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)slice_patch->GetNodeList().GetNext(Pos);

            double xx, yy, zz;
            Node->GetCoord3D_last(xx, yy, zz);

            xx += ui->doubleSpinBox_Xmove->value();
            yy += ui->doubleSpinBox_Ymove->value();
            zz += ui->doubleSpinBox_Zmove->value();

            Node->SetCoord3D(xx, yy, zz);
        }
    }

    //std::string tetName;
    //if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "bunny_cut"){
    //    tetName = "bunnyhead";
    //}
    //if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "yoga_cut") {
    //    tetName = "YogaNew";
    //}
    //if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "topo_cut") {
    //    tetName = "topopt_new";
    //}
    //
    ////read tet Model and its Vector Field
    //PolygenMesh* tet_polygenMesh = new PolygenMesh(TET);
    //std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    //std::sprintf(filename, "%s%s%s", "../DataSet/", tetName.c_str(),"_fabrication.tet");
    //std::cout << "Input tetrahedral mesh from: \n -->" << filename << std::endl;
    //std::string modelName = tetName + "_fabrication";
    //std::cout << "The model name is: " << modelName << std::endl;
    //tet_polygenMesh->ImportTETFile(filename, modelName);
    //tet_polygenMesh->BuildGLList(tet_polygenMesh->m_bVertexNormalShading);
    //pGLK->AddDisplayObj(tet_polygenMesh, true);
    //polygenMeshList.AddTail(tet_polygenMesh);
    //updateTree();
    //QMeshPatch* tet_Model = (QMeshPatch*)tet_polygenMesh->GetMeshList().GetHead();
    //_modifyCoord(tet_Model);
    //
    //// read the vector field 
    //Eigen::MatrixXd vectorField_set = Eigen::MatrixXd::Zero(tet_Model->GetTetraNumber(),3);
    //std::sprintf(filename, "%s%s%s", "../DataSet/", tetName.c_str(),"_vector_field.txt");
    //FILE* fp;   char linebuf[256];  int i_temp = 0; float nx, ny, nz;
    //fp = fopen(filename, "r");
    //if (!fp) {
    //    printf("===============================================\n");
    //    printf("Can not open the data file - vector Field File Import!\n");
    //    printf("===============================================\n");
    //}
    //while (1) { // right read CODE !!!
    //    fgets(linebuf, 255, fp);
    //    if (feof(fp)) break;
    //    sscanf(linebuf, "%f %f %f\n",&nx, &ny, &nz);
    //    vectorField_set.row(i_temp) << nx, -nz, ny;// coordinate trans upY to upZ
    //    //std::cout << i_temp << std::endl;
    //    i_temp++;
    //}
    //fclose(fp);
    //
    ////give the vector field to TET element
    //i_temp = 0;
    //for (GLKPOSITION posTet = tet_Model->GetTetraList().GetHeadPosition(); posTet != nullptr;) {
    //    QMeshTetra* tet = (QMeshTetra*)tet_Model->GetTetraList().GetNext(posTet);
    //
    //    tet->vectorField = vectorField_set.row(i_temp);
    //    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "bunny_cut"||
    //        (ui->lineEdit_SorceDataDir->text()).toStdString() == "topo_cut" ) {
    //        tet->vectorField = -tet->vectorField;
    //    }
    //    i_temp++;
    //    //std::cout << tet->vectorField.transpose() << std::endl;
    //}
    //std::cout << "finish input vectorField [ " << vectorField_set.rows() << " x " << vectorField_set.cols() << " ]" << std::endl;
    //
    //// Move tet model(Xmove, Ymove, Zmove)
    //for (GLKPOSITION patchPos = tet_polygenMesh->GetMeshList().GetHeadPosition(); patchPos;) {
    //    QMeshPatch* tet_patch = (QMeshPatch*)tet_polygenMesh->GetMeshList().GetNext(patchPos);
    //    for (GLKPOSITION Pos = tet_patch->GetNodeList().GetHeadPosition(); Pos;) {
    //        QMeshNode* Node = (QMeshNode*)tet_patch->GetNodeList().GetNext(Pos);
    //
    //        double xx, yy, zz;
    //        Node->GetCoord3D_last(xx, yy, zz);
    //
    //        xx += ui->doubleSpinBox_Xmove->value();
    //        yy += ui->doubleSpinBox_Ymove->value();
    //        zz += ui->doubleSpinBox_Zmove->value();
    //
    //        Node->SetCoord3D(xx, yy, zz);
    //    }
    //}

    this->_read_TET_vectorField();

    ui->spinBox_ShowLayerIndex->setMaximum(sliceSet->GetMeshList().GetCount() - 1);
    ui->pushButton_buildSupportRaySet->setEnabled(true);
    viewAll_Layers();

    std::cout << "------------------------------------------- Slices Load Finish!" << std::endl;
}

// read platform
void MainWindow::_read_platform() {

    char filename[1024];

    if (_detectPolygenMesh(CAD_PARTS, "Platform", false) == NULL) {

        PolygenMesh* Platform = _buildPolygenMesh(CAD_PARTS, "Platform");
        std::sprintf(filename, "%s", "../DataSet/platform.obj");
        //cout << filename << endl;
        QMeshPatch* cadPatch = new QMeshPatch;
        cadPatch->SetIndexNo(Platform->GetMeshList().GetCount()); //index begin from 0
        Platform->GetMeshList().AddTail(cadPatch);
        cadPatch->inputOBJFile(filename);
        cadPatch->patchName = "platform";

        //enlarge platform
        double ratio = 1.4; double xx, yy, zz;
        for (GLKPOSITION Pos = cadPatch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)cadPatch->GetNodeList().GetNext(Pos);

            Node->GetCoord3D(xx, yy, zz);
            xx *= ratio; yy *= ratio;
            Node->SetCoord3D(xx, yy, zz);
        }
    }
}

// read TET model and its vector field
void MainWindow::_read_TET_vectorField() {

    char filename[1024];

    std::string tetName;
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "bunny_cut") {
        tetName = "bunnyhead";
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "yoga_cut") {
        tetName = "YogaNew";
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "topo_cut") {
        tetName = "topopt_new";
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "dome_cut") {
        tetName = "dome";
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "arch_cut") {
        tetName = "arch";
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "CSquare_cut") {
        tetName = "CSquare";
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "bridgeSmall_cut") {
        tetName = "bridgeSmall";
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "helmetSmall_cut") {
        tetName = "helmetSmall";
    }
    if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "airbus_topopt_cut") {
        tetName = "airbus_topopt";
    }


    //read tet Model
    PolygenMesh* tet_polygenMesh = new PolygenMesh(TET);
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::sprintf(filename, "%s%s%s", "../DataSet/", tetName.c_str(), "_fabrication.tet");
    std::cout << "Input tetrahedral mesh from: \n -->" << filename << std::endl;
    std::string modelName = tetName + "_fabrication";
    std::cout << "The model name is: " << modelName << std::endl;
    tet_polygenMesh->ImportTETFile(filename, modelName);
    tet_polygenMesh->BuildGLList(tet_polygenMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(tet_polygenMesh, true);
    polygenMeshList.AddTail(tet_polygenMesh);
    updateTree();
    QMeshPatch* tet_Model = (QMeshPatch*)tet_polygenMesh->GetMeshList().GetHead();
    _modifyCoord(tet_Model);

    // read the vector field 
    Eigen::MatrixXd vectorField_set = Eigen::MatrixXd::Zero(tet_Model->GetTetraNumber(), 3);
    std::sprintf(filename, "%s%s%s", "../DataSet/", tetName.c_str(), "_vector_field.txt");
    FILE* fp;   char linebuf[256];  int i_temp = 0; float nx, ny, nz;
    fp = fopen(filename, "r");
    if (!fp) {
        printf("===============================================\n");
        printf("Can not open the data file - vector Field File Import!\n");
        printf("===============================================\n");
    }
    while (1) { // right read CODE !!!
        fgets(linebuf, 255, fp);
        if (feof(fp)) break;
        sscanf(linebuf, "%f %f %f\n", &nx, &ny, &nz);
        vectorField_set.row(i_temp) << nx, -nz, ny;// coordinate trans upY to upZ
        //std::cout << i_temp << std::endl;
        i_temp++;
    }
    fclose(fp);

    //give the vector field to TET element
    i_temp = 0;
    for (GLKPOSITION posTet = tet_Model->GetTetraList().GetHeadPosition(); posTet != nullptr;) {
        QMeshTetra* tet = (QMeshTetra*)tet_Model->GetTetraList().GetNext(posTet);

        tet->vectorField = vectorField_set.row(i_temp);
        if ((ui->lineEdit_SorceDataDir->text()).toStdString() == "bunny_cut" ||
            (ui->lineEdit_SorceDataDir->text()).toStdString() == "bridgeSmall_cut" ||
            (ui->lineEdit_SorceDataDir->text()).toStdString() == "topo_cut") {
            tet->vectorField = -tet->vectorField;
        }
        i_temp++;
        //std::cout << tet->vectorField.transpose() << std::endl;
    }

    std::cout << "finish input vectorField [ " << vectorField_set.rows() << " x " << vectorField_set.cols() << " ]" << std::endl;

    // Move tet model(Xmove, Ymove, Zmove)
    for (GLKPOSITION patchPos = tet_polygenMesh->GetMeshList().GetHeadPosition(); patchPos;) {
        QMeshPatch* tet_patch = (QMeshPatch*)tet_polygenMesh->GetMeshList().GetNext(patchPos);
        for (GLKPOSITION Pos = tet_patch->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)tet_patch->GetNodeList().GetNext(Pos);

            double xx, yy, zz;
            Node->GetCoord3D_last(xx, yy, zz);

            xx += ui->doubleSpinBox_Xmove->value();
            yy += ui->doubleSpinBox_Ymove->value();
            zz += ui->doubleSpinBox_Zmove->value();

            Node->SetCoord3D(xx, yy, zz);
        }
    }
    std::cout << "finish move TET model" << std::endl;

}

// get file name in dirctory and push_back into fileNameCell with nature order
// -onlyRead == true;  only read names without sorting
// -onlyInit == true;  only read initial layers
void MainWindow::_get_FileName(string dirctory, vector<string>& fileNameCell, bool onlyRead, bool onlyInit) {

    if (fileNameCell.empty() == false) return;

    DIR* dp;
    struct dirent* ep;
    string fullDir = dirctory;
    //cout << fullDir << endl;
    dp = opendir(fullDir.c_str());

    if (dp != NULL) {
        while (ep = readdir(dp)) {
            //cout << ep->d_name << endl;
            if ((string(ep->d_name) != ".") && (string(ep->d_name) != "..")) {
                //cout << ep->d_name << endl;
                if (onlyInit == true) {
                    //cout << ep->d_name << endl;
                    if (string(ep->d_name).find("S") != string::npos) continue;
                }
                //cout << "-->" << ep->d_name << endl;
                fileNameCell.push_back(string(ep->d_name));
            }
        }
        (void)closedir(dp);
    }
    else {
        perror("Couldn't open the directory");
    }
    //resort the files with nature order
    if(onlyRead == false)
        sort(fileNameCell.begin(), fileNameCell.end(), doj::alphanum_less<std::string>());
}

// modify the Yup sys to Zup sys
void MainWindow::_modifyCoord(QMeshPatch* patchFile) {

    for (GLKPOSITION Pos = patchFile->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)patchFile->GetNodeList().GetNext(Pos);

        double xx, yy, zz, nx, ny, nz;
        Node->GetCoord3D(xx, yy, zz);
        Node->GetNormal(nx, ny, nz);

        double tempPosYZ = yy;
        double tempNorYZ = ny;

        yy = -zz;
        zz = tempPosYZ;

        ny = -nz;
        nz = tempNorYZ;

        Node->SetCoord3D_last(xx, yy, zz);// base data of MoveModel
        Node->SetNormal_last(nx, ny, nz);
        Node->SetCoord3D(xx, yy, zz);
        Node->SetNormal(nx, ny, nz);

    }
}

// control the display of layers
void MainWindow::viewAll_Layers() {

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if ("Layers" != polygenMesh->getModelName()) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = false;//reset

            if (ui->checkBox_onlyShowOnetype_layers->isChecked()) {

                if (ui->radioButton_initialORsupport->isChecked()) {

                    if (Patch->is_SupportLayer) continue;
                }
                else {

                    if (!Patch->is_SupportLayer) continue;
                }
            }

            Patch->drawThisPatch = true;
        }
    }

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if ("Tight_supportLayerSet" != polygenMesh->getModelName() ||
            !ui->radioButton_tightSupportLayerDraw->isChecked()) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = true;
        }
    }
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::change_LayerDisplay() {
    bool single = ui->checkBox_EachLayerSwitch->isChecked();
    int currentLayerIndex = ui->spinBox_ShowLayerIndex->value();
    bool largeIsolayer = ui->checkBox_draw_LargeISOlayers->isChecked();

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if ("Layers" != polygenMesh->getModelName()) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = false;

            if (ui->checkBox_onlyShowOnetype_layers->isChecked()) {

                if (ui->radioButton_initialORsupport->isChecked()) {
                    
                    if (Patch->is_SupportLayer) continue;
                }
                else {

                    if (!Patch->is_SupportLayer) continue;
                }          
            }

            int index = Patch->GetIndexNo();
            int largeIsolayer_index = Patch->largeLayer_Index;

            if (single) {

                if (largeIsolayer) {
                    if (largeIsolayer_index == currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
                else {

                    if (index == currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
            }
            else {
                if (largeIsolayer) {
                    if (largeIsolayer_index <= currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
                else {
                    if (index <= currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
            }
        }
    }

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if ("Tight_supportLayerSet" != polygenMesh->getModelName() ||
            !ui->radioButton_tightSupportLayerDraw->isChecked() ) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = false;//reset

            if (Patch->largeLayer_Index == currentLayerIndex)
                Patch->drawThisPatch = true;
        }
    }

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (
            ("ToolPath_support" != polygenMesh->getModelName() && "ToolPath_initial" != polygenMesh->getModelName()) 
            ||  !ui->radioButton_tightSupportLayerDraw->isChecked()
           ) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = false;//reset

            if (Patch->largeLayer_Index == currentLayerIndex)
                Patch->drawThisPatch = true;
        }
    }

    pGLK->refresh(true);
}

void MainWindow::show_ISO_Layer() {

    bool largeIsolayer = ui->checkBox_draw_LargeISOlayers->isChecked();

    PolygenMesh* layers = _detectPolygenMesh(CURVED_LAYER, "Layers", false);

    if (layers == NULL) {
        std::cout << "There is no needed PolygenMesh <Layers>, please check" << std::endl;
        return;
    }

    int max_Layer_NUM = 0;

    for (GLKPOSITION posMesh = layers->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* Patch = (QMeshPatch*)layers->GetMeshList().GetNext(posMesh);

        if (largeIsolayer) {

            if (Patch->largeLayer_Index > max_Layer_NUM) 
                max_Layer_NUM = Patch->largeLayer_Index;

        }
        else {
            if (Patch->GetIndexNo() > max_Layer_NUM)
                max_Layer_NUM = Patch->GetIndexNo();
        }

    }
    //cout << "max layer num = " << max_Layer_NUM << endl;
    ui->spinBox_ShowLayerIndex->setMaximum(max_Layer_NUM);
    ui->spinBox_ShowLayerIndex->setValue(0);
}

void MainWindow::showRayOrSurface() {

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if ("Support_Ray" != polygenMesh->getModelName()) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = !(Patch->drawThisPatch);
        }
    }
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::showTightSupportSurface() {

    bool showTightSupportLayers = ui->radioButton_tightSupportLayerDraw->isChecked();
    if (!showTightSupportLayers) return;

    PolygenMesh* tightSupportLayers = _detectPolygenMesh(CURVED_LAYER, "Tight_supportLayerSet", false);

    if (tightSupportLayers == NULL) {
        std::cout << "There is no needed PolygenMesh <Tight_supportLayerSet>, please check" << std::endl;
        return;
    }

    //int max_Layer_NUM = 0;
    //for (GLKPOSITION posMesh = tightSupportLayers->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
    //    QMeshPatch* Patch = (QMeshPatch*)tightSupportLayers->GetMeshList().GetNext(posMesh);
    //    if (Patch->largeLayer_Index > max_Layer_NUM)    max_Layer_NUM = Patch->largeLayer_Index;
    //}

    //cout << "max layer num = " << max_Layer_NUM << endl;
    ui->checkBox_EachLayerSwitch->setChecked(true);
    ui->checkBox_onlyShowOnetype_layers->setChecked(true);
    ui->radioButton_initialORsupport->setChecked(true);
    ui->checkBox_draw_LargeISOlayers->setChecked(true);
    //ui->spinBox_ShowLayerIndex->setMaximum(max_Layer_NUM);
    ui->spinBox_ShowLayerIndex->setValue(0);
}

void MainWindow::deSelect_origin() {

    PolygenMesh* supportRaySet = _detectPolygenMesh(SUPPORT_RAY, "Support_Ray",false);
    if (supportRaySet == NULL) {
        std::cout << "There is no supportRaySet PolygenMesh, please check" << std::endl;
        return;
    }

    QMeshPatch* patch_supportRay = (QMeshPatch*)supportRaySet->GetMeshList().GetHead();
    for (GLKPOSITION Pos = patch_supportRay->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* support_Node = (QMeshNode*)patch_supportRay->GetNodeList().GetNext(Pos);

        if(ui->radioButton_deselect_origin->isChecked())
            support_Node->deselect_origin_of_RAY = true;
        else
            support_Node->deselect_origin_of_RAY = false;

    }
}

// MainFunction: build support Surface
void MainWindow::build_SupportRAY() {

    PolygenMesh* tetModel = _detectPolygenMesh(TET, "tet_fabrication", true);
    PolygenMesh* layers = _detectPolygenMesh(CURVED_LAYER, "Layers", false);
    PolygenMesh* platform = _detectPolygenMesh(CAD_PARTS, "Platform", false);
    PolygenMesh* supportRaySet = _buildPolygenMesh(SUPPORT_RAY, "Support_Ray");
    PolygenMesh* tight_supportLayerSet = _buildPolygenMesh(CURVED_LAYER, "Tight_supportLayerSet");
    PolygenMesh* toolpathSet_support = _buildPolygenMesh(TOOL_PATH, "ToolPath_support");
    PolygenMesh* toolpathSet_initial = _buildPolygenMesh(TOOL_PATH, "ToolPath_initial");

    if (layers == NULL || platform == NULL || supportRaySet == NULL)
        std::cout << "There is no needed PolygenMesh, please check" << std::endl;

    supportGene = new supportGeneration();
    supportGene->initial(tetModel, layers, platform, supportRaySet,
        tight_supportLayerSet, toolpathSet_support, toolpathSet_initial,
        (ui->lineEdit_SorceDataDir->text()).toStdString());

    //temp test: marking support region
    if (is_TET_surface_tree_new) {
        supportGene->markSupportFace();
        supportGene->build_Support_Tree_fromTETsurface3();
    }
    else if (is_TET_surface_tree) {
        supportGene->markSupportFace();
        supportGene->build_Support_Tree_fromTETsurface2();
        supportGene->collect_Support_Polyline_fromTETsurface();
    }
    else if (is_polyline) { // new 2: polyline from curved layers
        supportGene->build_Support_PolyLines();
        supportGene->collect_Support_Polyline();
    }
    else if (is_TET_surface_polyline) { // new 3: polyline from tetSurface and vector field
        supportGene->markSupportFace();
        supportGene->build_Support_Polyline_fromTETsurface();
        supportGene->collect_Support_Polyline_fromTETsurface();
    }
    else if (is_TET_surface_Ray) { // new 1: ray from tetSurface and vector field
        supportGene->markSupportFace();
        supportGene->build_SupportRays_vertical();
        supportGene->collect_SupportRays_vertical();
    }
    else { // initial: ray from curved layers
        supportGene->build_SupportRays();
        supportGene->collect_SupportRays();
    }
    std::cout << "### Collected the Support Rays." << std::endl;

    ui->radioButton_showRayOrSurface->setEnabled(true);
    ui->pushButton_buildSupportLayerSet->setEnabled(true);
    ui->checkBox_draw_LargeISOlayers->setEnabled(true);
    ui->radioButton_deselect_origin->setEnabled(true);

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::build_SupportMesh() {

    if (ui->checkBox_verifyByMarching->isChecked()) {
        supportGene->build_SupportSurface_MarchingCube();
        std::cout << "### Gotten the Cut Surface by Marching Cube \n(verify before next step)." << std::endl;
    }
    else {      
        if (is_polyline || is_TET_surface_polyline) {
            supportGene->compute_Support_polyLlne_Field();
        }
        else if (is_TET_surface_tree || is_TET_surface_tree_new) { 
            supportGene->compute_Support_tree_Field();
        }
        else {
            supportGene->compute_supportRayField();
        }
        std::cout << "### Calculated the Field Value of Cut Surface." << std::endl;
        supportGene->build_tight_supportLayers();
        std::cout << "### Gotten the tight support Surfaces." << std::endl;
        ui->radioButton_tightSupportLayerDraw->setEnabled(true);
    }

    pGLK->refresh(true);
    std::cout << "------------------------------------------- Build Support Mesh finished!" << std::endl;
}

void MainWindow::get_CurvedToolpath() {

    supportGene->toolPathCompute_support();
    supportGene->toolPathCompute_initial();

    pGLK->refresh(true);
    std::cout << "------------------------------------------- Build Toolpath finished!" << std::endl;
}

void MainWindow::output_Toolpath() {

    supportGene->output_toolpath();
    pGLK->refresh(true);
    std::cout << "------------------------------------------- Output Toolpath finished!" << std::endl;
}

// function test
void MainWindow::test_func() {

    supportGeneration* supportGene = new supportGeneration();

    ////////////////////////////////////////////////////////
    Eigen::Vector3d intersection_Node;
    Eigen::Vector3d Op = { 0.0,0.0,1.0 };
    Eigen::Vector3d Oo = { 0.0,0.0,-1.0 };
    Eigen::Vector3d Ap = { 1,0.0,2.5 };
    Eigen::Vector3d Aa = { (-sqrt(2.0) / 2.0), 0.0,(-sqrt(2.0) / 2.0) };
    bool is_intersect = supportGene->_LineLineIntersection(intersection_Node, Op, Oo, Ap, Aa);
    if (is_intersect) std::cout << "intersection OK, point is: " << intersection_Node.transpose() <<std::endl;
    else std::cout << "intersection X"<< std::endl;
    return;
    ////////////////////////////////////////////////////////

    Eigen::Vector3d O(0, 0, 50); // Vs
    Eigen::Vector3d T(0, 0, 0); // Ve
    Eigen::Vector3d queryPnt(1, 1, 50);
    double R = 5.0;
    double r_i = 1.2;
    double mu1, mu2;

    bool intersection = supportGene->_lineIntersectSphere(O, T, queryPnt, R, mu1, mu2);
    std::cout << "mu1: " << mu1 << "\nmu2: " << mu2 << std::endl;

    if (intersection == false) std::cout << "1: no intersection Pnt |()" << std::endl;
    if (abs(mu1) > 1.0 && abs(mu2)>1.0) std::cout << "2: no intersection Pnt (|)" << std::endl;

    Eigen::Vector3d iPnt_1 = O + mu1 * (T - O); //p1
    Eigen::Vector3d iPnt_2 = O + mu2 * (T - O); //p2

    std::cout << "iPnt_1: " << iPnt_1.transpose() << "\niPnt_2: " << iPnt_2.transpose() << std::endl;
    

    double s1 = MAX(0, (O - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());
    double s2 = MIN(1, (T - iPnt_1).dot(iPnt_2 - iPnt_1) / (iPnt_2 - iPnt_1).squaredNorm());

    std::cout << "s1: " << s1 << "\ns2: " << s2 << std::endl;
    std::cout << "------------------" << std::endl;

    double l = (iPnt_2 - iPnt_1).norm();
    double a = (queryPnt - iPnt_1).dot(iPnt_2 - iPnt_1);


    double F_queryPnt_eachRay = r_i / 15.0 / pow(R, 4) * (
        (3 * pow(l, 4) * pow(s2, 5) - 15 * a * pow(l, 2) * pow(s2, 4) + 20 * pow(a, 2) * pow(s2, 3))
        - (3 * pow(l, 4) * pow(s1, 5) - 15 * a * pow(l, 2) * pow(s1, 4) + 20 * pow(a, 2) * pow(s1, 3)));

    cout << "F_queryPnt_eachRay = " << F_queryPnt_eachRay << std::endl;

}

void MainWindow::on_pushButton_clearAll_clicked()
{
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        pGLK->DelDisplayObj(polygenMesh);
    }
    polygenMeshList.RemoveAll();
    pGLK->ClearDisplayObjList();
    pGLK->refresh();
    updateTree();
}