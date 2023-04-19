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

#include "fileIO.h"
#include <cstdlib>
#include "meshOperation.h"
#include "IsoLayerGeneration.h"
#include "toolpathGeneration.h"
#include "robotWpGeneration.h"


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
	connect(ui->pushButton_ShowAllLayers, SIGNAL(released()), this, SLOT(all_Display()));
	connect(ui->spinBox_ShowLayerIndex, SIGNAL(valueChanged(int)), this, SLOT(update_Layer_Display()));
    connect(ui->radioButton_compatibleLayer, SIGNAL(released()), this, SLOT(change_maxLayerNum_normalORcompatible()));

    //Buttons: Tianyu Zhang 2023-04-17 for support-generation of curved slicing
    connect(ui->pushButton_readData, SIGNAL(released()), this, SLOT(readData_tetModel_and_scalarField()));
    connect(ui->pushButton_model_positionUpdate, SIGNAL(released()), this, SLOT(update_model_postion_orientation()));
    connect(ui->pushButton_buildEnvelopeCH, SIGNAL(released()), this, SLOT(build_EnvelopeCH()));
    connect(ui->pushButton_remeshCH, SIGNAL(released()), this, SLOT(remeshCH()));
    connect(ui->pushButton_generateSupportSpace, SIGNAL(released()), this, SLOT(generate_supportSpace()));
    connect(ui->pushButton_readSupportSpace, SIGNAL(released()), this, SLOT(readData_supportSpace()));
    connect(ui->pushButton_transferField_2_SupportSpace, SIGNAL(released()), this, SLOT(transferField_2_supportSpace()));
    connect(ui->pushButton_generateCompatibleLayers, SIGNAL(released()), this, SLOT(generate_compatible_layers()));
    connect(ui->pushButton_generate_support_structure, SIGNAL(released()), this, SLOT(generate_support_structure()));
    connect(ui->pushButton_slimmedSupportGeneration, SIGNAL(released()), this, SLOT(extract_slim_supportLayer()));
    connect(ui->pushButton_toolPathGeneration, SIGNAL(released()), this, SLOT(toolPath_Generation()));
    connect(ui->pushButton_Tp4Ur5e, SIGNAL(released()), this, SLOT(UR_robot_waypoint_Generation()));

}

void MainWindow::open()
{
    QString filenameStr = QFileDialog::getOpenFileName(this, tr("Open File,"), "../DataSet/TET_MODEL/", tr(""));
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
        PolygenMesh *polygenMesh = new PolygenMesh(SURFACE_MESH);
        polygenMesh->ImportOBJFile(filename,modelName);
        polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
        pGLK->AddDisplayObj(polygenMesh,true);
        polygenMeshList.AddTail(polygenMesh);
    }

	else if (QString::compare(fileSuffix, "tet") == 0) {
		PolygenMesh *polygenMesh = new PolygenMesh(TET_MODEL);
		std::cout << filename << std::endl;
		std::cout << modelName << std::endl;
		polygenMesh->ImportTETFile(filename, modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);
        this->_update_Position_Orientation_Parameter();
	}

    updateTree();

    shiftToOrigin();
    pGLK->refresh(true);
}

void MainWindow::save()
{
    return;
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
	strcpy(output_filename, "../DataSet/temp/holeSelection/");
	strcat(output_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(output_filename, filetype);

	ofstream nodeSelection(output_filename);
	if (!nodeSelection)
		cerr << "Sorry!We were unable to build the file NodeSelect!\n";
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) { // node set
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
	strcpy(input_filename, "../DataSet/temp/holeSelection/");
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
    printf("--> consider the hole winding \n");
    printf("--> If crashed, please confirm the selected file is updated.\n");
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
        polygenMesh->meshType = TET_MODEL;
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

void MainWindow::on_pushButton_clearAll_clicked(){

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        pGLK->DelDisplayObj(polygenMesh);
    }
    polygenMeshList.RemoveAll();
    pGLK->ClearDisplayObjList();

    pGLK->refresh();
    updateTree();
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

PolygenMesh* MainWindow::_buildPolygenMesh(mesh_type type, std::string name) {

    PolygenMesh* newMesh = new PolygenMesh(type);
    newMesh->setModelName(name);
    newMesh->BuildGLList(newMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(newMesh, true);
    polygenMeshList.AddTail(newMesh);
    updateTree();
    return newMesh;

}

PolygenMesh* MainWindow::_detectPolygenMesh(mesh_type type) {

    PolygenMesh* detectedMesh = NULL;
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (thispolygenMesh->meshType == type) {
            detectedMesh = thispolygenMesh; break;
        }
    }
    return detectedMesh;
}

QMeshPatch* MainWindow::_detectPolygenMesh(mesh_type type, std::string patch_name) {

    PolygenMesh* detectedMesh = NULL;
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

        if (thispolygenMesh->meshType == type) {
            detectedMesh = thispolygenMesh;
            break;
        }
    }

    if (detectedMesh == NULL) return NULL;

    QMeshPatch* detectedPatch = NULL;
    for (GLKPOSITION posMesh = detectedMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* thisPatch = (QMeshPatch*)detectedMesh->GetMeshList().GetNext(posMesh);

        if (thisPatch->patchName == patch_name) {
            detectedPatch = thisPatch;
            break;
        }
    }
    return detectedPatch;
}

void MainWindow::update_Layer_Display() {

    bool single = ui->checkBox_EachLayerSwitch->isChecked();
    int currentLayerIndex = ui->spinBox_ShowLayerIndex->value();

    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (polygenMesh->meshType != CURVED_LAYER
            && polygenMesh->meshType != TOOL_PATH) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);

            Patch->drawThisPatch = false;

            if (ui->radioButton_compatibleLayer->isChecked()) {
                if (single) {
                    if (Patch->compatible_layer_Index == currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
                else {
                    if (Patch->compatible_layer_Index <= currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
            }
            else {

                if (single) {
                    if (Patch->GetIndexNo() == currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
                else {
                    if (Patch->GetIndexNo() <= currentLayerIndex)
                        Patch->drawThisPatch = true;
                }
            }
        }
    }

    pGLK->refresh(true);
}

//change the maxLayer number
void MainWindow::change_maxLayerNum_normalORcompatible() {

    PolygenMesh* compatible_isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (compatible_isoLayerSet == nullptr) { std::cerr << "No compatible layers is detected!" << std::endl; return; }

    int max_compatible_index = -1;
    for (GLKPOSITION posMesh = compatible_isoLayerSet->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshPatch* each_patch = (QMeshPatch*)compatible_isoLayerSet->GetMeshList().GetNext(posMesh);

        if (each_patch->compatible_layer_Index > max_compatible_index)
            max_compatible_index = each_patch->compatible_layer_Index;
    }

    if (ui->radioButton_compatibleLayer->isChecked()) {
        ui->spinBox_ShowLayerIndex->setMaximum(max_compatible_index);
    }
    else {
        ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);
    }
}

void MainWindow::all_Display() {
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);

        if (polygenMesh->meshType != CURVED_LAYER
            && polygenMesh->meshType != TOOL_PATH) continue;

        for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
            QMeshPatch* Patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
            Patch->drawThisPatch = true;
        }
    }
    pGLK->refresh(true);
}

void MainWindow::_update_Position_Orientation_Parameter() {

    std::string modelName = this->_detectPolygenMesh(TET_MODEL)->getModelName();

    if (modelName == "topopt") { 
        this->_setParameter_4_Rot_and_Mov(0, 20, 0, 0, 0, -140); //x y z rx ry rz
        ui->spinBox_isoLayerNumber->setValue(100);
    }
    else if (modelName == "bridge") {
        this->_setParameter_4_Rot_and_Mov(0, 100, 0, 180, 0, 0);
        ui->spinBox_isoLayerNumber->setValue(200);
    }
    else if (modelName == "connector2") {
        this->_setParameter_4_Rot_and_Mov(0, 40, 0, 45, 0, 0);
        ui->spinBox_isoLayerNumber->setValue(50);
    }
    else if (modelName == "dome") {
        this->_setParameter_4_Rot_and_Mov(0, 3.0, 0, 0, 0, 0);
        ui->spinBox_isoLayerNumber->setValue(80);
    }
    else {
        this->_setParameter_4_Rot_and_Mov(0, 0, 0, 0, 0, 0);
        ui->spinBox_isoLayerNumber->setValue(50);
    }

    ui->doubleSpinBox_offset_dist->setValue(3.0);
}

void MainWindow::_setParameter_4_Rot_and_Mov(
    double Xmove, double Ymove, double Zmove,
    double Xrot, double Yrot, double Zrot) {

    ui->doubleSpinBox_Xmove->setValue(Xmove);
    ui->doubleSpinBox_Ymove->setValue(Ymove);
    ui->doubleSpinBox_Zmove->setValue(Zmove);
    ui->doubleSpinBox_XRot->setValue(Xrot);
    ui->doubleSpinBox_YRot->setValue(Yrot);
    ui->doubleSpinBox_ZRot->setValue(Zrot);
}

//MainFunctions for support generation of curved pringting
void MainWindow::readData_tetModel_and_scalarField() {

    std::string modelName = ui->lineEdit_SorceDataDir->text().toStdString();

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == NULL) {

        char filename[1024];
        sprintf(filename, "%s%s%s", "../DataSet/TET_MODEL/", modelName.c_str(), ".tet");
        cout << "input " << modelName << " from: " << filename << endl;

        tetModel = this->_buildPolygenMesh(TET_MODEL, modelName);
        //read tet model
        tetModel->ImportTETFile(filename, modelName);
        this->_update_Position_Orientation_Parameter();
    }
    else {
        tetModel->ClearAll();
        std::cout << "There is already existing a tet PolygenMesh, please check!" << std::endl;
        return;
    }

    // read scalar field
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();
    model->patchName = modelName;
    fileIO* IO_operator = new fileIO();
    std::string input_dir = "../DataSet/temp/scalarField/" + modelName + "_scalarField.txt";
    IO_operator->input_scalar_field(model, input_dir);
    delete IO_operator;

    // show the geometry info
    double boundingBox[6];    model->ComputeBoundingBox(boundingBox);
    std::cout << "\nThe Range of model:\nX:" << boundingBox[1]- boundingBox[0] << "\tY: "<<
        boundingBox[3] - boundingBox[2] << "\tZ: " <<
        boundingBox[5] - boundingBox[4] << std::endl;
    std::cout << "\nThe Center of model:\nX:" << (boundingBox[1] + boundingBox[0])/2.0 << "\tY:" <<
        (boundingBox[3] + boundingBox[2])/2.0 << "\tZ: " <<
        (boundingBox[5] + boundingBox[4])/2.0 << std::endl;
    std::cout << "\nThe bounding Box of model:\nXmin: " << boundingBox[0] << "\t\tXmax: " << boundingBox[1] << "\n" <<
        "Ymin : " << boundingBox[2] << "\t\tYmax : " << boundingBox[3] << "\n" <<
        "Zmin : " << boundingBox[4] << "\t\tZmax : " << boundingBox[5] << std::endl;

    std::cout << "\nFinish readData_tetModel_and_scalarField.\n" << std::endl;

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::update_model_postion_orientation() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    double xMove = ui->doubleSpinBox_Xmove->value();
    double yMove = ui->doubleSpinBox_Ymove->value();
    double zMove = ui->doubleSpinBox_Zmove->value();
    //double Phi = ui->doubleSpinBox_rot_phi->value();
    //double Theta = ui->doubleSpinBox_rot_theta->value();
    double xRot = ui->doubleSpinBox_XRot->value();
    double yRot = ui->doubleSpinBox_YRot->value();
    double zRot = ui->doubleSpinBox_ZRot->value();

    this->_rot_And_Move_Model(model, xRot, yRot, zRot, xMove, yMove, zMove, false);
    this->_modify_scalarField_order(model);

    ui->pushButton_readSupportSpace->setEnabled(true);
    ui->pushButton_generateSupportSpace->setEnabled(true);
    ui->pushButton_generate_support_structure->setEnabled(true);

    std::cout << "\nFinish update_model_postion_orientation.\n" << std::endl;

    pGLK->Zoom_All_in_View();
    pGLK->refresh(true);
}

//Euler rotation operation
void MainWindow::_rot_And_Move_Model(QMeshPatch* m_tetModel, double xRot, double yRot, double zRot,
    double xMove, double yMove, double zMove, bool isUpdate_lastCoord3D) {

    //rotate model
    double pitch = DEGREE_TO_ROTATE(xRot);
    double yaw = DEGREE_TO_ROTATE(yRot);
    double roll = DEGREE_TO_ROTATE(zRot);

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q =  pitchAngle * yawAngle * rollAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    m_tetModel->model_rotMat = rotationMatrix;

    for (GLKPOSITION posMesh = m_tetModel->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
        QMeshNode* node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(posMesh);

        Eigen::Vector3d pp; node->GetCoord3D_last(pp(0), pp(1), pp(2));
        Eigen::Vector3d rotatedpp = rotationMatrix * pp;

        node->SetCoord3D(rotatedpp[0], rotatedpp[1], rotatedpp[2]);

        //std::cout << "pp \t" << pp.transpose() << "\n";
        //std::cout << "rotatedpp \t" << rotatedpp.transpose() << "\n\n";
        //std::cout << "pp -  rotatedpp \t" << pp.transpose() - rotatedpp.transpose() << "\n";
    }

    std::cout << "\n-----------\nRotationMatrix:\n" << rotationMatrix << std::endl;
    std::cout << "\nxRot: " << xRot << "\tyRot: " << yRot << "\tzRot: " << zRot;
    std::cout << "\nFinish rotate model." << std::endl;

    //move model
    double pp[3];
    for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);
        Node->GetCoord3D(pp[0], pp[1], pp[2]);

        pp[0] += xMove; pp[1] += yMove; pp[2] += zMove;
        Node->SetCoord3D(pp[0], pp[1], pp[2]);
        if (isUpdate_lastCoord3D)   Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
    }
    std::cout << "\nxMove: " << xMove << "\tyMove: " << yMove << "\tzMove: " << zMove;
    std::cout << "\nFinish move model.\n" << std::endl;
}

void MainWindow::_modify_scalarField_order(QMeshPatch* m_tetModel) {

    double minHeight = 999999999.9;		double scalarValue_minHeight = 0.0;
    double maxHeight = -999999999.9;	double scalarValue_maxHeight = 0.0;
    double pp[3];
    for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);

        Node->GetCoord3D(pp[0], pp[1], pp[2]);
        if (pp[1] > maxHeight) {
            maxHeight = pp[1]; scalarValue_maxHeight = Node->scalarField;
        }
        if (pp[1] < minHeight) {
            minHeight = pp[1]; scalarValue_minHeight = Node->scalarField;
        }
    }

    // inverse scalar field
    if (scalarValue_minHeight > scalarValue_maxHeight) {
        for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
            QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);
            Node->scalarField = 1.0 - Node->scalarField;

            if (Node->scalarField < 0.0) Node->scalarField = 0.0;
            if (Node->scalarField > 1.0) Node->scalarField = 1.0;
        }
        std::cout << "\nchange scalar_order.\n-----------\n" << std::endl;
    }

    std::cout << "\nFinish _modify_scalarField_order.\n-----------\n" << std::endl;
}

void MainWindow::build_EnvelopeCH() {

    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    //platform
    PolygenMesh* platform = this->_detectPolygenMesh(CNC_PRT);
    if (platform == NULL) {
        platform = this->_buildPolygenMesh(CNC_PRT, "platform");

        QMeshPatch* platform_patch = new QMeshPatch;
        platform_patch->patchName = tetModel->getModelName() + "_platform";
        platform_patch->SetIndexNo(platform->GetMeshList().GetCount());
        platform->GetMeshList().AddTail(platform_patch);

        std::string inputFile = "../DataSet/temp/platform.obj";
        platform_patch->inputOBJFile((char*)inputFile.c_str(), false);
        //std::cout << "There are " << platform->GetMeshList().GetCount() << " patch(s) in the PolygenMesh\n";
    }
    QMeshPatch* platform_patch = (QMeshPatch*)platform->GetMeshList().GetHead();

    PolygenMesh* supportEnvelope = _buildPolygenMesh(CH_ENVELOPE, "Support_Envelope");
    PolygenMesh* supportRay = _buildPolygenMesh(SUPPORT_RAY, "Support_Ray");

    if (tetModel == NULL || platform_patch == NULL || supportEnvelope == NULL || supportRay == NULL)
        std::cout << "There is no needed PolygenMesh, please check" << std::endl;

    supportOperator = new supportGene();
    supportOperator->initial_4_envelope(model, platform_patch, supportEnvelope, supportRay,
        45.0, ui->doubleSpinBox_offset_dist->value());
    supportOperator->initial_Guess_SupportEnvelope();           //get initial support space
    supportOperator->compute_initialGuess_EnvelopeHull();       //build convex hull of initial guess
    delete supportOperator;
    std::cout << "\nFinish building the convex envelope for the tetmesh." << std::endl;

    fileIO* IO_operator = new fileIO();
    QMeshPatch* convexHull = (QMeshPatch*)supportEnvelope->GetMeshList().GetHead();
    convexHull->patchName = tetModel->getModelName() + "_envelope_CH";
    std::string output_dir = "../DataSet/temp/remesh/ch_envelope/input/";
    IO_operator->remove_allFile_in_Dir(output_dir);
    IO_operator->output_OBJ_Mesh(convexHull, output_dir);
    std::cout << "Finish outputing box mesh of the envelope of tet model." << std::endl;
    delete IO_operator;

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::remeshCH() {

    std::string path = "../DataSet/temp/remesh/ch_envelope/remesh.py";
    std::cout << "\n******************************\nDouble click remesh.py to remesh: \n" 
        << path << "\n******************************\n" << std::endl;
    //system("C:/PHD/Code/SupportGeneration/DataSet/temp/remesh/remesh_cube.bat");
}

void MainWindow::generate_supportSpace() {

    //1. obtain tet model
    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }

    //2. input remeshed envelope mesh
    PolygenMesh* ch_envelope = this->_detectPolygenMesh(CH_ENVELOPE);
    if (ch_envelope == NULL) {

        std::string ch_Name = tetModel->getModelName() + "_envelope_CH";
        ch_envelope = this->_buildPolygenMesh(CH_ENVELOPE, ch_Name);
    }
    QMeshPatch* cubeEnvelopeRemeshed_patch = new QMeshPatch;
    cubeEnvelopeRemeshed_patch->patchName = tetModel->getModelName() + "_envelope_CH_remeshed";
    cubeEnvelopeRemeshed_patch->SetIndexNo(ch_envelope->GetMeshList().GetCount());
    ch_envelope->GetMeshList().AddTail(cubeEnvelopeRemeshed_patch);

    std::string inputFile = "../DataSet/temp/remesh/ch_envelope/output/" + tetModel->getModelName() + "_envelope_CH.obj";
    cubeEnvelopeRemeshed_patch->inputOBJFile((char*)inputFile.c_str(), false);
    std::cout << "There are " << ch_envelope->GetMeshList().GetCount() << " patch(s) in the PolygenMesh\n";

    //3. combine the boundary of tet model(*.obj) and box mesh(*.obj)
    //4. generate the tet mesh of boxEnvelope containing boundary of tet mesh
    //5. extract support space by Boolean operation
    //6. output the support space

    QMeshPatch* ch_remeshed = 
        this->_detectPolygenMesh(CH_ENVELOPE, (tetModel->getModelName() + "_envelope_CH_remeshed"));
    if (ch_remeshed == NULL) { std::cerr << "No tet remeshed box is detected!" << std::endl; return; }
    QMeshPatch* model = (QMeshPatch*)tetModel->GetMeshList().GetHead();
    meshOperation* meshOperator = new meshOperation;
    meshOperator->tetMeshGeneration_extract_SupportSpace(ch_remeshed, model);
    delete meshOperator;

    std::cout << "\nFinish generate_supportSpace.\n" << std::endl;

    updateTree();
    pGLK->refresh(true);
}

void MainWindow::readData_supportSpace() {

    std::string modelName = ui->lineEdit_SorceDataDir->text().toStdString();
    modelName += "_supportSpace";

    PolygenMesh* tetModel_supportSpace = this->_detectPolygenMesh(SUPPORT_TET_MODEL);
    if (tetModel_supportSpace == NULL) {

        char filename[1024];
        sprintf(filename, "%s%s%s", "../DataSet/TET_MODEL/", modelName.c_str(), ".tet");
        cout << "input " << modelName << " from: " << filename << endl;

        tetModel_supportSpace = this->_buildPolygenMesh(SUPPORT_TET_MODEL, modelName);
        tetModel_supportSpace->ImportTETFile(filename, modelName);
    }
    else {
        tetModel_supportSpace->ClearAll();
        std::cout << "There is already existing a tet PolygenMesh, please check!" << std::endl;
        return;
    }

    QMeshPatch* model = (QMeshPatch*)tetModel_supportSpace->GetMeshList().GetHead();
    model->patchName = modelName;

    std::cout << "\nFinish readData_supportSpace.\n" << std::endl;

    pGLK->refresh(true);
}

void MainWindow::transferField_2_supportSpace() {

    //1. obtain tet model
    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* tet_model = (QMeshPatch*)tetModel->GetMeshList().GetHead();
    
    PolygenMesh* tetModel_supportSpace = this->_detectPolygenMesh(SUPPORT_TET_MODEL);
    if (tetModel_supportSpace == nullptr) { std::cerr << "No tet mesh(support) is detected!" << std::endl; return; }
    QMeshPatch* tet_sup_model = (QMeshPatch*)tetModel_supportSpace->GetMeshList().GetHead();

    //2. generate vector/scalar for supportSpace
    supportOperator = new supportGene;
    supportOperator->initial_4_supportSpace_Extraction(tet_model, tet_sup_model);
    //method 1: supportOperator->generate_field_4_tetMeshes();
    //method 2: supportOperator->generate_field_4_tetMeshes_correction();
    supportOperator->generate_field_4_tetMeshes_correction();
    std::cout << "\nFinish transferField_2_supportSpace.\n" << std::endl;

    pGLK->refresh(true);
}

void MainWindow::generate_compatible_layers() {

    //1. obtain tet model
    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* tet_model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    PolygenMesh* tetModel_supportSpace = this->_detectPolygenMesh(SUPPORT_TET_MODEL);
    if (tetModel_supportSpace == nullptr) { std::cerr << "No tet mesh(support) is detected!" << std::endl; return; }
    QMeshPatch* tet_sup_model = (QMeshPatch*)tetModel_supportSpace->GetMeshList().GetHead();

    //generate compatible layers
    std::string modelName = tet_model->patchName + "_isoLayers";
    PolygenMesh* compatible_isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, modelName);
    IsoLayerGeneration* slicer = new IsoLayerGeneration(tet_model);
    slicer->generateIsoSurface(compatible_isoLayerSet, ui->spinBox_isoLayerNumber->value());
    slicer->generateIsoSurface_support(
        compatible_isoLayerSet, tet_sup_model, ui->spinBox_isoLayerNumber->value());
    delete slicer;

    ui->spinBox_ShowLayerIndex->setMinimum((int)0);
    ui->radioButton_compatibleLayer->setEnabled(true);
    ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);

    //organize the compatible layers
    supportOperator->organize_compatibleLayer_Index(compatible_isoLayerSet,
        compatible_isoLayerSet->GetMeshList().GetCount() - ui->spinBox_isoLayerNumber->value());

    fileIO* IO_operator = new fileIO();
    std::string output_dir = "../DataSet/temp/remesh/compatible_layers/input/";
    IO_operator->remove_allFile_in_Dir(output_dir);
    IO_operator->output_compatibleLayer_4_remesh(compatible_isoLayerSet, output_dir);
    delete IO_operator;
    delete supportOperator;
    std::cout << "\nFinish generate_compatible_layers. Please remesh the mesh.\n" << std::endl;

    pGLK->refresh(true);
}

void MainWindow::generate_support_structure() {

    supportOperator = new supportGene;

    //tet model
    PolygenMesh* tetModel = this->_detectPolygenMesh(TET_MODEL);
    if (tetModel == nullptr) { std::cerr << "No tet mesh is detected!" << std::endl; return; }
    QMeshPatch* tet_model = (QMeshPatch*)tetModel->GetMeshList().GetHead();

    //iso-layers
    std::string modelName = tet_model->patchName + "_isoLayers";
    PolygenMesh* compatible_isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (compatible_isoLayerSet == NULL) {
        compatible_isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, modelName);
    }
    else {
        compatible_isoLayerSet->ClearAll();
        std::cout << "There is already existing a compatible_isoLayerSet PolygenMesh, it will be reconstructed!" << std::endl;
    }

    //platform
    PolygenMesh* platform = this->_detectPolygenMesh(CNC_PRT);
    if (platform == NULL) {
        platform = this->_buildPolygenMesh(CNC_PRT, "platform");

        QMeshPatch* platform_patch = new QMeshPatch;
        platform_patch->patchName = tetModel->getModelName() + "_platform";
        platform_patch->SetIndexNo(platform->GetMeshList().GetCount());
        platform->GetMeshList().AddTail(platform_patch);

        std::string inputFile = "../DataSet/temp/platform.obj";
        platform_patch->inputOBJFile((char*)inputFile.c_str(), false);
        //std::cout << "There are " << platform->GetMeshList().GetCount() << " patch(s) in the PolygenMesh\n";
    }
    else {
        platform->ClearAll();
        std::cout << "There is already existing a platform PolygenMesh, please check!" << std::endl;
        return;
    }

    //support tree container
    modelName = tet_model->patchName + "_supportRay";
    PolygenMesh* tetModel_supportRay = this->_detectPolygenMesh(SUPPORT_RAY);
    if (tetModel_supportRay == NULL) {
        tetModel_supportRay = this->_buildPolygenMesh(SUPPORT_RAY, modelName);
    }
    else {
        tetModel_supportRay->ClearAll();
        std::cout << "There is already existing a support Ray PolygenMesh, please check!" << std::endl;
        return;
    }

    //modify the size of platform
    double boundingBox[6];    tet_model->ComputeBoundingBox(boundingBox);
    Eigen::Vector4d boundingBox_vector;
    boundingBox_vector << fabs(boundingBox[0]), fabs(boundingBox[1]), fabs(boundingBox[4]), fabs(boundingBox[5]);

    QMeshPatch* platform_patch = this->_detectPolygenMesh(CNC_PRT, tetModel->getModelName() + "_platform");
    if (platform == NULL) { std::cerr << "No platform is detected!" << std::endl; return; }
    platform_patch->ComputeBoundingBox(boundingBox);
    double xmax_platform = boundingBox[1];
    for (GLKPOSITION Pos = platform_patch->GetNodeList().GetHeadPosition(); Pos;) {
        QMeshNode* Node = (QMeshNode*)platform_patch->GetNodeList().GetNext(Pos);

        double xx, yy, zz;
        Node->GetCoord3D(xx, yy, zz);

        
        xx *= boundingBox_vector.maxCoeff() / xmax_platform * 1.2;
        zz *= boundingBox_vector.maxCoeff() / xmax_platform * 1.2;

        Node->SetCoord3D(xx, yy, zz);
    }

    //read the remeshed layers
    fileIO* IO_operator = new fileIO();
    std::string path = "../DataSet/temp/remesh/compatible_layers/output";
    IO_operator->input_remeshed_compatibleLayer(compatible_isoLayerSet, path);
    delete IO_operator;

    double detect_overhang_angle;
    if (tetModel->getModelName() == "dome") detect_overhang_angle = 50;
    else detect_overhang_angle = 45;

    supportOperator->initial_4_treeGeneration(tet_model, 
        compatible_isoLayerSet, platform_patch, tetModel_supportRay, detect_overhang_angle);
    supportOperator->generate_support_tree();
    
    //update the display
    ui->spinBox_ShowLayerIndex->setMinimum((int)0);
    ui->radioButton_compatibleLayer->setEnabled(true);
    ui->radioButton_compatibleLayer->setChecked(false);
    ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);

    std::cout << "\nFinish generate_support_structure.\n" << std::endl;

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
}

void MainWindow::extract_slim_supportLayer() {

    supportOperator->compute_Support_tree_Field();
    std::cout << "Finish the implicit field calculation." << std::endl;
    supportOperator->build_tight_supportLayers();
    std::cout << "Finish the tight support layer generation." << std::endl;
    delete supportOperator;

    PolygenMesh* layerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (layerSet == NULL) {
        std::cerr << "There is no layer set, please check." << std::endl; return;
    }
    fileIO* IO_operator = new fileIO();
    std::string path = "../DataSet/temp/remesh/slimed_layers/input/";
    IO_operator->remove_allFile_in_Dir(path);
    IO_operator->outputIsoSurfaceSet(layerSet, true, path, true);
    delete IO_operator;

    ui->spinBox_ShowLayerIndex->setMaximum(layerSet->GetMeshList().GetCount() - 1);

    std::cout << "\nFinish slim support layer generation. please remesh them.\n" << std::endl;

    pGLK->refresh(true);
}

void MainWindow::toolPath_Generation() {

    PolygenMesh* compatible_isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (compatible_isoLayerSet == NULL) {
        compatible_isoLayerSet = _buildPolygenMesh(CURVED_LAYER, "compatible_isoLayers");
    }
    else {
        compatible_isoLayerSet->ClearAll();
        //polygenMeshList.Remove(supportModelSet);
        std::cout << "There is already existing a compatible_isoLayers PolygenMesh, it has been reconstructed!" << std::endl;
    }

    fileIO* IO_operator = new fileIO();
    std::string path = "../DataSet/temp/remesh/slimed_layers/output";
    IO_operator->input_remeshed_init_and_slimSupport_layers(compatible_isoLayerSet, path);
    

    PolygenMesh* toolpathSet = this->_buildPolygenMesh(TOOL_PATH, "toolPath");
    toolpathGeneration* ToolPathComp_layer = new toolpathGeneration(compatible_isoLayerSet, toolpathSet,
        ui->doubleSpinBox_toolPathWidth->value(), ui->doubleSpinBox_toolPathDistance->value());

    ToolPathComp_layer->generate_all_toolPath();
    delete ToolPathComp_layer;


    path = "../DataSet/temp/remesh/slimed_layers/output";
    IO_operator->output_toolpath(toolpathSet, true, false);
    delete IO_operator;

    ui->radioButton_compatibleLayer->setEnabled(true);
    ui->spinBox_ShowLayerIndex->setMaximum(compatible_isoLayerSet->GetMeshList().GetCount() - 1);
    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "Finish generating toolpath in the materialSpace.\n" << std::endl;
}

void MainWindow::UR_robot_waypoint_Generation() {

    this->on_pushButton_clearAll_clicked();

    PolygenMesh* isoLayerSet = this->_detectPolygenMesh(CURVED_LAYER);
    if (isoLayerSet == NULL) {
        isoLayerSet = this->_buildPolygenMesh(CURVED_LAYER, "IsoLayer");
    }
    else {
        isoLayerSet->ClearAll();
        std::cout << "There is already existing a isoLayerSet PolygenMesh, it will be reconstructed!" << std::endl;
    }

    PolygenMesh* toolpathSet = this->_detectPolygenMesh(TOOL_PATH);
    if (toolpathSet == NULL) {
        toolpathSet = this->_buildPolygenMesh(TOOL_PATH, "Toolpath");
    }
    else {
        toolpathSet->ClearAll();
        std::cout << "There is already existing a toolpathSet PolygenMesh, it will be reconstructed!" << std::endl;
    }

    fileIO* IO_operator = new fileIO();
    std::string FileDir = "../DataSet/temp/robotWpGeneration/";
    int layerNum = IO_operator->read_layer_toolpath_files(
        isoLayerSet, toolpathSet, FileDir);
    ui->spinBox_ShowLayerIndex->setMaximum(layerNum - 1);

    //platform
    PolygenMesh* platform = this->_detectPolygenMesh(CNC_PRT);
    if (platform == NULL) {
        platform = this->_buildPolygenMesh(CNC_PRT, "platform");

        QMeshPatch* platform_patch = new QMeshPatch;
        platform_patch->patchName = "patch_platform";
        platform_patch->SetIndexNo(platform->GetMeshList().GetCount());
        platform->GetMeshList().AddTail(platform_patch);

        std::string inputFile = "../DataSet/temp/platform_zup.obj";
        platform_patch->inputOBJFile((char*)inputFile.c_str(), false);

        //std::cout << "platform_patch->drawThisPatch = " << platform_patch->drawThisPatch << std::endl;
        //std::cout << "There are " << platform->GetMeshList().GetCount() << " patch(s) in the PolygenMesh\n";
    }
    else {
        std::cout << "There is already existing a platform PolygenMesh, please check!" << std::endl;
        return;
    }

    robotWpGeneration* rob_wp = new robotWpGeneration();
    rob_wp->initial(isoLayerSet, toolpathSet, platform, ui->doubleSpinBox_toolPathWidth->value());
    rob_wp->calDHW();
    rob_wp->modify_wp_normal(false);
    delete rob_wp; 

    FileDir = "../DataSet/temp/robotWpGeneration/toolpath_robot_ur/";
    IO_operator->output_toolpath_UR5e(toolpathSet, FileDir);
    delete IO_operator;

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();
    std::cout << "Finish generating waypoints for UR5e robot.\n" << std::endl;

}