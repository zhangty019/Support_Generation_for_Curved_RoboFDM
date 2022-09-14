/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionFront;
    QAction *actionBack;
    QAction *actionTop;
    QAction *actionBottom;
    QAction *actionLeft;
    QAction *actionRight;
    QAction *actionIsometric;
    QAction *actionZoom_In;
    QAction *actionZoom_Out;
    QAction *actionZoom_All;
    QAction *actionZoom_Window;
    QAction *actionShade;
    QAction *actionMesh;
    QAction *actionNode;
    QAction *actionSave;
    QAction *actionSelectNode;
    QAction *actionSelectFace;
    QAction *actionShifttoOrigin;
    QAction *actionProfile;
    QAction *actionFaceNormal;
    QAction *actionNodeNormal;
    QAction *actionSelectEdge;
    QAction *actionGenerate;
    QAction *actionTest_1;
    QAction *actionSelectFix;
    QAction *actionSelectHandle;
    QAction *actionSaveSelection;
    QAction *actionReadSelection;
    QAction *actionSelectChamber;
    QAction *actionExport_to_Abaqus_model;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QToolBar *navigationToolBar;
    QStatusBar *statusBar;
    QToolBar *selectionToolBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QLabel *label_MANY_3DP_CNC_CAM;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_PosNorFile;
    QLineEdit *lineEdit_SorceDataDir;
    QFrame *line;
    QLabel *label_5;
    QPushButton *pushButton_Comp_initialGuess_envelopSupport;
    QFrame *line_6;
    QFrame *line_7;
    QLabel *label;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_Xmove;
    QDoubleSpinBox *doubleSpinBox_Xmove;
    QLabel *label_Ymove;
    QDoubleSpinBox *doubleSpinBox_Ymove;
    QLabel *label_Zmove;
    QDoubleSpinBox *doubleSpinBox_Zmove;
    QPushButton *pushButton_readGcodeSourceData;
    QFrame *line_4;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QSpinBox *spinBox_ShowLayerIndex;
    QCheckBox *checkBox_EachLayerSwitch;
    QPushButton *pushButton_ShowAllLayers;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_4;
    QHBoxLayout *horizontalLayout_5;
    QCheckBox *checkBox_onlyShowOnetype_layers;
    QRadioButton *radioButton_initialORsupport;
    QCheckBox *checkBox_draw_LargeISOlayers;
    QFrame *line_5;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *pushButton_buildSupportRaySet;
    QRadioButton *radioButton_deselect_origin;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_buildSupportLayerSet;
    QCheckBox *checkBox_verifyByMarching;
    QRadioButton *radioButton_showRayOrSurface;
    QFrame *line_3;
    QHBoxLayout *horizontalLayout_3;
    QRadioButton *radioButton_tightSupportLayerDraw;
    QCheckBox *boxDeselect;
    QFrame *line_2;
    QPushButton *pushButton_buildSupportToolpathSet;
    QPushButton *pushButton_output_Toolpath;
    QTreeView *treeView;
    QPushButton *pushButton_TestFunc;
    QPushButton *pushButton_readRobot_model;
    QPushButton *pushButton_clearAll;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuSelect;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1331, 1070);
        MainWindow->setMinimumSize(QSize(0, 0));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        MainWindow->setFont(font);
        MainWindow->setMouseTracking(true);
        MainWindow->setFocusPolicy(Qt::StrongFocus);
        MainWindow->setAcceptDrops(true);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resource/Open Folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon);
        actionFront = new QAction(MainWindow);
        actionFront->setObjectName(QString::fromUtf8("actionFront"));
        actionFront->setCheckable(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resource/Front View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFront->setIcon(icon1);
        actionBack = new QAction(MainWindow);
        actionBack->setObjectName(QString::fromUtf8("actionBack"));
        actionBack->setCheckable(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resource/Back View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBack->setIcon(icon2);
        actionTop = new QAction(MainWindow);
        actionTop->setObjectName(QString::fromUtf8("actionTop"));
        actionTop->setCheckable(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resource/Top View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionTop->setIcon(icon3);
        actionBottom = new QAction(MainWindow);
        actionBottom->setObjectName(QString::fromUtf8("actionBottom"));
        actionBottom->setCheckable(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resource/Bottom View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBottom->setIcon(icon4);
        actionLeft = new QAction(MainWindow);
        actionLeft->setObjectName(QString::fromUtf8("actionLeft"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resource/Left View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLeft->setIcon(icon5);
        actionRight = new QAction(MainWindow);
        actionRight->setObjectName(QString::fromUtf8("actionRight"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/resource/Right View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRight->setIcon(icon6);
        actionIsometric = new QAction(MainWindow);
        actionIsometric->setObjectName(QString::fromUtf8("actionIsometric"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/resource/Isometric View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionIsometric->setIcon(icon7);
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName(QString::fromUtf8("actionZoom_In"));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/resource/Zoom In.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_In->setIcon(icon8);
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName(QString::fromUtf8("actionZoom_Out"));
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/resource/Zoom Out.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Out->setIcon(icon9);
        actionZoom_All = new QAction(MainWindow);
        actionZoom_All->setObjectName(QString::fromUtf8("actionZoom_All"));
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/resource/Zoom All.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_All->setIcon(icon10);
        actionZoom_Window = new QAction(MainWindow);
        actionZoom_Window->setObjectName(QString::fromUtf8("actionZoom_Window"));
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/resource/Zoom Window.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Window->setIcon(icon11);
        actionShade = new QAction(MainWindow);
        actionShade->setObjectName(QString::fromUtf8("actionShade"));
        actionShade->setCheckable(true);
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/resource/Shade.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShade->setIcon(icon12);
        actionMesh = new QAction(MainWindow);
        actionMesh->setObjectName(QString::fromUtf8("actionMesh"));
        actionMesh->setCheckable(true);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/resource/Mesh.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMesh->setIcon(icon13);
        actionNode = new QAction(MainWindow);
        actionNode->setObjectName(QString::fromUtf8("actionNode"));
        actionNode->setCheckable(true);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/resource/Node.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNode->setIcon(icon14);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/resource/Save as.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon15);
        actionSelectNode = new QAction(MainWindow);
        actionSelectNode->setObjectName(QString::fromUtf8("actionSelectNode"));
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/resource/selectNode.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectNode->setIcon(icon16);
        actionSelectFace = new QAction(MainWindow);
        actionSelectFace->setObjectName(QString::fromUtf8("actionSelectFace"));
        QIcon icon17;
        icon17.addFile(QString::fromUtf8(":/resource/selectFace.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFace->setIcon(icon17);
        actionShifttoOrigin = new QAction(MainWindow);
        actionShifttoOrigin->setObjectName(QString::fromUtf8("actionShifttoOrigin"));
        actionProfile = new QAction(MainWindow);
        actionProfile->setObjectName(QString::fromUtf8("actionProfile"));
        actionProfile->setCheckable(true);
        QIcon icon18;
        icon18.addFile(QString::fromUtf8(":/resource/Profile.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionProfile->setIcon(icon18);
        actionFaceNormal = new QAction(MainWindow);
        actionFaceNormal->setObjectName(QString::fromUtf8("actionFaceNormal"));
        actionFaceNormal->setCheckable(true);
        QIcon icon19;
        icon19.addFile(QString::fromUtf8(":/resource/FaceNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFaceNormal->setIcon(icon19);
        actionNodeNormal = new QAction(MainWindow);
        actionNodeNormal->setObjectName(QString::fromUtf8("actionNodeNormal"));
        actionNodeNormal->setCheckable(true);
        QIcon icon20;
        icon20.addFile(QString::fromUtf8(":/resource/NodeNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNodeNormal->setIcon(icon20);
        actionSelectEdge = new QAction(MainWindow);
        actionSelectEdge->setObjectName(QString::fromUtf8("actionSelectEdge"));
        QIcon icon21;
        icon21.addFile(QString::fromUtf8(":/resource/selectEdge.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectEdge->setIcon(icon21);
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QString::fromUtf8("actionGenerate"));
        actionTest_1 = new QAction(MainWindow);
        actionTest_1->setObjectName(QString::fromUtf8("actionTest_1"));
        actionSelectFix = new QAction(MainWindow);
        actionSelectFix->setObjectName(QString::fromUtf8("actionSelectFix"));
        QIcon icon22;
        icon22.addFile(QString::fromUtf8(":/resource/selectFix.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFix->setIcon(icon22);
        actionSelectHandle = new QAction(MainWindow);
        actionSelectHandle->setObjectName(QString::fromUtf8("actionSelectHandle"));
        QIcon icon23;
        icon23.addFile(QString::fromUtf8(":/resource/selectHandle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectHandle->setIcon(icon23);
        actionSaveSelection = new QAction(MainWindow);
        actionSaveSelection->setObjectName(QString::fromUtf8("actionSaveSelection"));
        QIcon icon24;
        icon24.addFile(QString::fromUtf8(":/resource/SaveSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSaveSelection->setIcon(icon24);
        actionReadSelection = new QAction(MainWindow);
        actionReadSelection->setObjectName(QString::fromUtf8("actionReadSelection"));
        QIcon icon25;
        icon25.addFile(QString::fromUtf8(":/resource/InputSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionReadSelection->setIcon(icon25);
        actionSelectChamber = new QAction(MainWindow);
        actionSelectChamber->setObjectName(QString::fromUtf8("actionSelectChamber"));
        actionExport_to_Abaqus_model = new QAction(MainWindow);
        actionExport_to_Abaqus_model->setObjectName(QString::fromUtf8("actionExport_to_Abaqus_model"));
        actionExport_to_Abaqus_model->setCheckable(false);
        QIcon icon26;
        icon26.addFile(QString::fromUtf8(":/resource/abaqus logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExport_to_Abaqus_model->setIcon(icon26);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMouseTracking(true);
        centralWidget->setAcceptDrops(true);
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        MainWindow->setCentralWidget(centralWidget);
        navigationToolBar = new QToolBar(MainWindow);
        navigationToolBar->setObjectName(QString::fromUtf8("navigationToolBar"));
        navigationToolBar->setMovable(false);
        navigationToolBar->setIconSize(QSize(25, 25));
        navigationToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, navigationToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
        selectionToolBar = new QToolBar(MainWindow);
        selectionToolBar->setObjectName(QString::fromUtf8("selectionToolBar"));
        selectionToolBar->setMovable(false);
        selectionToolBar->setIconSize(QSize(25, 25));
        selectionToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, selectionToolBar);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dockWidget->sizePolicy().hasHeightForWidth());
        dockWidget->setSizePolicy(sizePolicy);
        dockWidget->setMinimumSize(QSize(401, 984));
        dockWidget->setMaximumSize(QSize(401, 524287));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setLayoutDirection(Qt::LeftToRight);
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_MANY_3DP_CNC_CAM = new QLabel(dockWidgetContents);
        label_MANY_3DP_CNC_CAM->setObjectName(QString::fromUtf8("label_MANY_3DP_CNC_CAM"));
        QFont font1;
        font1.setPointSize(10);
        label_MANY_3DP_CNC_CAM->setFont(font1);

        verticalLayout->addWidget(label_MANY_3DP_CNC_CAM);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_PosNorFile = new QLabel(dockWidgetContents);
        label_PosNorFile->setObjectName(QString::fromUtf8("label_PosNorFile"));
        QFont font2;
        font2.setPointSize(8);
        font2.setBold(true);
        font2.setWeight(75);
        label_PosNorFile->setFont(font2);

        horizontalLayout_9->addWidget(label_PosNorFile);

        lineEdit_SorceDataDir = new QLineEdit(dockWidgetContents);
        lineEdit_SorceDataDir->setObjectName(QString::fromUtf8("lineEdit_SorceDataDir"));

        horizontalLayout_9->addWidget(lineEdit_SorceDataDir);


        verticalLayout->addLayout(horizontalLayout_9);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        label_5 = new QLabel(dockWidgetContents);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setFont(font);
        label_5->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 255);"));

        verticalLayout->addWidget(label_5);

        pushButton_Comp_initialGuess_envelopSupport = new QPushButton(dockWidgetContents);
        pushButton_Comp_initialGuess_envelopSupport->setObjectName(QString::fromUtf8("pushButton_Comp_initialGuess_envelopSupport"));

        verticalLayout->addWidget(pushButton_Comp_initialGuess_envelopSupport);

        line_6 = new QFrame(dockWidgetContents);
        line_6->setObjectName(QString::fromUtf8("line_6"));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_6);

        line_7 = new QFrame(dockWidgetContents);
        line_7->setObjectName(QString::fromUtf8("line_7"));
        line_7->setFrameShape(QFrame::HLine);
        line_7->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_7);

        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font);
        label->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 255);"));

        verticalLayout->addWidget(label);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_Xmove = new QLabel(dockWidgetContents);
        label_Xmove->setObjectName(QString::fromUtf8("label_Xmove"));
        label_Xmove->setMaximumSize(QSize(16777215, 16777215));
        QFont font3;
        font3.setPointSize(8);
        label_Xmove->setFont(font3);

        horizontalLayout_11->addWidget(label_Xmove);

        doubleSpinBox_Xmove = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_Xmove->setObjectName(QString::fromUtf8("doubleSpinBox_Xmove"));
        doubleSpinBox_Xmove->setMaximumSize(QSize(62, 16777215));
        doubleSpinBox_Xmove->setFont(font3);
        doubleSpinBox_Xmove->setMinimum(-150.000000000000000);
        doubleSpinBox_Xmove->setMaximum(150.000000000000000);
        doubleSpinBox_Xmove->setValue(0.000000000000000);

        horizontalLayout_11->addWidget(doubleSpinBox_Xmove);

        label_Ymove = new QLabel(dockWidgetContents);
        label_Ymove->setObjectName(QString::fromUtf8("label_Ymove"));
        label_Ymove->setMaximumSize(QSize(16777215, 16777215));
        label_Ymove->setFont(font3);

        horizontalLayout_11->addWidget(label_Ymove);

        doubleSpinBox_Ymove = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_Ymove->setObjectName(QString::fromUtf8("doubleSpinBox_Ymove"));
        doubleSpinBox_Ymove->setMaximumSize(QSize(62, 16777215));
        doubleSpinBox_Ymove->setFont(font3);
        doubleSpinBox_Ymove->setMinimum(-150.000000000000000);
        doubleSpinBox_Ymove->setMaximum(150.000000000000000);
        doubleSpinBox_Ymove->setValue(0.000000000000000);

        horizontalLayout_11->addWidget(doubleSpinBox_Ymove);

        label_Zmove = new QLabel(dockWidgetContents);
        label_Zmove->setObjectName(QString::fromUtf8("label_Zmove"));
        label_Zmove->setMaximumSize(QSize(16777215, 16777215));
        label_Zmove->setFont(font3);

        horizontalLayout_11->addWidget(label_Zmove);

        doubleSpinBox_Zmove = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_Zmove->setObjectName(QString::fromUtf8("doubleSpinBox_Zmove"));
        doubleSpinBox_Zmove->setMaximumSize(QSize(62, 16777215));
        doubleSpinBox_Zmove->setFont(font3);
        doubleSpinBox_Zmove->setMinimum(-99.989999999999995);
        doubleSpinBox_Zmove->setValue(0.000000000000000);

        horizontalLayout_11->addWidget(doubleSpinBox_Zmove);

        pushButton_readGcodeSourceData = new QPushButton(dockWidgetContents);
        pushButton_readGcodeSourceData->setObjectName(QString::fromUtf8("pushButton_readGcodeSourceData"));
        pushButton_readGcodeSourceData->setMinimumSize(QSize(90, 0));
        QFont font4;
        font4.setPointSize(10);
        font4.setBold(true);
        font4.setItalic(false);
        font4.setUnderline(true);
        font4.setWeight(75);
        pushButton_readGcodeSourceData->setFont(font4);
        pushButton_readGcodeSourceData->setStyleSheet(QString::fromUtf8("color: rgb(0, 80, 0);"));

        horizontalLayout_11->addWidget(pushButton_readGcodeSourceData);


        verticalLayout->addLayout(horizontalLayout_11);

        line_4 = new QFrame(dockWidgetContents);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_4);

        label_3 = new QLabel(dockWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setFont(font);
        label_3->setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 255);"));

        verticalLayout->addWidget(label_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        spinBox_ShowLayerIndex = new QSpinBox(dockWidgetContents);
        spinBox_ShowLayerIndex->setObjectName(QString::fromUtf8("spinBox_ShowLayerIndex"));
        spinBox_ShowLayerIndex->setMinimumSize(QSize(1, 0));

        horizontalLayout_2->addWidget(spinBox_ShowLayerIndex);

        checkBox_EachLayerSwitch = new QCheckBox(dockWidgetContents);
        checkBox_EachLayerSwitch->setObjectName(QString::fromUtf8("checkBox_EachLayerSwitch"));

        horizontalLayout_2->addWidget(checkBox_EachLayerSwitch);

        pushButton_ShowAllLayers = new QPushButton(dockWidgetContents);
        pushButton_ShowAllLayers->setObjectName(QString::fromUtf8("pushButton_ShowAllLayers"));

        horizontalLayout_2->addWidget(pushButton_ShowAllLayers);

        label_4 = new QLabel(dockWidgetContents);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_2->addWidget(label_4);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        checkBox_onlyShowOnetype_layers = new QCheckBox(dockWidgetContents);
        checkBox_onlyShowOnetype_layers->setObjectName(QString::fromUtf8("checkBox_onlyShowOnetype_layers"));
        checkBox_onlyShowOnetype_layers->setFont(font);

        horizontalLayout_5->addWidget(checkBox_onlyShowOnetype_layers);

        radioButton_initialORsupport = new QRadioButton(dockWidgetContents);
        radioButton_initialORsupport->setObjectName(QString::fromUtf8("radioButton_initialORsupport"));
        QFont font5;
        font5.setBold(false);
        font5.setWeight(50);
        radioButton_initialORsupport->setFont(font5);
        radioButton_initialORsupport->setAutoRepeat(false);
        radioButton_initialORsupport->setAutoExclusive(false);

        horizontalLayout_5->addWidget(radioButton_initialORsupport);


        horizontalLayout_4->addLayout(horizontalLayout_5);

        checkBox_draw_LargeISOlayers = new QCheckBox(dockWidgetContents);
        checkBox_draw_LargeISOlayers->setObjectName(QString::fromUtf8("checkBox_draw_LargeISOlayers"));
        checkBox_draw_LargeISOlayers->setEnabled(false);
        checkBox_draw_LargeISOlayers->setFont(font5);

        horizontalLayout_4->addWidget(checkBox_draw_LargeISOlayers);


        verticalLayout->addLayout(horizontalLayout_4);

        line_5 = new QFrame(dockWidgetContents);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        pushButton_buildSupportRaySet = new QPushButton(dockWidgetContents);
        pushButton_buildSupportRaySet->setObjectName(QString::fromUtf8("pushButton_buildSupportRaySet"));
        pushButton_buildSupportRaySet->setEnabled(false);

        horizontalLayout_6->addWidget(pushButton_buildSupportRaySet);

        radioButton_deselect_origin = new QRadioButton(dockWidgetContents);
        radioButton_deselect_origin->setObjectName(QString::fromUtf8("radioButton_deselect_origin"));
        radioButton_deselect_origin->setEnabled(false);
        radioButton_deselect_origin->setAutoExclusive(false);

        horizontalLayout_6->addWidget(radioButton_deselect_origin);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        pushButton_buildSupportLayerSet = new QPushButton(dockWidgetContents);
        pushButton_buildSupportLayerSet->setObjectName(QString::fromUtf8("pushButton_buildSupportLayerSet"));
        pushButton_buildSupportLayerSet->setEnabled(false);

        horizontalLayout_7->addWidget(pushButton_buildSupportLayerSet);

        checkBox_verifyByMarching = new QCheckBox(dockWidgetContents);
        checkBox_verifyByMarching->setObjectName(QString::fromUtf8("checkBox_verifyByMarching"));
        checkBox_verifyByMarching->setMaximumSize(QSize(150, 16777215));
        checkBox_verifyByMarching->setChecked(false);

        horizontalLayout_7->addWidget(checkBox_verifyByMarching);


        verticalLayout->addLayout(horizontalLayout_7);

        radioButton_showRayOrSurface = new QRadioButton(dockWidgetContents);
        radioButton_showRayOrSurface->setObjectName(QString::fromUtf8("radioButton_showRayOrSurface"));
        radioButton_showRayOrSurface->setEnabled(true);
        radioButton_showRayOrSurface->setFont(font5);
        radioButton_showRayOrSurface->setAutoExclusive(false);

        verticalLayout->addWidget(radioButton_showRayOrSurface);

        line_3 = new QFrame(dockWidgetContents);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_3);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        radioButton_tightSupportLayerDraw = new QRadioButton(dockWidgetContents);
        radioButton_tightSupportLayerDraw->setObjectName(QString::fromUtf8("radioButton_tightSupportLayerDraw"));
        radioButton_tightSupportLayerDraw->setEnabled(false);
        radioButton_tightSupportLayerDraw->setFont(font5);

        horizontalLayout_3->addWidget(radioButton_tightSupportLayerDraw);

        boxDeselect = new QCheckBox(dockWidgetContents);
        boxDeselect->setObjectName(QString::fromUtf8("boxDeselect"));
        boxDeselect->setMaximumSize(QSize(150, 16777215));

        horizontalLayout_3->addWidget(boxDeselect);


        verticalLayout->addLayout(horizontalLayout_3);

        line_2 = new QFrame(dockWidgetContents);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line_2);

        pushButton_buildSupportToolpathSet = new QPushButton(dockWidgetContents);
        pushButton_buildSupportToolpathSet->setObjectName(QString::fromUtf8("pushButton_buildSupportToolpathSet"));

        verticalLayout->addWidget(pushButton_buildSupportToolpathSet);

        pushButton_output_Toolpath = new QPushButton(dockWidgetContents);
        pushButton_output_Toolpath->setObjectName(QString::fromUtf8("pushButton_output_Toolpath"));

        verticalLayout->addWidget(pushButton_output_Toolpath);

        treeView = new QTreeView(dockWidgetContents);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setEnabled(true);
        treeView->setProperty("showDropIndicator", QVariant(true));
        treeView->setIndentation(5);
        treeView->header()->setVisible(false);

        verticalLayout->addWidget(treeView);

        pushButton_TestFunc = new QPushButton(dockWidgetContents);
        pushButton_TestFunc->setObjectName(QString::fromUtf8("pushButton_TestFunc"));

        verticalLayout->addWidget(pushButton_TestFunc);

        pushButton_readRobot_model = new QPushButton(dockWidgetContents);
        pushButton_readRobot_model->setObjectName(QString::fromUtf8("pushButton_readRobot_model"));

        verticalLayout->addWidget(pushButton_readRobot_model);

        pushButton_clearAll = new QPushButton(dockWidgetContents);
        pushButton_clearAll->setObjectName(QString::fromUtf8("pushButton_clearAll"));

        verticalLayout->addWidget(pushButton_clearAll);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1331, 26));
        menuBar->setLayoutDirection(Qt::LeftToRight);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuSelect = new QMenu(menuBar);
        menuSelect->setObjectName(QString::fromUtf8("menuSelect"));
        MainWindow->setMenuBar(menuBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setMovable(false);
        toolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        navigationToolBar->addAction(actionFront);
        navigationToolBar->addAction(actionBack);
        navigationToolBar->addAction(actionTop);
        navigationToolBar->addAction(actionBottom);
        navigationToolBar->addAction(actionLeft);
        navigationToolBar->addAction(actionRight);
        navigationToolBar->addAction(actionIsometric);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionZoom_In);
        navigationToolBar->addAction(actionZoom_Out);
        navigationToolBar->addAction(actionZoom_All);
        navigationToolBar->addAction(actionZoom_Window);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionShade);
        navigationToolBar->addAction(actionMesh);
        navigationToolBar->addAction(actionNode);
        navigationToolBar->addAction(actionProfile);
        navigationToolBar->addAction(actionFaceNormal);
        navigationToolBar->addAction(actionNodeNormal);
        selectionToolBar->addAction(actionSaveSelection);
        selectionToolBar->addAction(actionReadSelection);
        selectionToolBar->addSeparator();
        selectionToolBar->addAction(actionSelectNode);
        selectionToolBar->addAction(actionSelectEdge);
        selectionToolBar->addAction(actionSelectFace);
        selectionToolBar->addAction(actionSelectFix);
        selectionToolBar->addAction(actionSelectHandle);
        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuBar->addAction(menuSelect->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSaveSelection);
        menuFile->addAction(actionReadSelection);
        menuView->addAction(actionFront);
        menuView->addAction(actionBack);
        menuView->addAction(actionTop);
        menuView->addAction(actionBottom);
        menuView->addAction(actionLeft);
        menuView->addAction(actionRight);
        menuView->addAction(actionIsometric);
        menuView->addSeparator();
        menuView->addAction(actionZoom_In);
        menuView->addAction(actionZoom_Out);
        menuView->addAction(actionZoom_All);
        menuView->addAction(actionZoom_Window);
        menuView->addSeparator();
        menuView->addAction(actionShade);
        menuView->addAction(actionMesh);
        menuView->addAction(actionNode);
        menuView->addAction(actionProfile);
        menuView->addSeparator();
        menuView->addAction(actionShifttoOrigin);
        menuSelect->addAction(actionSelectNode);
        menuSelect->addAction(actionSelectEdge);
        menuSelect->addAction(actionSelectFace);
        menuSelect->addSeparator();
        menuSelect->addAction(actionSelectFix);
        menuSelect->addAction(actionSelectHandle);
        menuSelect->addSeparator();
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", nullptr));
        actionFront->setText(QApplication::translate("MainWindow", "Front", nullptr));
        actionBack->setText(QApplication::translate("MainWindow", "Back", nullptr));
        actionTop->setText(QApplication::translate("MainWindow", "Top", nullptr));
        actionBottom->setText(QApplication::translate("MainWindow", "Bottom", nullptr));
        actionLeft->setText(QApplication::translate("MainWindow", "Left", nullptr));
        actionRight->setText(QApplication::translate("MainWindow", "Right", nullptr));
        actionIsometric->setText(QApplication::translate("MainWindow", "Isometric", nullptr));
        actionZoom_In->setText(QApplication::translate("MainWindow", "Zoom In", nullptr));
        actionZoom_Out->setText(QApplication::translate("MainWindow", "Zoom Out", nullptr));
        actionZoom_All->setText(QApplication::translate("MainWindow", "Zoom All", nullptr));
        actionZoom_Window->setText(QApplication::translate("MainWindow", "Zoom Window", nullptr));
        actionShade->setText(QApplication::translate("MainWindow", "Shade", nullptr));
        actionMesh->setText(QApplication::translate("MainWindow", "Mesh", nullptr));
        actionNode->setText(QApplication::translate("MainWindow", "Node", nullptr));
        actionSave->setText(QApplication::translate("MainWindow", "Save", nullptr));
        actionSelectNode->setText(QApplication::translate("MainWindow", "Node", nullptr));
        actionSelectFace->setText(QApplication::translate("MainWindow", "Face", nullptr));
        actionShifttoOrigin->setText(QApplication::translate("MainWindow", "Shift to Origin", nullptr));
        actionProfile->setText(QApplication::translate("MainWindow", "Profile", nullptr));
        actionFaceNormal->setText(QApplication::translate("MainWindow", "FaceNormal", nullptr));
        actionNodeNormal->setText(QApplication::translate("MainWindow", "NodeNormal", nullptr));
        actionSelectEdge->setText(QApplication::translate("MainWindow", "Edge", nullptr));
        actionGenerate->setText(QApplication::translate("MainWindow", "Generate", nullptr));
        actionTest_1->setText(QApplication::translate("MainWindow", "Test_1", nullptr));
        actionSelectFix->setText(QApplication::translate("MainWindow", "Fix", nullptr));
        actionSelectHandle->setText(QApplication::translate("MainWindow", "Handle & Rigid", nullptr));
        actionSaveSelection->setText(QApplication::translate("MainWindow", "Save selection", nullptr));
        actionReadSelection->setText(QApplication::translate("MainWindow", "Read selection", nullptr));
        actionSelectChamber->setText(QApplication::translate("MainWindow", "Select Chamber (SORO)", nullptr));
        actionExport_to_Abaqus_model->setText(QApplication::translate("MainWindow", "Export to Abaqus model", nullptr));
        navigationToolBar->setWindowTitle(QApplication::translate("MainWindow", "navigationToolBar", nullptr));
        selectionToolBar->setWindowTitle(QApplication::translate("MainWindow", "selectionToolBar", nullptr));
        label_MANY_3DP_CNC_CAM->setText(QApplication::translate("MainWindow", "Support Optimization", nullptr));
        label_PosNorFile->setText(QApplication::translate("MainWindow", "File Dir:", nullptr));
        lineEdit_SorceDataDir->setText(QApplication::translate("MainWindow", "yoga_cut", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "Initial Guess of Support Envelope", nullptr));
        pushButton_Comp_initialGuess_envelopSupport->setText(QApplication::translate("MainWindow", "Compute conservative Convex Hull", nullptr));
        label->setText(QApplication::translate("MainWindow", "Input Layers", nullptr));
        label_Xmove->setText(QApplication::translate("MainWindow", "X", nullptr));
        label_Ymove->setText(QApplication::translate("MainWindow", "Y", nullptr));
        label_Zmove->setText(QApplication::translate("MainWindow", "Z", nullptr));
        pushButton_readGcodeSourceData->setText(QApplication::translate("MainWindow", "1.Read Data", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Draw inputed Layers", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "Show:", nullptr));
        checkBox_EachLayerSwitch->setText(QApplication::translate("MainWindow", "single", nullptr));
        pushButton_ShowAllLayers->setText(QApplication::translate("MainWindow", "Show ALL", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "layer", nullptr));
        checkBox_onlyShowOnetype_layers->setText(QApplication::translate("MainWindow", "Only ", nullptr));
        radioButton_initialORsupport->setText(QApplication::translate("MainWindow", "initial/support", nullptr));
        checkBox_draw_LargeISOlayers->setText(QApplication::translate("MainWindow", "iso-layer", nullptr));
        pushButton_buildSupportRaySet->setText(QApplication::translate("MainWindow", "Build Support RAY", nullptr));
        radioButton_deselect_origin->setText(QApplication::translate("MainWindow", "de-select origin", nullptr));
        pushButton_buildSupportLayerSet->setText(QApplication::translate("MainWindow", "Build Support Layer", nullptr));
        checkBox_verifyByMarching->setText(QApplication::translate("MainWindow", "Marching cube", nullptr));
        radioButton_showRayOrSurface->setText(QApplication::translate("MainWindow", " ray/surface of MC (support Convex hull)", nullptr));
        radioButton_tightSupportLayerDraw->setText(QApplication::translate("MainWindow", "tight support show", nullptr));
        boxDeselect->setText(QApplication::translate("MainWindow", "de-Select", nullptr));
        pushButton_buildSupportToolpathSet->setText(QApplication::translate("MainWindow", "Toolpath generation", nullptr));
        pushButton_output_Toolpath->setText(QApplication::translate("MainWindow", "Toolpath output", nullptr));
        pushButton_TestFunc->setText(QApplication::translate("MainWindow", "test Function", nullptr));
        pushButton_readRobot_model->setText(QApplication::translate("MainWindow", "read Robot CAD", nullptr));
        pushButton_clearAll->setText(QApplication::translate("MainWindow", "Clear all", nullptr));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", nullptr));
        menuView->setTitle(QApplication::translate("MainWindow", "View", nullptr));
        menuSelect->setTitle(QApplication::translate("MainWindow", "Select", nullptr));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
