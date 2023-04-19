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
#include <QtWidgets/QSpacerItem>
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
    QLabel *label_13;
    QLabel *label_2;
    QFrame *line;
    QLabel *label;
    QHBoxLayout *horizontalLayout_16;
    QLabel *label_PosNorFile;
    QLineEdit *lineEdit_SorceDataDir;
    QPushButton *pushButton_readData;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_5;
    QPushButton *pushButton_model_positionUpdate;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_Xmove;
    QDoubleSpinBox *doubleSpinBox_Xmove;
    QLabel *label_Ymove;
    QDoubleSpinBox *doubleSpinBox_Ymove;
    QLabel *label_Zmove;
    QDoubleSpinBox *doubleSpinBox_Zmove;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QDoubleSpinBox *doubleSpinBox_XRot;
    QLabel *label_4;
    QDoubleSpinBox *doubleSpinBox_YRot;
    QLabel *label_18;
    QDoubleSpinBox *doubleSpinBox_ZRot;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_7;
    QDoubleSpinBox *doubleSpinBox_offset_dist;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_buildEnvelopeCH;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *pushButton_remeshCH;
    QPushButton *pushButton_generateSupportSpace;
    QPushButton *pushButton_readSupportSpace;
    QPushButton *pushButton_transferField_2_SupportSpace;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_generateCompatibleLayers;
    QLabel *label_14;
    QSpinBox *spinBox_isoLayerNumber;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_15;
    QSpinBox *spinBox_ShowLayerIndex;
    QCheckBox *checkBox_EachLayerSwitch;
    QRadioButton *radioButton_compatibleLayer;
    QPushButton *pushButton_ShowAllLayers;
    QPushButton *pushButton_generate_support_structure;
    QPushButton *pushButton_slimmedSupportGeneration;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_16;
    QDoubleSpinBox *doubleSpinBox_toolPathWidth;
    QLabel *label_17;
    QDoubleSpinBox *doubleSpinBox_toolPathDistance;
    QPushButton *pushButton_toolPathGeneration;
    QPushButton *pushButton_Tp4Ur5e;
    QTreeView *treeView;
    QHBoxLayout *horizontalLayout_12;
    QCheckBox *boxDeselect;
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
        MainWindow->resize(1290, 991);
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
        dockWidget->setMinimumSize(QSize(350, 900));
        dockWidget->setMaximumSize(QSize(350, 900));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setLayoutDirection(Qt::LeftToRight);
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_13 = new QLabel(dockWidgetContents);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        QFont font1;
        font1.setPointSize(10);
        label_13->setFont(font1);

        verticalLayout->addWidget(label_13);

        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font);
        label->setStyleSheet(QString::fromUtf8(""));

        verticalLayout->addWidget(label);

        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setSpacing(6);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        label_PosNorFile = new QLabel(dockWidgetContents);
        label_PosNorFile->setObjectName(QString::fromUtf8("label_PosNorFile"));
        QFont font2;
        font2.setPointSize(8);
        font2.setBold(false);
        font2.setWeight(50);
        label_PosNorFile->setFont(font2);

        horizontalLayout_16->addWidget(label_PosNorFile);

        lineEdit_SorceDataDir = new QLineEdit(dockWidgetContents);
        lineEdit_SorceDataDir->setObjectName(QString::fromUtf8("lineEdit_SorceDataDir"));

        horizontalLayout_16->addWidget(lineEdit_SorceDataDir);

        pushButton_readData = new QPushButton(dockWidgetContents);
        pushButton_readData->setObjectName(QString::fromUtf8("pushButton_readData"));
        QFont font3;
        font3.setFamily(QString::fromUtf8("3ds"));
        font3.setPointSize(9);
        font3.setBold(true);
        font3.setItalic(false);
        font3.setUnderline(false);
        font3.setWeight(75);
        pushButton_readData->setFont(font3);
        pushButton_readData->setStyleSheet(QString::fromUtf8(""));

        horizontalLayout_16->addWidget(pushButton_readData);


        verticalLayout->addLayout(horizontalLayout_16);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_5 = new QLabel(dockWidgetContents);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_10->addWidget(label_5);

        pushButton_model_positionUpdate = new QPushButton(dockWidgetContents);
        pushButton_model_positionUpdate->setObjectName(QString::fromUtf8("pushButton_model_positionUpdate"));
        pushButton_model_positionUpdate->setMaximumSize(QSize(64, 16777215));
        QFont font4;
        font4.setPointSize(8);
        font4.setBold(true);
        font4.setWeight(75);
        pushButton_model_positionUpdate->setFont(font4);
        pushButton_model_positionUpdate->setStyleSheet(QString::fromUtf8(""));

        horizontalLayout_10->addWidget(pushButton_model_positionUpdate);


        verticalLayout->addLayout(horizontalLayout_10);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_Xmove = new QLabel(dockWidgetContents);
        label_Xmove->setObjectName(QString::fromUtf8("label_Xmove"));
        label_Xmove->setMaximumSize(QSize(16777215, 16777215));
        label_Xmove->setFont(font2);

        horizontalLayout_11->addWidget(label_Xmove);

        doubleSpinBox_Xmove = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_Xmove->setObjectName(QString::fromUtf8("doubleSpinBox_Xmove"));
        doubleSpinBox_Xmove->setFont(font2);
        doubleSpinBox_Xmove->setMinimum(-150.000000000000000);
        doubleSpinBox_Xmove->setMaximum(150.000000000000000);
        doubleSpinBox_Xmove->setValue(0.000000000000000);

        horizontalLayout_11->addWidget(doubleSpinBox_Xmove);

        label_Ymove = new QLabel(dockWidgetContents);
        label_Ymove->setObjectName(QString::fromUtf8("label_Ymove"));
        label_Ymove->setMaximumSize(QSize(16777215, 16777215));
        label_Ymove->setFont(font2);

        horizontalLayout_11->addWidget(label_Ymove);

        doubleSpinBox_Ymove = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_Ymove->setObjectName(QString::fromUtf8("doubleSpinBox_Ymove"));
        doubleSpinBox_Ymove->setFont(font2);
        doubleSpinBox_Ymove->setMinimum(-150.000000000000000);
        doubleSpinBox_Ymove->setMaximum(150.000000000000000);
        doubleSpinBox_Ymove->setValue(0.000000000000000);

        horizontalLayout_11->addWidget(doubleSpinBox_Ymove);

        label_Zmove = new QLabel(dockWidgetContents);
        label_Zmove->setObjectName(QString::fromUtf8("label_Zmove"));
        label_Zmove->setMaximumSize(QSize(16777215, 16777215));
        label_Zmove->setFont(font2);

        horizontalLayout_11->addWidget(label_Zmove);

        doubleSpinBox_Zmove = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_Zmove->setObjectName(QString::fromUtf8("doubleSpinBox_Zmove"));
        doubleSpinBox_Zmove->setFont(font2);
        doubleSpinBox_Zmove->setMinimum(-150.000000000000000);
        doubleSpinBox_Zmove->setMaximum(150.000000000000000);
        doubleSpinBox_Zmove->setValue(0.000000000000000);

        horizontalLayout_11->addWidget(doubleSpinBox_Zmove);


        verticalLayout->addLayout(horizontalLayout_11);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_3 = new QLabel(dockWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        QFont font5;
        font5.setPointSize(8);
        font5.setBold(false);
        font5.setWeight(50);
        font5.setKerning(false);
        label_3->setFont(font5);

        horizontalLayout_2->addWidget(label_3);

        doubleSpinBox_XRot = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_XRot->setObjectName(QString::fromUtf8("doubleSpinBox_XRot"));
        doubleSpinBox_XRot->setFont(font2);
        doubleSpinBox_XRot->setMinimum(-180.000000000000000);
        doubleSpinBox_XRot->setMaximum(180.000000000000000);

        horizontalLayout_2->addWidget(doubleSpinBox_XRot);

        label_4 = new QLabel(dockWidgetContents);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setFont(font2);

        horizontalLayout_2->addWidget(label_4);

        doubleSpinBox_YRot = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_YRot->setObjectName(QString::fromUtf8("doubleSpinBox_YRot"));
        doubleSpinBox_YRot->setFont(font2);
        doubleSpinBox_YRot->setMinimum(-180.000000000000000);
        doubleSpinBox_YRot->setMaximum(180.000000000000000);

        horizontalLayout_2->addWidget(doubleSpinBox_YRot);

        label_18 = new QLabel(dockWidgetContents);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setFont(font2);

        horizontalLayout_2->addWidget(label_18);

        doubleSpinBox_ZRot = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_ZRot->setObjectName(QString::fromUtf8("doubleSpinBox_ZRot"));
        doubleSpinBox_ZRot->setFont(font2);
        doubleSpinBox_ZRot->setMinimum(-180.000000000000000);
        doubleSpinBox_ZRot->setMaximum(180.000000000000000);

        horizontalLayout_2->addWidget(doubleSpinBox_ZRot);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_7 = new QLabel(dockWidgetContents);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setFont(font2);

        horizontalLayout_6->addWidget(label_7);

        doubleSpinBox_offset_dist = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_offset_dist->setObjectName(QString::fromUtf8("doubleSpinBox_offset_dist"));
        doubleSpinBox_offset_dist->setFont(font2);

        horizontalLayout_6->addWidget(doubleSpinBox_offset_dist);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer);

        pushButton_buildEnvelopeCH = new QPushButton(dockWidgetContents);
        pushButton_buildEnvelopeCH->setObjectName(QString::fromUtf8("pushButton_buildEnvelopeCH"));
        pushButton_buildEnvelopeCH->setFont(font4);

        horizontalLayout_6->addWidget(pushButton_buildEnvelopeCH);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        pushButton_remeshCH = new QPushButton(dockWidgetContents);
        pushButton_remeshCH->setObjectName(QString::fromUtf8("pushButton_remeshCH"));
        QFont font6;
        font6.setPointSize(8);
        pushButton_remeshCH->setFont(font6);

        horizontalLayout_5->addWidget(pushButton_remeshCH);

        pushButton_generateSupportSpace = new QPushButton(dockWidgetContents);
        pushButton_generateSupportSpace->setObjectName(QString::fromUtf8("pushButton_generateSupportSpace"));
        pushButton_generateSupportSpace->setEnabled(false);
        pushButton_generateSupportSpace->setFont(font6);

        horizontalLayout_5->addWidget(pushButton_generateSupportSpace);


        verticalLayout->addLayout(horizontalLayout_5);

        pushButton_readSupportSpace = new QPushButton(dockWidgetContents);
        pushButton_readSupportSpace->setObjectName(QString::fromUtf8("pushButton_readSupportSpace"));
        pushButton_readSupportSpace->setEnabled(false);
        QFont font7;
        font7.setFamily(QString::fromUtf8("3ds"));
        font7.setBold(true);
        font7.setWeight(75);
        pushButton_readSupportSpace->setFont(font7);
        pushButton_readSupportSpace->setStyleSheet(QString::fromUtf8("color: rgb(0,0, 255);"));

        verticalLayout->addWidget(pushButton_readSupportSpace);

        pushButton_transferField_2_SupportSpace = new QPushButton(dockWidgetContents);
        pushButton_transferField_2_SupportSpace->setObjectName(QString::fromUtf8("pushButton_transferField_2_SupportSpace"));
        pushButton_transferField_2_SupportSpace->setFont(font7);
        pushButton_transferField_2_SupportSpace->setStyleSheet(QString::fromUtf8("color: rgb(0,0, 255);"));

        verticalLayout->addWidget(pushButton_transferField_2_SupportSpace);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        pushButton_generateCompatibleLayers = new QPushButton(dockWidgetContents);
        pushButton_generateCompatibleLayers->setObjectName(QString::fromUtf8("pushButton_generateCompatibleLayers"));
        pushButton_generateCompatibleLayers->setFont(font7);
        pushButton_generateCompatibleLayers->setStyleSheet(QString::fromUtf8("color: rgb(0,0, 255);"));

        horizontalLayout_7->addWidget(pushButton_generateCompatibleLayers);

        label_14 = new QLabel(dockWidgetContents);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setFont(font2);

        horizontalLayout_7->addWidget(label_14);

        spinBox_isoLayerNumber = new QSpinBox(dockWidgetContents);
        spinBox_isoLayerNumber->setObjectName(QString::fromUtf8("spinBox_isoLayerNumber"));
        spinBox_isoLayerNumber->setFont(font2);
        spinBox_isoLayerNumber->setMaximum(999);

        horizontalLayout_7->addWidget(spinBox_isoLayerNumber);


        verticalLayout->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_15 = new QLabel(dockWidgetContents);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setFont(font2);

        horizontalLayout_8->addWidget(label_15);

        spinBox_ShowLayerIndex = new QSpinBox(dockWidgetContents);
        spinBox_ShowLayerIndex->setObjectName(QString::fromUtf8("spinBox_ShowLayerIndex"));
        spinBox_ShowLayerIndex->setFont(font2);
        spinBox_ShowLayerIndex->setMaximum(999);

        horizontalLayout_8->addWidget(spinBox_ShowLayerIndex);

        checkBox_EachLayerSwitch = new QCheckBox(dockWidgetContents);
        checkBox_EachLayerSwitch->setObjectName(QString::fromUtf8("checkBox_EachLayerSwitch"));
        checkBox_EachLayerSwitch->setFont(font2);

        horizontalLayout_8->addWidget(checkBox_EachLayerSwitch);

        radioButton_compatibleLayer = new QRadioButton(dockWidgetContents);
        radioButton_compatibleLayer->setObjectName(QString::fromUtf8("radioButton_compatibleLayer"));
        radioButton_compatibleLayer->setEnabled(false);
        radioButton_compatibleLayer->setFont(font2);

        horizontalLayout_8->addWidget(radioButton_compatibleLayer);

        pushButton_ShowAllLayers = new QPushButton(dockWidgetContents);
        pushButton_ShowAllLayers->setObjectName(QString::fromUtf8("pushButton_ShowAllLayers"));
        pushButton_ShowAllLayers->setFont(font2);

        horizontalLayout_8->addWidget(pushButton_ShowAllLayers);


        verticalLayout->addLayout(horizontalLayout_8);

        pushButton_generate_support_structure = new QPushButton(dockWidgetContents);
        pushButton_generate_support_structure->setObjectName(QString::fromUtf8("pushButton_generate_support_structure"));
        pushButton_generate_support_structure->setEnabled(false);
        pushButton_generate_support_structure->setFont(font7);
        pushButton_generate_support_structure->setStyleSheet(QString::fromUtf8("color: rgb(0,192, 64);"));

        verticalLayout->addWidget(pushButton_generate_support_structure);

        pushButton_slimmedSupportGeneration = new QPushButton(dockWidgetContents);
        pushButton_slimmedSupportGeneration->setObjectName(QString::fromUtf8("pushButton_slimmedSupportGeneration"));
        pushButton_slimmedSupportGeneration->setFont(font7);
        pushButton_slimmedSupportGeneration->setStyleSheet(QString::fromUtf8("color: rgb(0,192, 64);"));

        verticalLayout->addWidget(pushButton_slimmedSupportGeneration);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_16 = new QLabel(dockWidgetContents);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setFont(font2);

        horizontalLayout_9->addWidget(label_16);

        doubleSpinBox_toolPathWidth = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_toolPathWidth->setObjectName(QString::fromUtf8("doubleSpinBox_toolPathWidth"));
        doubleSpinBox_toolPathWidth->setFont(font2);
        doubleSpinBox_toolPathWidth->setSingleStep(0.100000000000000);
        doubleSpinBox_toolPathWidth->setValue(0.600000000000000);

        horizontalLayout_9->addWidget(doubleSpinBox_toolPathWidth);

        label_17 = new QLabel(dockWidgetContents);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setFont(font2);

        horizontalLayout_9->addWidget(label_17);

        doubleSpinBox_toolPathDistance = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_toolPathDistance->setObjectName(QString::fromUtf8("doubleSpinBox_toolPathDistance"));
        doubleSpinBox_toolPathDistance->setFont(font2);
        doubleSpinBox_toolPathDistance->setSingleStep(0.100000000000000);
        doubleSpinBox_toolPathDistance->setValue(1.000000000000000);

        horizontalLayout_9->addWidget(doubleSpinBox_toolPathDistance);


        verticalLayout->addLayout(horizontalLayout_9);

        pushButton_toolPathGeneration = new QPushButton(dockWidgetContents);
        pushButton_toolPathGeneration->setObjectName(QString::fromUtf8("pushButton_toolPathGeneration"));
        pushButton_toolPathGeneration->setFont(font7);
        pushButton_toolPathGeneration->setStyleSheet(QString::fromUtf8("color: rgb(0,192, 64);"));

        verticalLayout->addWidget(pushButton_toolPathGeneration);

        pushButton_Tp4Ur5e = new QPushButton(dockWidgetContents);
        pushButton_Tp4Ur5e->setObjectName(QString::fromUtf8("pushButton_Tp4Ur5e"));
        QFont font8;
        font8.setFamily(QString::fromUtf8("3ds"));
        font8.setPointSize(9);
        font8.setBold(true);
        font8.setWeight(75);
        pushButton_Tp4Ur5e->setFont(font8);
        pushButton_Tp4Ur5e->setStyleSheet(QString::fromUtf8("color: rgb(0,192, 64);"));

        verticalLayout->addWidget(pushButton_Tp4Ur5e);

        treeView = new QTreeView(dockWidgetContents);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setEnabled(true);
        treeView->setProperty("showDropIndicator", QVariant(true));
        treeView->setIndentation(5);
        treeView->header()->setVisible(false);

        verticalLayout->addWidget(treeView);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        boxDeselect = new QCheckBox(dockWidgetContents);
        boxDeselect->setObjectName(QString::fromUtf8("boxDeselect"));

        horizontalLayout_12->addWidget(boxDeselect);

        pushButton_clearAll = new QPushButton(dockWidgetContents);
        pushButton_clearAll->setObjectName(QString::fromUtf8("pushButton_clearAll"));

        horizontalLayout_12->addWidget(pushButton_clearAll);


        verticalLayout->addLayout(horizontalLayout_12);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1290, 26));
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
        label_13->setText(QApplication::translate("MainWindow", "Support Generation", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "for Curved CCF printing", nullptr));
        label->setText(QApplication::translate("MainWindow", "Input model", nullptr));
        label_PosNorFile->setText(QApplication::translate("MainWindow", "Name:", nullptr));
        lineEdit_SorceDataDir->setText(QApplication::translate("MainWindow", "bridge", nullptr));
        pushButton_readData->setText(QApplication::translate("MainWindow", "1.Read Data", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "Modify model", nullptr));
        pushButton_model_positionUpdate->setText(QApplication::translate("MainWindow", "Update", nullptr));
        label_Xmove->setText(QApplication::translate("MainWindow", "Xm", nullptr));
        label_Ymove->setText(QApplication::translate("MainWindow", "Ym", nullptr));
        label_Zmove->setText(QApplication::translate("MainWindow", "Zm", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Xr", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Yr", nullptr));
        label_18->setText(QApplication::translate("MainWindow", "Zr", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "Offset", nullptr));
        pushButton_buildEnvelopeCH->setText(QApplication::translate("MainWindow", "Build envelope CH", nullptr));
        pushButton_remeshCH->setText(QApplication::translate("MainWindow", "Remesh CH", nullptr));
        pushButton_generateSupportSpace->setText(QApplication::translate("MainWindow", "Generate support space", nullptr));
        pushButton_readSupportSpace->setText(QApplication::translate("MainWindow", "2.Read Support spcace", nullptr));
        pushButton_transferField_2_SupportSpace->setText(QApplication::translate("MainWindow", "3.Transfer Field to Support Space", nullptr));
        pushButton_generateCompatibleLayers->setText(QApplication::translate("MainWindow", "4.Layer generation", nullptr));
        label_14->setText(QApplication::translate("MainWindow", "layer num", nullptr));
        label_15->setText(QApplication::translate("MainWindow", "show", nullptr));
        checkBox_EachLayerSwitch->setText(QApplication::translate("MainWindow", "each", nullptr));
        radioButton_compatibleLayer->setText(QApplication::translate("MainWindow", "pair", nullptr));
        pushButton_ShowAllLayers->setText(QApplication::translate("MainWindow", "ALL", nullptr));
        pushButton_generate_support_structure->setText(QApplication::translate("MainWindow", "2.Generate Support Skeleton", nullptr));
        pushButton_slimmedSupportGeneration->setText(QApplication::translate("MainWindow", "3.Extract Slim Support Layers", nullptr));
        label_16->setText(QApplication::translate("MainWindow", "Width", nullptr));
        label_17->setText(QApplication::translate("MainWindow", "Distance", nullptr));
        pushButton_toolPathGeneration->setText(QApplication::translate("MainWindow", "4.Contour Toolpath Generation", nullptr));
        pushButton_Tp4Ur5e->setText(QApplication::translate("MainWindow", "5.Waypoint generation UR5e", nullptr));
        boxDeselect->setText(QApplication::translate("MainWindow", "Deselect", nullptr));
        pushButton_clearAll->setText(QApplication::translate("MainWindow", "Clear All", nullptr));
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
