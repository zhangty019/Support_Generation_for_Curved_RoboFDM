#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"
#include <omp.h>
#include <QTimer>
#include <QLabel>

//#define PI		3.141592654
//#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
//#define ROTATE_TO_DEGREE(x)		57.295780490443*x

#include "../QMeshLib/BSPTree.h"
#include "../QMeshLib/QMeshVoxel.h"

#include "../QMeshLib/BSPTreeOperation.h"
#include "../QMeshLib/QMeshVoxelOperation.h"
#include "../QMeshLib/VOXSetStructure.h"

#include "supportGeneration.h"

using namespace std;

class DeformTet;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
	// Qtimer - defined function
    //void doTimerGcodeMoving();

private:
    Ui::MainWindow *ui;
    GLKLib *pGLK;

    /* add for Gcode generation */
    //QTimer Gcode_timer; //Gcode Simulation timer
    //int gocodetimerItertime;
    //int simuLayerInd;
    //Eigen::MatrixXf Gcode_Table;
    //unsigned int operationTime = 0;
    /* ------------------------ */
	GLKObList polygenMeshList;

private:
    void createActions();
    void createTreeView();
	void showTetraDeformationRatio();
	void MoveHandleRegion();
	void QTgetscreenshoot();

    PolygenMesh *getSelectedPolygenMesh();

    QSignalMapper *signalMapper;
    QStandardItemModel *treeModel;

private:// functions for support Opt.
    void _get_FileName(string dirctory, vector<string>& fileNameCell, bool onlyRead, bool onlyInit);
    void _modifyCoord(QMeshPatch* patchFile);

    PolygenMesh* _detectPolygenMesh(mesh_type type, std::string name, bool detectType);
    PolygenMesh* _buildPolygenMesh(mesh_type type, std::string name);

    /*This is for TetModel and vectorField input*/
    void _read_TET_vectorField();
    void _read_platform();

    // for SupportRAY collection
    supportGeneration* supportGene = NULL;
    bool is_TET_surface_Ray = false;
    bool is_TET_surface_polyline = false;
    bool is_TET_surface_tree = false;
    bool is_TET_surface_tree_new = true;
    bool is_polyline = false;
    //END

protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private slots:
    void open();
    void save();
	void saveSelection();
	void readSelection();

    void signalNavigation(int flag);
    void shiftToOrigin();
    void updateTree();
	void mouseMoveEvent(QMouseEvent *event);
    void on_pushButton_clearAll_clicked();
    void on_treeView_clicked(const QModelIndex &index);

	/*This is RoboSYS*/
    void import_CAD_RoboSYS();

    /*This is for Display*/
    void change_LayerDisplay();
    void viewAll_Layers();
    void showRayOrSurface();
    void show_ISO_Layer();
    void showTightSupportSurface();
    void deSelect_origin();

    /*This is for Initial guess of support Envelope*/
    void compute_initial_Guess_SupportEnvelope();
    /*This is for Initial layers input*/
    void readSliceData();
    /*This is for support ray generation*/
    void build_SupportRAY();
    /*This is for support layer generation*/
    void build_SupportMesh();
    /*This is for toolpath generation*/
    void get_CurvedToolpath();
    /*This is for toolpath output*/
    void output_Toolpath();

    // function test
    void test_func();
    // supportFree MainFunction
    //void build_SupportFree_Toolpath();
};

#endif // MAINWINDOW_H