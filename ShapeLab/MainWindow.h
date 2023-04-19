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
#include "supportGene.h"

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

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
    //int Gcode_line_ind;
    //int simuLayerInd;
    //Eigen::MatrixXf Gcode_Table;
    //unsigned int operationTime = 0;
    /* ------------------------ */
	GLKObList polygenMeshList;

private:
    void createActions();
    void createTreeView();

    PolygenMesh *getSelectedPolygenMesh();

    QSignalMapper *signalMapper;
    QStandardItemModel *treeModel;

    supportGene* supportOperator;

private:
    PolygenMesh* _buildPolygenMesh(mesh_type type, std::string name);
    PolygenMesh* _detectPolygenMesh(mesh_type type);
    QMeshPatch* _detectPolygenMesh(mesh_type type, std::string patch_name);
    void _update_Position_Orientation_Parameter();
    void _setParameter_4_Rot_and_Mov(
        double Xmove, double Ymove, double Zmove,
        double Xrot, double Yrot, double Zrot);
    void _rot_And_Move_Model(QMeshPatch* m_tetModel, 
        double xRot, double yRot, double zRot,
        double xMove, double yMove, double zMove, 
        bool isUpdate_lastCoord3D);
    void _modify_scalarField_order(QMeshPatch* m_tetModel);
    void outputCH();

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

    /*This is for Display*/
    void update_Layer_Display();
    void all_Display();
    void change_maxLayerNum_normalORcompatible();
    //This if for Tree support generation
    void readData_tetModel_and_scalarField();
    void update_model_postion_orientation();
    void build_EnvelopeCH();
    void remeshCH();
    void generate_supportSpace();
    void readData_supportSpace();
    void transferField_2_supportSpace();
    void generate_compatible_layers();
    void generate_support_structure();
    void extract_slim_supportLayer();
    void toolPath_Generation();
    void UR_robot_waypoint_Generation();
};

#endif // MAINWINDOW_H
