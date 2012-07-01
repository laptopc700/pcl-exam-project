#ifndef UI_H
#define UI_H

#include <QtGui>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/interactor.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <vtkSmartPointer.h>

class Ui : public QMainWindow
{
    Q_OBJECT

public:
    Ui();// constructor

private slots:
    void about();
    void aboutPCL();

private:
    void createActions(); // create all the actions of the ui
    void setupMenuBar();
    void setupMainWidget();
    void setupDockWidgets();
    void setupStatusBar();
    static void pointPickCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);

    QMenu *fileMenu;
    QMenu *helpMenu;
    QAction *quitAct;
    QAction *aboutAct;
    QAction *aboutQtAct;
    QAction *aboutPCLAct;

    QVTKWidget *qvtkVisualizer;
    pcl::visualization::PCLVisualizer *viewer;

};

#endif // UI_H
