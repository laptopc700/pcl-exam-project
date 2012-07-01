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
    // UI functions
    void createActions(); // create all the actions of the ui
    void createQVTKWidget();
    void setupMenuBar();
    void setupStatusBar();
    void setupBoxAndLayout();

    // Motor functions
    static void pointPickCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);

    // Menu bar elements
    QMenu *fileMenu;
    QMenu *helpMenu;

    // Action placeholders
    QAction *quitAct;
    QAction *aboutAct;
    QAction *aboutQtAct;
    QAction *aboutPCLAct;

    // Widgets
    QPushButton *browseButton;
    QLineEdit *pathField;
    QPushButton *loadButton;
    QPushButton *addComponentButton;
    QPushButton *delComponentButton;
    QListWidget *componentList;

    // Group Boxes
    QGroupBox *loadTBox;
    QGroupBox *componentBox;
    QGroupBox *checkBox;
    QGroupBox *loadSBox;
    QGroupBox *resultsBox;
    QGroupBox *viewerControlsBox;

    // Layout handlers
    QHBoxLayout *mainLayout;
    QVBoxLayout *commandLayout;
    QVBoxLayout *viewerLayout;
    QHBoxLayout *loadTargetLayout;
    QVBoxLayout *componentLayout;
    QHBoxLayout *componentButtonsLayout;
    QVBoxLayout *checkLayout;
    QHBoxLayout *loadSourceLayout;
    QHBoxLayout *resultsLayout;
    QHBoxLayout *showTargetComponent;
    QHBoxLayout *showSourceComponent;

    // Visualization widget
    QVTKWidget *qvtkVisualizer;
    pcl::visualization::PCLVisualizer *viewer;

};

#endif // UI_H
