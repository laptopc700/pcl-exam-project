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
    void setMotor();// funzione per settare il puntatore all'oggetto che elabora le point cloud, esterno alla UI

private slots:
    void about();
    void aboutPCL();

private:
// UI functions
    void createActions(); // create all the actions of the ui
    void setupMenuBar();
    void setupStatusBar();
    void setupLoadTBox();
    void setupComponentsBox();
    void setupChecksBox();
    void setupLoadSBox();
    void setupVisualizer();
    void setupResultsBox();
    void setupVisualizerCommands();
    void setupMainLayout();
    static void pointPickCallback(const pcl::visualization::PointPickingEvent& event, void* cookie); // callback function for the visualizer to interact with the mouse

// Motor object
    // TO DO: Creare classe che gestisca le cloud e fornisca metodi da utilizzare nella Ui per elaborare le point cloud (ancora da progettare)

// Menu bar elements
    QMenu *fileMenu;
    QMenu *helpMenu;

// Action placeholders
    QAction *quitAct;
    QAction *aboutAct;
    QAction *aboutQtAct;
    QAction *aboutPCLAct;

// Widgets
    QWidget *mainWidget;
    // load target widgets
    QPushButton *browseTButton;
    QLineEdit *pathTField;
    QPushButton *loadTButton;
    // add component widgets
    QPushButton *addComponentButton;
    QPushButton *delComponentButton;
    QListWidget *componentsList;
    // add check widgets
    QPushButton *addCheckButton;
    QPushButton *delCheckButton;
    QListWidget *checksList;
    // load source widgets
    QPushButton *browseSButton;
    QLineEdit *pathSField;
    QPushButton *loadSButton;
    // results widget
    QPushButton *startButton;
    QListWidget *resultsList;
    // visualization buttons widgets
    QPushButton *showTButton;
    QPushButton *showSButton;
    QPushButton *showTComponentButton;
    QComboBox *targetComponentsList;
    QPushButton *showSComponentButton;
    QComboBox *sourceComponentsList;
    QPushButton *clearAll;
    // visualization widget
    QVTKWidget *qvtkVisualizer;
    pcl::visualization::PCLVisualizer *viewer;

// Group Boxes
    QGroupBox *loadTBox;
    QGroupBox *componentsBox;
    QGroupBox *checksBox;
    QGroupBox *loadSBox;
    QGroupBox *resultsBox;
    QGroupBox *viewerControlsBox;

// Layout handlers
    QHBoxLayout *mainLayout;
    QVBoxLayout *commandsLayout;
    QVBoxLayout *viewerLayout;
    QHBoxLayout *loadTargetLayout;
    QVBoxLayout *componentsLayout;
    QHBoxLayout *componentButtonsLayout;
    QVBoxLayout *checksLayout;
    QHBoxLayout *checkButtonsLayout;
    QHBoxLayout *loadSourceLayout;
    QHBoxLayout *resultsLayout;
    QHBoxLayout *showTargetComponentLayout;
    QHBoxLayout *showSourceComponentLayout;
};

#endif // UI_H
