#ifndef UI_H
#define UI_H

#include <QtGui>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/interactor.h>
#include <vtkSmartPointer.h>
#include "pcqc.h"

class Ui : public QMainWindow
{
    Q_OBJECT

public:

    // INIT
    Ui(Pcqc *pcqc);// constructor, also creates puntatore all'oggetto che elabora le point cloud, esterno alla UI (motor)
    // destructor is not needed for the gui elements because everything is parented in the QObjects tree.
    //They are deleted when the parent (main window->mainWidget) is deleted.
    ~Ui(); //The only thing to delete is the viewer (i guess), the motor is allocated in the stack

    // SETTERS
    void setMotor(Pcqc *pcqc);// to set another motor if we want the ui work with different pcqc instances.

    // GETTERS
    Pcqc* getMotor();
    pcl::visualization::PCLVisualizer* getViewer(); // returns the main window's visualizer
    pcl::visualization::PCLVisualizer* getDialogViewer(); // returns the dialog window's visualizer
    QVTKWidget* getViewerWidget(); // returns the main window's widget that holds the visualizer's renderer
    QDialog* getComponentDialog(); // returns the dialog window's widget

private slots:
    void about();
    void aboutPCL();
    void browseLoadTarget();
    void openComponentDialog();
    void setComponentDialogCallback();
    void resetComponentDialogCallback();
    void setColorThreshold();
    void setClusterThreshold();
    void segmentComponent();
    void saveComponent();
    void deleteComponent();
    void openCheckDialog();
    void browseLoadSource();

    void start();
    void showTarget();
    void showSource();
    void showRegistered();
    void showTargetComponent();
    void showSourceComponent();
    void clearAll();




private:
// UI FUNCTIONS
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
    static void pointPickCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
    static void pointPickCallbackSelectComponent(const pcl::visualization::PointPickingEvent& event, void* cookie);
    static QString colorToStyleSheet(QColor *color);

// Motor object
    Pcqc *motor;

// Main UI Menu bar elements
    QMenu *fileMenu;
    QMenu *helpMenu;

// Main UI action placeholders
    QAction *quitAct;
    QAction *aboutAct;
    QAction *aboutQtAct;
    QAction *aboutPCLAct;

// Main UI Widgets
    QWidget *mainWidget;
    // load target widgets
    QPushButton *browseTButton;
    QLineEdit *pathTField;
//    QPushButton *loadTButton;
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
//    QPushButton *loadSButton;
    // results widget
    QPushButton *startButton;
    QTextEdit *resultsList;
    // visualization buttons widgets
    QPushButton *showTButton;
    QPushButton *showSButton;
    QPushButton *showRButton;
    QPushButton *showTComponentButton;
    QComboBox *targetComponentsList;
    QPushButton *showSComponentButton;
    QComboBox *sourceComponentsList;
    QPushButton *clearAllButton;
    // visualization widget
    QVTKWidget *qvtkVisualizer;
    pcl::visualization::PCLVisualizer *viewer;

// Main UI Group Boxes
    QGroupBox *loadTBox;
    QGroupBox *componentsBox;
    QGroupBox *checksBox;
    QGroupBox *loadSBox;
    QGroupBox *resultsBox;
    QGroupBox *viewerControlsBox;

// Main UI Layout handlers
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

// Dialog UIs
    QDialog *addComponentDialog;
    boost::signals2::connection componentCallbackConnection;
    pcl::visualization::PCLVisualizer *dialogViewer;

    QDialog *addCheckDialog;

};

#endif // UI_H
