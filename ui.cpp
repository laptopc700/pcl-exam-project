#include "ui.h"

Ui::Ui()
{
    createActions();
    setupMenuBar();
    setupStatusBar();
    setupMainLayout();
    setWindowTitle(tr("PCQC - Point Cloud Quality Control"));
    resize(1024,768);
}

void Ui::about()
{
   QMessageBox::about(this, tr("About pcl-exam-project"), tr("The <b>pcl-exam-project</b> is super awesome") );
}

void Ui::aboutPCL()
{
   QMessageBox::about(this, tr("About PCL"), tr("The Point Cloud Library (or PCL) is a large scale,"
                                                "open project for 3D point cloud processing.") );
}

// TO DO: functions ofr every action (every button)

void Ui::createActions()
{
    // Menu Bar Actions
    quitAct = new QAction(tr("&Quit"), this);
    quitAct->setShortcuts(QKeySequence::Quit);
    quitAct->setStatusTip(tr("Quit the application"));
    connect(quitAct, SIGNAL(triggered()), this, SLOT(close()));

    aboutAct = new QAction(tr("&About"), this);
    aboutAct->setStatusTip(tr("Show the application's About box"));
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

    aboutQtAct = new QAction(tr("About &Qt"), this);
    aboutQtAct->setStatusTip(tr("Show the Qt library's About box"));
    connect(aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()));

    aboutPCLAct = new QAction(tr("About &PCL"), this);
    aboutPCLAct->setStatusTip(tr("Show the PCL library's About box"));
    connect(aboutPCLAct, SIGNAL(triggered()), this, SLOT(aboutPCL()));

    //TO DO: actions for every button.
}

void Ui::setupMenuBar()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(quitAct);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
    helpMenu->addAction(aboutQtAct);
    helpMenu->addAction(aboutPCLAct);
}

void Ui::setupStatusBar()
{
    statusBar()->showMessage(tr("Ready"));
}

void Ui::setupLoadTBox()
{
    loadTBox = new QGroupBox(QString("Load Target Cloud"));
    browseTButton = new QPushButton(QString("Browse..."));
    pathTField = new QLineEdit();
    loadTButton = new QPushButton(QString("LOAD!"));
    loadTargetLayout = new QHBoxLayout;
    loadTargetLayout->addWidget(browseTButton);
    loadTargetLayout->addWidget(pathTField);
    loadTargetLayout->addWidget(loadTButton);
    loadTBox->setLayout(loadTargetLayout);
}

void Ui::setupComponentsBox()
{
    componentsBox = new QGroupBox(QString("Components Definition"));
    componentButtonsLayout = new QHBoxLayout;
    addComponentButton = new QPushButton(QString("Add..."));
    delComponentButton = new QPushButton(QString("Delete"));
    componentButtonsLayout->addWidget(addComponentButton);
    componentButtonsLayout->addWidget(delComponentButton);
    componentsLayout = new QVBoxLayout;
    componentsList = new QListWidget;
    componentsLayout->addLayout(componentButtonsLayout);
    componentsLayout->addWidget(componentsList);
    componentsBox->setLayout(componentsLayout);
}

void Ui::setupChecksBox()
{
    checksBox = new QGroupBox(QString("Checks Definition"));
    checkButtonsLayout = new QHBoxLayout;
    addCheckButton = new QPushButton(QString("Add..."));
    delCheckButton = new QPushButton(QString("Delete"));
    checkButtonsLayout->addWidget(addCheckButton);
    checkButtonsLayout->addWidget(delCheckButton);
    checksLayout = new QVBoxLayout;
    checksList = new QListWidget;
    checksLayout->addLayout(checkButtonsLayout);
    checksLayout->addWidget(checksList);
    checksBox->setLayout(checksLayout);
}

void Ui::setupLoadSBox()
{
    loadSBox = new QGroupBox(QString("Load Source Cloud"));
    browseSButton = new QPushButton(QString("Browse..."));
    pathSField = new QLineEdit();
    loadSButton = new QPushButton(QString("LOAD!"));
    loadSourceLayout = new QHBoxLayout;
    loadSourceLayout->addWidget(browseSButton);
    loadSourceLayout->addWidget(pathSField);
    loadSourceLayout->addWidget(loadSButton);
    loadSBox->setLayout(loadSourceLayout);
}

void Ui::setupVisualizer()
{
    qvtkVisualizer = new QVTKWidget();// create qvtk widget
    viewer = new pcl::visualization::PCLVisualizer("3DViewer", false);// don't display in the vtk visualizer, render it on a qt window
    qvtkVisualizer->SetRenderWindow(viewer->getRenderWindow());// set as render window the render window of the pcl visualizer
    viewer->setupInteractor(qvtkVisualizer->GetInteractor(), qvtkVisualizer->GetRenderWindow());// tells the visualizer what interactor is using now and for what window
    viewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);// ripristina input system of original visualizer (shift+click for points)
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->registerPointPickingCallback(&pointPickCallback, this); // callback function for interaction with the mous on the visualizer
}

void Ui::setupResultsBox()
{
    resultsBox = new QGroupBox(QString("Analysis Results"));
    startButton = new QPushButton(QString("START!"));
    resultsList = new QListWidget;
    resultsLayout = new QHBoxLayout;
    resultsLayout->addWidget(startButton);
    resultsLayout->addWidget(resultsList);
    resultsBox->setLayout(resultsLayout);
}

void Ui::setupVisualizerCommands()
{
    showTButton = new QPushButton(QString("Show/Hide Target Cloud"));
    showSButton = new QPushButton(QString("Show/Hide Source Cloud"));
    clearAll = new QPushButton(QString("Clear All Clouds"));
    showTComponentButton = new QPushButton(QString("Show/Hide Target Component"));
    targetComponentsList = new QComboBox;
    showTargetComponentLayout = new QHBoxLayout;
    showTargetComponentLayout->addWidget(showTComponentButton);
    showTargetComponentLayout->addWidget(targetComponentsList);
    showSComponentButton = new QPushButton(QString("Show/Hide Source Component"));
    sourceComponentsList = new QComboBox;
    showSourceComponentLayout = new QHBoxLayout;
    showSourceComponentLayout->addWidget(showSComponentButton);
    showSourceComponentLayout->addWidget(sourceComponentsList);
}

void Ui::setupMainLayout()
{
    setupLoadTBox();
    setupComponentsBox();
    setupChecksBox();
    setupLoadSBox();
    setupVisualizer();
    setupResultsBox();
    setupVisualizerCommands();

    commandsLayout = new QVBoxLayout;
    commandsLayout->addWidget(loadTBox);
    commandsLayout->addWidget(componentsBox);
    commandsLayout->addWidget(checksBox);
    commandsLayout->addWidget(loadSBox);

    viewerLayout = new QVBoxLayout;
    viewerLayout->addWidget(qvtkVisualizer);
    viewerLayout->addWidget(resultsBox);
    viewerLayout->addWidget(showTButton);
    viewerLayout->addWidget(showSButton);
    viewerLayout->addLayout(showTargetComponentLayout);
    viewerLayout->addLayout(showSourceComponentLayout);

    mainLayout = new QHBoxLayout;
    mainLayout->addLayout(commandsLayout);
    mainLayout->addLayout(viewerLayout);
    mainWidget = new QWidget;
    mainWidget->setLayout(mainLayout);
    setCentralWidget(mainWidget);
}

void Ui::pointPickCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
    Ui *ui = (Ui*)cookie;
    float x,y,z;
    if (event.getPointIndex() == -1)
        ui->statusBar()->showMessage(tr("No point was clicked"));
    else
    {
        event.getPoint(x,y,z);
        ui->statusBar()->showMessage(QString("Point Clicked index: %1 x: %2 y: %3 z: %4")
                                 .arg(event.getPointIndex())
                                 .arg(x)
                                 .arg(y)
                                 .arg(z)
                                 );
    }
}
