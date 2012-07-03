#include "ui.h"

Ui::Ui(Pcqc *pcqc)
{
    motor = pcqc;
    createActions();
    setupMenuBar();
    setupStatusBar();
    setupMainLayout();
    setWindowTitle(tr("PCQC - Point Cloud Quality Control"));
    resize(1024,768);
}

Ui::~Ui()
{
    delete viewer;
    delete mainWidget; // maybe a redundant delete
    delete fileMenu; // maybe a redundant delete
    delete helpMenu; // maybe a redundant delete
}

// SLOT FUNCTIONS
void Ui::about()
{
   QMessageBox::about(this, tr("About pcl-exam-project"), tr("The <b>PCQC</b> is super awesome") );
}

void Ui::aboutPCL()
{
   QMessageBox::about(this, tr("About PCL"), tr("The Point Cloud Library (or PCL) is a large scale,"
                                                "open project for 3D point cloud processing.") );
}

void Ui::browseTarget()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Load Target Point Cloud"), "/home", tr("Point Clouds *.pcd (*.pcd);;All Files (*.*)"));
    pathTField->setText(fileName);
}

void Ui::loadTarget()
{
    if(motor->loadTargetCloud(pathTField->displayText()))
        statusBar()->showMessage(pathTField->displayText()+QString(" successfully loaded!"));
    else statusBar()->showMessage(QString("couldn't load the target point cloud, maybe the path or the filename are not correct."));
}

void Ui::browseSource()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Load Source Point Cloud"), "/home", tr("Point Clouds *.pcd (*.pcd);;All Files (*.*)"));
    pathSField->setText(fileName);
}

void Ui::loadSource()
{
    if(motor->loadSourceCloud(pathSField->displayText()))
        statusBar()->showMessage(pathSField->displayText()+QString(" successfully loaded!"));
    else statusBar()->showMessage(QString("couldn't load the source point cloud, maybe the path or the filename are not correct."));
}

void Ui::showTarget()
{
    if(!viewer->removePointCloud("target"))
    {
        viewer->addPointCloud<pcl::PointXYZRGB>(motor->getTargetCloud(), "target");
        statusBar()->showMessage(QString("Target point cloud added to the visualizer."));
    }
    else statusBar()->showMessage(QString("Target point cloud removed from the visualizer."));
    qvtkVisualizer->update();
}

void Ui::showSource()
{
    if(!viewer->removePointCloud("source"))
    {
        viewer->addPointCloud<pcl::PointXYZRGB>(motor->getSourceCloud(), "source");
        statusBar()->showMessage(QString("Source point cloud added to the visualizer."));
    }
    else statusBar()->showMessage(QString("Source point cloud removed from the visualizer."));
    qvtkVisualizer->update();
}

void Ui::clearAll()
{
    viewer->removeAllPointClouds();
}

void Ui::openComponentDialog()
{
    addComponentDialog = new QDialog(this); // set as child of Ui, to be sure that it will be deleted in the end.
//    QVBoxLayout *dialogLayout = new QVBoxLayout; // create vertical layout
//    QVTKWidget *dialogVisualizer = new QVTKWidget; // create qvtk widget
//    pcl::visualization::PCLVisualizer *dialogViewer = new pcl::visualization::PCLVisualizer("Dialog Viewer", false);
//    dialogVisualizer->SetRenderWindow(dialogViewer->getRenderWindow()); // set as render window the render window of the dialog visualizer
//    dialogViewer->setupInteractor(dialogVisualizer->GetInteractor(), dialogVisualizer->GetRenderWindow()); // tells the visualizer what interactor is using now and for what window
//    dialogViewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT); // ripristina input system of original visualizer (shift+click for points)
//    dialogViewer->addPointCloud<pcl::PointXYZRGB>(motor->getTargetCloud(), "target");
//    dialogViewer->setBackgroundColor(0, 0, 0);
//    dialogViewer->initCameraParameters();
////    dialogViewer->registerPointPickingCallback(&pointPickCallback, this); // TO DO: IMPOSTARE NUOVA CALLBACK DEDICATA PER GESTIRE L'AGGIUNTA DI UN COMPONENTE
//    QLineEdit *addComponentDialogName  = new QLineEdit(QString("Insert Component Name"));
//    QHBoxLayout *addComponentDialogSegLayout = new QHBoxLayout;
//    QPushButton *selectPointSegButton = new QPushButton(QString("Select Point"));
//    //connect
//    QSlider *setSegThresholdBar = new QSlider(Qt::Horizontal);
//    //connect
//    QPushButton *showSegButton = new QPushButton(QString("Segment!"));
//    //connect
//    QHBoxLayout *addComponentDialogColLayout = new QHBoxLayout;
//    QPushButton *selectPointColButton = new QPushButton(QString("Select Point"));
//    //connect
//    QTableWidget *colorBox = new QTableWidget(1, 1);
//    QColor *selectedColor = new QColor(255, 0, 0, 255); // initializer color at black
//    colorBox->item(0,0)->setBackgroundColor(*selectedColor);
//    QSlider *setColThresholdBar = new QSlider(Qt::Horizontal);
//    //connect
//    QPushButton *showColButton = new QPushButton(QString("Segment!"));
//    //connect
//    QPushButton *saveComponent = new QPushButton(QString("Add component to component list"));
//    saveComponent->setDefault(true); //default button, pressed if enter is pressed
//    //connect

//    addComponentDialogSegLayout->addWidget(selectPointSegButton);
//    addComponentDialogSegLayout->addWidget(setSegThresholdBar);
//    addComponentDialogSegLayout->addWidget(showSegButton);
//    addComponentDialogColLayout->addWidget(selectPointColButton);
//    addComponentDialogColLayout->addWidget(colorBox);
//    addComponentDialogColLayout->addWidget(setColThresholdBar);
//    addComponentDialogColLayout->addWidget(showColButton);
//    dialogLayout->addWidget(dialogVisualizer);
//    dialogLayout->addWidget(addComponentDialogName);
//    dialogLayout->addLayout(addComponentDialogSegLayout);
//    dialogLayout->addLayout(addComponentDialogColLayout);
//    dialogLayout->addWidget(saveComponent);
//    addComponentDialog->setLayout(dialogLayout);
    addComponentDialog->deleteLater(); // delete dialog whet it is closed
    addComponentDialog->exec(); // se Ui rimane bloccato non solo nell'interfaccia usare show() che è non bloccante
    // finita l'esecuzione, deleta tutto (forse non serve?!)

}

void Ui::openCheckDialog()
{

}

// TO DO: create slot functions for every action (every button)

// MAIN UI MENU ACTIONS
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
}

// LAYOUT FUNCTIONS
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
    connect(browseTButton, SIGNAL(clicked()), this, SLOT(browseTarget()));
    pathTField = new QLineEdit();
    loadTButton = new QPushButton(QString("LOAD!"));
    connect(loadTButton, SIGNAL(clicked()), this, SLOT(loadTarget()));
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
    connect(addComponentButton, SIGNAL(clicked()), this, SLOT(openComponentDialog()));
    delComponentButton = new QPushButton(QString("Delete"));
    connect(delComponentButton, SIGNAL(clicked()), this, SLOT(openCheckDialog()));
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
    connect(browseSButton, SIGNAL(clicked()), this, SLOT(browseSource()));
    pathSField = new QLineEdit();
    loadSButton = new QPushButton(QString("LOAD!"));
    connect(loadSButton, SIGNAL(clicked()), this, SLOT(loadSource()));
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

    // workaround per posizionare la camera sulla zona delle cloud :D
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prova (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile ("target.pcd", *prova);
    viewer->addPointCloud<pcl::PointXYZRGB>(prova, "prova");
//    viewer->removeAllPointClouds();
    //delete prova

    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->registerPointPickingCallback(&pointPickCallback, this); // callback function for interaction with the mouse on the visualizer
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
    connect(showTButton, SIGNAL(clicked()), this, SLOT(showTarget()));
    showSButton = new QPushButton(QString("Show/Hide Source Cloud"));
    connect(showSButton, SIGNAL(clicked()), this, SLOT(showSource()));
    clearAllButton = new QPushButton(QString("Clear All Clouds"));
    connect(clearAllButton, SIGNAL(clicked()), this, SLOT(clearAll()));
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
    mainWidget = new QWidget;
    mainLayout = new QHBoxLayout;
    viewerLayout = new QVBoxLayout;
    commandsLayout = new QVBoxLayout;

    setupLoadTBox();
    setupComponentsBox();
    setupChecksBox();
    setupLoadSBox();
    setupVisualizer();
    setupResultsBox();
    setupVisualizerCommands();

    commandsLayout->addWidget(loadTBox);
    commandsLayout->addWidget(componentsBox);
    commandsLayout->addWidget(checksBox);
    commandsLayout->addWidget(loadSBox);

    viewerLayout->addWidget(qvtkVisualizer);
    viewerLayout->addWidget(resultsBox);
    viewerLayout->addWidget(showTButton);
    viewerLayout->addWidget(showSButton);
    viewerLayout->addLayout(showTargetComponentLayout);
    viewerLayout->addLayout(showSourceComponentLayout);
    viewerLayout->addWidget(clearAllButton);

    mainLayout->addLayout(commandsLayout);
    mainLayout->addLayout(viewerLayout);

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
