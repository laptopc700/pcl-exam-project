#include "ui.h"

//PUBLIC FUNCTIONS
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
    delete mainLayout; // maybe a redundant delete
    delete mainWidget; // maybe a redundant delete
    delete fileMenu; // maybe a redundant delete
    delete helpMenu; // maybe a redundant delete
}

Pcqc* Ui::getMotor()
{
    return motor;
}

pcl::visualization::PCLVisualizer* Ui::getViewer()
{
    return viewer;
}

pcl::visualization::PCLVisualizer* Ui::getDialogViewer()
{
    return dialogViewer;
}

QVTKWidget* Ui::getViewerWidget()
{
    return qvtkVisualizer;
}

QDialog* Ui::getComponentDialog()
{
    return addComponentDialog;
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
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(motor->getTargetCloud());
        viewer->addPointCloud<pcl::PointXYZRGB>(motor->getTargetCloud(), rgb, "target");
        viewer->resetCamera();
        statusBar()->showMessage(QString("Target point cloud added to the visualizer."));
    }
    else statusBar()->showMessage(QString("Target point cloud removed from the visualizer."));
    qvtkVisualizer->update();
}

void Ui::showSource()
{
    if(!viewer->removePointCloud("source"))
    {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(motor->getSourceCloud());
        viewer->addPointCloud<pcl::PointXYZRGB>(motor->getSourceCloud(), rgb, "source");
        viewer->resetCamera();
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
    QVBoxLayout *dialogLayout = new QVBoxLayout; // create vertical layout

    QVTKWidget *dialogVisualizer = new QVTKWidget; // create qvtk widget
    dialogViewer = new pcl::visualization::PCLVisualizer("Dialog Viewer", false);
    dialogVisualizer->SetRenderWindow(dialogViewer->getRenderWindow()); // set as render window the render window of the dialog visualizer
    dialogViewer->setupInteractor(dialogVisualizer->GetInteractor(), dialogVisualizer->GetRenderWindow()); // tells the visualizer what interactor is using now and for what window
    dialogViewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT); // ripristina input system of original visualizer (shift+click for points)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = motor->getTargetCloud();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(temp);
    dialogViewer->addPointCloud<pcl::PointXYZRGB>(temp, rgb, "cloud");
    dialogViewer->setBackgroundColor(0.5, 0.5, 0.5);
    dialogViewer->initCameraParameters();
    dialogViewer->resetCamera();
    componentCallbackConnection = dialogViewer->registerPointPickingCallback(&pointPickCallback, this); // callback standard non segmenta nulla

    QLineEdit *addComponentDialogName  = new QLineEdit("Insert Component Name");
    addComponentDialogName->setObjectName("componentname");

    QHBoxLayout *dialogControlsLayout = new QHBoxLayout;

    QVBoxLayout *buttonsBox = new QVBoxLayout;
    QPushButton *selectPointButton = new QPushButton("Select Component");
    connect(selectPointButton, SIGNAL(clicked()), this, SLOT(setComponentDialogCallback()));
    QPushButton *resetButton = new QPushButton("Reset Selection");
    connect(resetButton, SIGNAL(clicked()), this, SLOT(resetComponentDialogCallback()));
    buttonsBox->addWidget(selectPointButton);
    buttonsBox->addWidget(resetButton);

    QVBoxLayout *numbersBox = new QVBoxLayout;
    QColor *selectedColor = new QColor(0, 0, 0, 255); // initialize color at black
    QPushButton *colorBox = new QPushButton("+-0");
    colorBox->setObjectName("colorbox");
    colorBox->setStyleSheet(colorToStyleSheet(selectedColor));
    colorBox->setMaximumWidth(50);
    QLineEdit *clusterBox = new QLineEdit("0");
    clusterBox->setReadOnly(true);
    clusterBox->setObjectName("clusterbox");
    clusterBox->setMaxLength(5);
    clusterBox->setMaximumWidth(50);
    numbersBox->addWidget(clusterBox);
    numbersBox->addWidget(colorBox);

    QVBoxLayout *slidersBox = new QVBoxLayout;
    QSlider *setCluThresholdBar = new QSlider(Qt::Horizontal);
    setCluThresholdBar->setRange(0,5000);
    setCluThresholdBar->setObjectName("sliderCluster");
    connect(setCluThresholdBar, SIGNAL(sliderReleased()), this, SLOT(setClusterThreshold()));
    QSlider *setColThresholdBar = new QSlider(Qt::Horizontal);
    setColThresholdBar->setRange(0,255);
    setColThresholdBar->setObjectName("sliderColor");
    connect(setColThresholdBar, SIGNAL(sliderReleased()), this, SLOT(setColorThreshold()));
    slidersBox->addWidget(setCluThresholdBar);
    slidersBox->addWidget(setColThresholdBar);

    QPushButton *showSegButton = new QPushButton("Segment!");
    connect(showSegButton, SIGNAL(clicked()), this, SLOT(segmentComponent()));

    dialogControlsLayout->addLayout(buttonsBox);
    dialogControlsLayout->addLayout(numbersBox);
    dialogControlsLayout->addLayout(slidersBox);
    dialogControlsLayout->addWidget(showSegButton);

    QPushButton *saveComponent = new QPushButton("Save to component list");
    saveComponent->setDefault(true); // default button, pressed if enter is pressed
    connect(saveComponent, SIGNAL(clicked()), this, SLOT(saveComponent()));

    dialogLayout->addWidget(dialogVisualizer);
    dialogLayout->addWidget(addComponentDialogName);
    dialogLayout->addLayout(dialogControlsLayout);
    dialogLayout->addWidget(saveComponent);
    addComponentDialog->setLayout(dialogLayout);

    // DIALOG EXECUTION
//    addComponentDialog->deleteLater(); // delete dialog when the control returns to the event loop from which deleteLater() was called (after exec i guess)
//    causa seg fault se viene attivata la callback

    dialogLayout->deleteLater(); // delete dialog layout when the control returns to the event loop from which deleteLater() was called (after exec i guess)
    addComponentDialog->resize(800,600);
    addComponentDialog->exec();
    componentCallbackConnection.disconnect(); // disconnect the callback function from the viewer
    delete dialogViewer; // finita l'esecuzione, deallocare il viewer (deallocare altra eventuale memoria non indirizzata nel QObject tree).
}

void Ui::setComponentDialogCallback()
{
    dialogViewer->registerPointPickingCallback(&pointPickCallbackSelectComponent, this);
    //TO DO: cambia cursore
}

void Ui::resetComponentDialogCallback()
{
    dialogViewer->registerPointPickingCallback(&pointPickCallback, this);
    dialogViewer->updatePointCloud(motor->getTargetCloud(),"cloud");
    //TO DO: reset cursore
}

void Ui::setClusterThreshold()
{
    QSlider *slider= addComponentDialog->findChild<QSlider *>("sliderCluster");
    motor->setClusterSegThreshold(slider->value());
    QLineEdit *clusterbox = addComponentDialog->findChild<QLineEdit *>("clusterbox");
    clusterbox->setText(QString("%1").arg((float)slider->value() / 1000));
}

void Ui::setColorThreshold()
{
    QSlider *slider= addComponentDialog->findChild<QSlider *>("sliderColor");
    motor->setColorSegThreshold(slider->value());
    QPushButton *colorbox = addComponentDialog->findChild<QPushButton *>("colorbox");
    colorbox->setText(QString("+-%1").arg(slider->value()));
}

void Ui::segmentComponent()
{
    if(motor->componentSegmentation())
        statusBar()->showMessage("Segmented new component.");
    else statusBar()->showMessage("Couldn't segment this selection.");
    dialogViewer->updatePointCloud(motor->getNewComponentCloud(),"cloud");
}

void Ui::saveComponent()
{
    QLineEdit *componentname = addComponentDialog->findChild<QLineEdit *>("componentname");
    if( motor->componentSave(componentname->text()) )
        statusBar()->showMessage("Component successfully saved!");
    else statusBar()->showMessage("Couldn't save this component.");
}

void Ui::openCheckDialog()
{
    addCheckDialog = new QDialog(this); // set as child of Ui, to be sure that it will be deleted in the end.
    QVBoxLayout *dialogLayout = new QVBoxLayout;
    QTabWidget *tabWidget = new QTabWidget();

    QWidget *presence = new QWidget();
    QVBoxLayout *presenceLayout = new QVBoxLayout();
    QLineEdit *presenceName = new QLineEdit(QString("Insert check name"));
    QComboBox *presenceComp = new QComboBox();
    QLineEdit *presenceThreshold = new QLineEdit(QString("Insert error threshold (number of points difference)"));
    QPushButton *presenceAdd = new QPushButton(QString("Add Check"));
    //connect
    presenceLayout->addWidget(presenceName);
    presenceLayout->addWidget(presenceComp);
    presenceLayout->addWidget(presenceThreshold);
    presenceLayout->addWidget(presenceAdd);
    presence->setLayout(presenceLayout);

    QWidget *planarity = new QWidget();
    QVBoxLayout *planarityLayout = new QVBoxLayout();
    QLineEdit *planarityName = new QLineEdit(QString("Insert check name"));
    QComboBox *planarityComp = new QComboBox();
    QLineEdit *planarityThreshold = new QLineEdit(QString("Insert error threshold (angular inclination)"));
    QPushButton *planarityAdd = new QPushButton(QString("Add Check"));
    //connect
    planarityLayout->addWidget(planarityName);
    planarityLayout->addWidget(planarityComp);
    planarityLayout->addWidget(planarityThreshold);
    planarityLayout->addWidget(planarityAdd);
    planarity->setLayout(planarityLayout);

    QWidget *pose = new QWidget();
    QVBoxLayout *poseLayout = new QVBoxLayout();
    QLineEdit *poseName = new QLineEdit(QString("Insert check name"));
    QComboBox *poseComp = new QComboBox();
    QLineEdit *poseThreshold = new QLineEdit(QString("Insert error threshold (angular inclination)"));
    QPushButton *poseAdd = new QPushButton(QString("Add Check"));
    //connect
    poseLayout->addWidget(poseName);
    poseLayout->addWidget(poseComp);
    poseLayout->addWidget(poseThreshold);
    poseLayout->addWidget(poseAdd);
    pose->setLayout(poseLayout);

    QWidget *color = new QWidget();
    QVBoxLayout *colorLayout = new QVBoxLayout();
    QLineEdit *colorName = new QLineEdit(QString("Insert check name"));
    QComboBox *colorComp = new QComboBox();
    QLineEdit *colorThreshold = new QLineEdit(QString("Insert error threshold (color difference in r g b values)"));
    QPushButton *colorAdd = new QPushButton(QString("Add Check"));
    //connect
    colorLayout->addWidget(colorName);
    colorLayout->addWidget(colorComp);
    colorLayout->addWidget(colorThreshold);
    colorLayout->addWidget(colorAdd);
    color->setLayout(colorLayout);

    QWidget *distance = new QWidget();
    QVBoxLayout *distanceLayout = new QVBoxLayout();
    QLineEdit *distanceName = new QLineEdit(QString("Insert check name"));
    QComboBox *distanceComp = new QComboBox();
    QComboBox *distanceComp2 = new QComboBox();
    QLineEdit *distanceThreshold = new QLineEdit(QString("Insert error threshold (distance difference)"));
    QPushButton *distanceAdd = new QPushButton(QString("Add Check"));
    //connect
    distanceLayout->addWidget(distanceName);
    distanceLayout->addWidget(distanceComp);
    distanceLayout->addWidget(distanceComp2);
    distanceLayout->addWidget(distanceThreshold);
    distanceLayout->addWidget(distanceAdd);
    distance->setLayout(distanceLayout);

    QWidget *height = new QWidget();
    QVBoxLayout *heightLayout = new QVBoxLayout();
    QLineEdit *heightName = new QLineEdit(QString("Insert check name"));
    QComboBox *heightComp = new QComboBox();
    QLineEdit *heightThreshold = new QLineEdit(QString("Insert error threshold (height difference)"));
    QPushButton *heightAdd = new QPushButton(QString("Add Check"));
    //connect
    heightLayout->addWidget(heightName);
    heightLayout->addWidget(heightComp);
    heightLayout->addWidget(heightThreshold);
    heightLayout->addWidget(heightAdd);
    height->setLayout(heightLayout);

    QWidget *circleRadius = new QWidget();
    QVBoxLayout *circleLayout = new QVBoxLayout();
    QLineEdit *circleRadiusName = new QLineEdit(QString("Insert check name"));
    QComboBox *circleRadiusComp = new QComboBox();
    QLineEdit *circleRadiusThreshold = new QLineEdit(QString("Insert error threshold (circle radius difference)"));
    QPushButton *circleRadiusAdd = new QPushButton(QString("Add Check"));
    //connect
    circleLayout->addWidget(circleRadiusName);
    circleLayout->addWidget(circleRadiusComp);
    circleLayout->addWidget(circleRadiusThreshold);
    circleLayout->addWidget(circleRadiusAdd);
    circleRadius->setLayout(circleLayout);

    QWidget *cable = new QWidget();
    QVBoxLayout *cableLayout = new QVBoxLayout();
    QLineEdit *cableName = new QLineEdit(QString("Insert check name"));
    QComboBox *cableComp = new QComboBox();
    QLineEdit *cableLengthThreshold = new QLineEdit(QString("Insert error threshold (cable length difference)"));
    //definire punti di routing e connessione, forse servirebbe un visualizzatore in questo caso
    QPushButton *cableAdd = new QPushButton(QString("Add Check"));
    //connect
    cableLayout->addWidget(cableName);
    cableLayout->addWidget(cableComp);
    cableLayout->addWidget(cableLengthThreshold);
    cableLayout->addWidget(cableAdd);
    cable->setLayout(cableLayout);

    tabWidget->addTab(presence, QString("Presence"));
    tabWidget->addTab(planarity, QString("Planarity"));
    tabWidget->addTab(pose, QString("Pose"));
    tabWidget->addTab(color, QString("Color"));
    tabWidget->addTab(distance, QString("Distance"));
    tabWidget->addTab(height, QString("Height"));
    tabWidget->addTab(circleRadius, QString("Circle Radius"));
    tabWidget->addTab(cable, QString("Cable"));

    dialogLayout->addWidget(tabWidget);
    addCheckDialog->setLayout(dialogLayout);
    addCheckDialog->deleteLater(); // delete dialog when the control returns to the event loop from which deleteLater() was called (after exec i guess)
    dialogLayout->deleteLater(); // delete dialog layout when the control returns to the event loop from which deleteLater() was called (after exec i guess)
    addCheckDialog->resize(640, 240);
    addCheckDialog->exec();
}

// TO DO: create slot functions for every action (every button)
// TO DO: create slot functions for every action (every button)
// TO DO: create slot functions for every action (every button)
// TO DO: create slot functions for every action (every button)

// UI FUNCTIONS
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
    //connect
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
    connect(addCheckButton, SIGNAL(clicked()), this, SLOT(openCheckDialog()));
    delCheckButton = new QPushButton(QString("Delete"));
    //connect
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
    viewer->setBackgroundColor(0.5, 0.5, 0.5);
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

void Ui::pointPickCallbackSelectComponent(const pcl::visualization::PointPickingEvent& event, void* cookie)
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
        QPushButton *colorbox = ui->getComponentDialog()->findChild<QPushButton *>("colorbox");
        colorbox->setStyleSheet(colorToStyleSheet(ui->getMotor()->getPointColor(event.getPointIndex())));
        ui->getMotor()->componentSelection(event.getPointIndex());
        ui->getDialogViewer()->updatePointCloud(ui->getMotor()->getNewComponentCloud(),"cloud");
    }
}

QString Ui::colorToStyleSheet(QColor *color)
{
    QString styleSheet = QString("* { background-color: rgb(%1,%2,%3) }")
                                    .arg(color->red())
                                    .arg(color->green())
                                    .arg(color->blue());
    return styleSheet;
}
