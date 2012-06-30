#include "ui.h"

Ui::Ui()
{
    setupMainWidget();
    setupDockWidgets();
    setupStatusBar();
    setWindowTitle(tr("pcl-exam-project"));
}

void Ui::setupMainWidget()
{
    qvtkVisualizer = new QVTKWidget();
    viewer = new pcl::visualization::PCLVisualizer("3DViewer", false);// don't display in the vtk visualizer, render it on a qt window
    qvtkVisualizer->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(qvtkVisualizer->GetInteractor(), qvtkVisualizer->GetRenderWindow());// tells the viewer what interactor and what window is using now
    viewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);// ripristina input system of original visualizer (shift+click for points)
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
//    viewer.registerPointPickingCallback(&pointPickCallback);
    setCentralWidget(qvtkVisualizer);
}

void Ui::setupDockWidgets()
{

}

void Ui::setupStatusBar()
{

}
