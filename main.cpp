#include <QtGui/QApplication>

//PROJECT INCLUDES
#include "ui.h"
#include "registration.h"
#include "componentSelection.h"
#include "componentMatch.h"

//TESTING INCLUDES
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/interactor.h>
#include <vtkSmartPointer.h>
#include <QVTKWidget.h>

//DEBUG POINT PICK CALLBACK FUNCTION
//Attraverso il cookie bisogna passare un riferimento alla cloud(input)
//Bisognerà passare fouri in qualche modo gli indici del cluster per visualizzarli, bisognerà usare una classe con variabili condivise
//è una bestemmia fare tutto attraverso sto cookie. Oppure si passa dentro il viewer e si aggiunge la cloud da dentro qua boh...
void pointPickCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
    float x,y,z;
    if (event.getPointIndex() == -1)
        printf("No point was clicked\n");
    else
    {
        event.getPoint(x,y,z);
    }
    printf("Point Clicked index: %d x: %f y: %f z: %f \n", event.getPointIndex(), x, y, z);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ((pcl::PointCloud<pcl::PointXYZRGB>*)cookie);
//    pcl::PointIndices::Ptr clusterPoints;
//    segmentComponent(cloud, clusterPoints, event.getPointIndex(), 500);//troppo lento blocca tutto
//    printf("Cluster size: %d \n", clusterPoints->indices.size());
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);
    //Q_INIT_RESOURCE(dockwidgets); // per ora non uso risorse
    Ui ui;
    ui.show();
    return app.exec();

//    REGISTRATION TEST
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::io::loadPCDFile ("source.pcd", *source);
//    pcl::io::loadPCDFile ("target.pcd", *target);
//    int verbosity=1,compute=1;
//    cout << "Be verbose? "<<flush;
//    scanf ("%d",&verbosity);
//    cout << "Compute target? "<<flush;
//    scanf ("%d",&compute);
//    registerSourceToTarget(source, target, registered, verbosity, compute);

//    VISUALIZATION
//    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//    viewer.setBackgroundColor (0, 0, 0);
//    viewer.initCameraParameters ();
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(registered);
//    viewer.addPointCloud<pcl::PointXYZRGB> (registered, rgb, "source_registered");
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(target);
//    viewer.addPointCloud<pcl::PointXYZRGB> (target, rgb2, "target_reference");
//    viewer.registerPointPickingCallback (&pointPickCallback);

//    widget.show();
//    return app.exec();
}

