#include <QtGui/QApplication>
#include "mainwindow.h"
#include "registration.h"
#include "componentSelection.h"
#include "componentMatch.h"
#include <pcl/visualization/pcl_visualizer.h>

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
//    workaround che non si può vedere, sistemerò al più presto
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ((pcl::PointCloud<pcl::PointXYZRGB>*)cookie);
    pcl::PointIndices::Ptr clusterPoints;
    segmentComponent(cloud, clusterPoints, event.getPointIndex(), 500);//troppo lento blocca tutto
    printf("Cluster size: %d \n", clusterPoints->indices.size());
}

int main(int argc, char *argv[])
{
//    WINDOW TEST
//    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();
//    return a.exec();

//    REGISTRATION TEST
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> *pippo = new pcl::PointCloud<pcl::PointXYZRGB>;//    workaround che non si può vedere, sistemerò al più presto
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (pippo);//    workaround che non si può vedere, sistemerò al più presto
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);//   condice "corretto"
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile ("source.pcd", *source);
    pcl::io::loadPCDFile ("target.pcd", *target);
    int verbosity=1,compute=1;
    cout << "Be verbose? "<<flush;
    scanf ("%d",&verbosity);
    cout << "Compute target? "<<flush;
    scanf ("%d",&compute);
    registerSourceToTarget(source, target, registered, verbosity, compute);

//    VISUALIZATION
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.initCameraParameters ();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(registered);
    viewer.addPointCloud<pcl::PointXYZRGB> (registered, rgb, "source_registered");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(target);
    viewer.addPointCloud<pcl::PointXYZRGB> (target, rgb2, "target_reference");
    viewer.registerPointPickingCallback (&pointPickCallback, pippo);
    viewer.spin();
}

