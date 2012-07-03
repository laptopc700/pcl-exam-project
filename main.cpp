#include <QtGui/QApplication>

//PROJECT INCLUDES
#include "ui.h"
#include "pcqc.h"

//DEBUG POINT PICK CALLBACK FUNCTION
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
    //Q_INIT_RESOURCE(dockwidgets); // per ora non uso risorse (icone&immagini)
    Pcqc pcqc;
    Ui ui(&pcqc);
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
//    viewer.spin();
}


