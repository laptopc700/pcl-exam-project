#include <QtGui/QApplication>
#include "mainwindow.h"
#include "registration.h"
#include "componentSelection.h"
#include "componentMatch.h"
#include <pcl/visualization/pcl_visualizer.h>

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile ("source.pcd", *source);
    pcl::io::loadPCDFile ("target.pcd", *target);
    int verbosity=1,compute=1;
    cout << "Be verbose? "<<flush;
    scanf ("%d",&verbosity);
    cout << "Compute target? "<<flush;
    scanf ("%d",&compute);
    registerSourceToTarget(source, target, registered, verbosity, compute);

//    VisualizationXYZRGB
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.initCameraParameters ();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(registered);
    viewer.addPointCloud<pcl::PointXYZRGB> (registered, rgb, "source_registered");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(target);
    viewer.addPointCloud<pcl::PointXYZRGB> (target, rgb2, "target_reference");
    viewer.registerPointPickingCallback (&pointPickCallback, 0);
    while (!viewer.wasStopped ())	{viewer.spin ();}


}

