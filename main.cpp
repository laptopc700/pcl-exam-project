#include <QtGui/QApplication>
#include "mainwindow.h"
#include "registration.h"


int main(int argc, char *argv[])
{
//    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();
    
//    return a.exec();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile ("source.pcd", *source);
  pcl::io::loadPCDFile ("target.pcd", *target);
int verbosity=1,compute=0;
/*cout << "Be verbose? "<<flush;
scanf ("%d",&verbosity);
cout << "Compute target? "<<flush;
scanf ("%d",&compute);*/
registerSourceToTarget(source, target, registered, verbosity, compute);


    // VisualizationXYZRGB
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.initCameraParameters ();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(registered);
    viewer.addPointCloud<pcl::PointXYZRGB> (registered, rgb, "source_registered");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(target);
    viewer.addPointCloud<pcl::PointXYZRGB> (target, rgb2, "target_reference");
    while (!viewer.wasStopped ())	{viewer.spin ();}


}

