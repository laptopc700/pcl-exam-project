#include "pcqc.h"

Pcqc::Pcqc()
{
    targetCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    targetCloudTemp.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    sourceCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    registeredCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
}

bool Pcqc::loadTargetCloud(QString path)
{
    const std::string stdpath = path.toStdString();
    if(pcl::io::loadPCDFile(stdpath, *targetCloud) == 0 ){
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*targetCloud, *targetCloud, indices);

        targetCloud=voxelCloud(targetCloud,1,1);
        segmentation(targetCloud,targetCloud,1);
            return true;}
    else return false;
}

bool Pcqc::loadSourceCloud(QString path)
{
    const std::string stdpath = path.toStdString();
    if(pcl::io::loadPCDFile(stdpath, *sourceCloud) == 0)
    {
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*sourceCloud, *sourceCloud, indices);
        sourceCloud = voxelCloud(sourceCloud,1,1); // alleggerisce il calcolo, da capire se peggiora il risultato o meno.
        segmentation(sourceCloud,sourceCloud,1); // segmentazione del piano principale.
        return true;
    }
    else return false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getTargetCloud()
{
    return targetCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getSourceCloud()
{
    return sourceCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getTargetCloudTemp()
{
    pcl::copyPointCloud(*targetCloud, *targetCloudTemp);
    return targetCloudTemp;
}
