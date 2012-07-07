#include "pcqc.h"

Pcqc::Pcqc()
{
    targetCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    targetCloudTemp.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    sourceCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    targetCloudColorSeg.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    int colThreshold = 0;
    targetCloudClusterSeg.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    int cluThreshold = 0;
    newComponentPointIndices.reset (new pcl::PointIndices);
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

//GETTERS
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getTargetCloudColorSeg()
{
    return targetCloudColorSeg;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getTargetCloudClusterSeg()
{
    return targetCloudClusterSeg;
}

//SETTERS
void Pcqc::setClusterSegThreshold(int threshold)
{
    cluThreshold = threshold;
}

void Pcqc::setColorSwgThreshold(int threshold)
{
    colThreshold = threshold;
}

//FUNCTIONS
void Pcqc::clusterSegmentation(int selectedPointIndex, int threshold, bool isFirstStep)
{
    if(isFirstStep)
    {
        //start from a new copy of the cloud
        pcl::copyPointCloud(*targetCloud, *targetCloudClusterSeg);
        segmentCluster(targetCloudClusterSeg, newComponentPointIndices, selectedPointIndex, threshold );
        colorIndices(targetCloudClusterSeg, newComponentPointIndices);
    }
    else
    {
        //start from the result of the first step: the color segmented cloud (it can't be otherwise)
        pcl::copyPointCloud(*targetCloudColorSeg, *targetCloudClusterSeg);
        segmentCluster(targetCloudClusterSeg, newComponentPointIndices, selectedPointIndex, threshold );
        colorIndices(targetCloudClusterSeg, newComponentPointIndices);
    }

}

void Pcqc::colorSegmentation(int selectedPointIndex, int threshold, bool isFirstStep)
{
    if(isFirstStep)
    {
        //start from a new copy of the cloud
        pcl::copyPointCloud(*targetCloud, *targetCloudColorSeg);
        segmentColor(targetCloudColorSeg, newComponentPointIndices, selectedPointIndex, threshold );
        colorIndices(targetCloudColorSeg, newComponentPointIndices);
    }
    else
    {
        //start from the result of the first step: the cluster segmented cloud (it can't be otherwise)
        pcl::copyPointCloud(*targetCloudClusterSeg, *targetCloudColorSeg);
        segmentCluster(targetCloudColorSeg, newComponentPointIndices, selectedPointIndex, threshold );
        colorIndices(targetCloudColorSeg, newComponentPointIndices);
    }
}
