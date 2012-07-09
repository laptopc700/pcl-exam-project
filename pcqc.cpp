#include "pcqc.h"

Pcqc::Pcqc()
{
    targetCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    sourceCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    targetCloudComponentSeg.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    targetCloudColorSeg.reset (new pcl::PointCloud<pcl::PointXYZRGB>); // TO DELETE
    colThreshold = 0;
    targetCloudClusterSeg.reset (new pcl::PointCloud<pcl::PointXYZRGB>); // TO DELETE
    cluThreshold = 0;
    newComponentPointIndices.reset (new pcl::PointIndices);
    registeredCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
}

bool Pcqc::loadTargetCloud(QString path)
{
    const std::string stdpath = path.toStdString();
    if(pcl::io::loadPCDFile(stdpath, *targetCloud) == 0 ){
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*targetCloud, *targetCloud, indices);

        targetCloud=voxelCloud(targetCloud,0.4,1);
        segmentation(targetCloud,targetCloud,1);
        removeOutliers(targetCloud);
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
        sourceCloud = voxelCloud(sourceCloud,0.4,1); // alleggerisce il calcolo, da capire se peggiora il risultato o meno.
        segmentation(sourceCloud,sourceCloud,1); // segmentazione del piano principale.
        removeOutliers(sourceCloud);
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

QColor* Pcqc::getPointColor(int pointIndex)
{
    QColor *color = new QColor;
    color->setRed(targetCloud->at(pointIndex).r);
    color->setGreen(targetCloud->at(pointIndex).g);
    color->setBlue(targetCloud->at(pointIndex).b);
    return color;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getTargetCloudColorSeg()
{
    return targetCloudColorSeg;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getTargetCloudClusterSeg()
{
    return targetCloudClusterSeg;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getTargetCloudComponentSeg()
{
    return targetCloudComponentSeg;
}
//SETTERS

void Pcqc::setClusterSegThreshold(int threshold)
{
    cluThreshold = threshold;
}

void Pcqc::setColorSegThreshold(int threshold)
{
    colThreshold = threshold;
}

//FUNCTIONS

void Pcqc::componentSegmentation(int selectedPointIndex)
{ cout << "Component Segmentation... "<<flush;
        //start from a new copy of the cloud
        pcl::copyPointCloud(*targetCloud, *targetCloudComponentSeg);
        pcl::PointIndices::Ptr tempClusterIndices(new pcl::PointIndices);
        segmentCluster(targetCloudComponentSeg, tempClusterIndices, selectedPointIndex, cluThreshold/1000 );
        pcl::PointIndices::Ptr tempColorIndices(new pcl::PointIndices);
        segmentColor(targetCloudComponentSeg, tempColorIndices, selectedPointIndex, colThreshold );


        while(!tempClusterIndices->indices.empty())
        {
            if(tempClusterIndices->indices.back()==tempColorIndices->indices.back())//trovato
            {
                newComponentPointIndices->indices.push_back(tempClusterIndices->indices.back());
                tempClusterIndices->indices.pop_back(); //poppa da cluster
            }
            else tempColorIndices->indices.pop_back(); //poppa da color
        }

            // To DO:||||||||||||||||||||||| migliorare questo che e' quadratico ||||||||||||||||||||||
//        for (int i=0 ;i<tempClusterIndices->indices.size(); i++)
//            for (int j=0 ;j<tempColorIndices->indices.size(); j++)
//                if (tempClusterIndices->indices.at(i) == tempColorIndices->indices.at(j))
//                {
//                    newComponentPointIndices->indices.push_back(tempClusterIndices->indices.at(i));
//                    break;
//                }

        colorIndices(targetCloudComponentSeg, newComponentPointIndices);
        cout << "OK!\n"<<flush;
}

void Pcqc::clusterSegmentation(int selectedPointIndex, bool isFirstStep)//TODELETE
{
    if(isFirstStep)
    {
        //start from a new copy of the cloud
        pcl::copyPointCloud(*targetCloud, *targetCloudClusterSeg);
        segmentCluster(targetCloudClusterSeg, newComponentPointIndices, selectedPointIndex, cluThreshold/1000 );
        colorIndices(targetCloudClusterSeg, newComponentPointIndices);
    }
    else
    {
        //start from the result of the first step: the color segmented cloud (it can't be otherwise)
        pcl::copyPointCloud(*targetCloudColorSeg, *targetCloudClusterSeg);
        segmentCluster(targetCloudClusterSeg, newComponentPointIndices, selectedPointIndex, cluThreshold/1000 );
        colorIndices(targetCloudClusterSeg, newComponentPointIndices);
    }
}

void Pcqc::colorSegmentation(int selectedPointIndex, bool isFirstStep)//TODELETE
{
    if(isFirstStep)
    {
        //start from a new copy of the cloud
        pcl::copyPointCloud(*targetCloud, *targetCloudColorSeg);
        segmentColor(targetCloudColorSeg, newComponentPointIndices, selectedPointIndex, colThreshold );
        colorIndices(targetCloudColorSeg, newComponentPointIndices);
    }
    else
    {
        //start from the result of the first step: the cluster segmented cloud (it can't be otherwise)
        pcl::copyPointCloud(*targetCloudClusterSeg, *targetCloudColorSeg);
        segmentColor(targetCloudColorSeg, newComponentPointIndices, selectedPointIndex, colThreshold );
        colorIndices(targetCloudColorSeg, newComponentPointIndices);
    }
}
