#include "pcqc.h"

Pcqc::Pcqc()
{
    targetCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    sourceCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    newComponentCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    colThreshold = 0;
    cluThreshold = 0;
    newComponentPointIndices.reset (new pcl::PointIndices);
    registeredCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
}

bool Pcqc::loadTargetCloud(QString path)
{
    const std::string stdpath = path.toStdString();
    if(pcl::io::loadPCDFile(stdpath, *targetCloud) == 0 )
    {
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*targetCloud, *targetCloud, indices);
        targetCloud=voxelCloud(targetCloud,0.4,1);
        segmentation(targetCloud,targetCloud,1);
        removeOutliers(targetCloud);
            return true;
    }
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getRegisteredCloud()
{
    return registeredCloud;
}

QColor* Pcqc::getPointColor(int pointIndex)
{
    QColor *color = new QColor;
    color->setRed(targetCloud->at(pointIndex).r);
    color->setGreen(targetCloud->at(pointIndex).g);
    color->setBlue(targetCloud->at(pointIndex).b);
    return color;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getNewComponentCloud()
{
    return newComponentCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getComponentCloud(QString componentName)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr componentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*targetCloud, componentsList.value(componentName), *componentCloud);
    return componentCloud;
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
void Pcqc::componentSelection(int selectedPointIndex)
{
    cout << "Component Segmentation... "<<flush; // DEBUG PRINT
    pcl::copyPointCloud(*targetCloud, *newComponentCloud); // start from a new copy of the cloud
    newComponentPointIndices->indices.clear();// and a new point indices

    pcl::PointIndices::Ptr tempClusterIndices(new pcl::PointIndices);
    segmentCluster(newComponentCloud, tempClusterIndices, selectedPointIndex, cluThreshold/1000 );

    pcl::PointIndices::Ptr tempColorIndices(new pcl::PointIndices);
    segmentColor(newComponentCloud, tempColorIndices, selectedPointIndex, colThreshold );

    // point indices intersection cycle
    while(!tempClusterIndices->indices.empty() && !tempColorIndices->indices.empty())
    {
        if(tempClusterIndices->indices.back()==tempColorIndices->indices.back())//trovato
        {
            newComponentPointIndices->indices.push_back(tempClusterIndices->indices.back()); //pusha dentro
            tempClusterIndices->indices.pop_back(); //passa al prossimo
        }
        else //poppa fuori il minimo
        {
            if(tempClusterIndices->indices.back()  <  tempColorIndices->indices.back())
                tempClusterIndices->indices.pop_back();
            else
                tempColorIndices->indices.pop_back();
        }
    }
    colorIndices(newComponentCloud, newComponentPointIndices);
    cout << "OK! Selected "<< newComponentPointIndices->indices.size() <<" points for this component\n"<<flush; // DEBUG PRINT
}

bool Pcqc::componentSegmentation()
{
    pcl::copyPointCloud(*targetCloud, *newComponentPointIndices, *newComponentCloud);
    return true;
}

bool Pcqc::componentSave(QString componentName)
{
    if(componentsList.find(componentName) == componentsList.end())
    {
        componentsList.insert(componentName, *newComponentPointIndices); // se non è già presente quella chiave, aggiungo il componente, altrimenti no
        return true;
    }
    else return false;
}

bool Pcqc::componentDelete(QString componentName)
{
    if(componentsList.find(componentName) != componentsList.end())
    {
        componentsList.remove(componentName); // se è presente quella chiave, cancello l'item, altrimenti no
        return true;
    }
    else return false;
}
