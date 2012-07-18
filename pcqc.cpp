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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::voxelCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double leafSize)
{
    cout << "Voxelling... "<<flush; //DEBUG
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (input);
    sor.setLeafSize (leafSize,leafSize,leafSize);
    sor.filter (*cloud_filtered);
    return cloud_filtered;
}

void Pcqc::segmentation (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented)
{
  cout << "Plane segmentation... " <<flush; //DEBUG
  // fit plane and keep points above that plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (3);
  seg.setInputCloud (source);
  seg.segment (*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (source);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*segmented);
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);
}

void Pcqc::removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cout << "Removing outliers... " << flush; //DEBUG
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud);
    cout << "OK! " << cloud->size() << " points Loaded."<<endl; //DEBUG
}

bool Pcqc::loadTargetCloud(QString path)
{
    const std::string stdpath = path.toStdString();
    if(pcl::io::loadPCDFile(stdpath, *targetCloud) == 0 )
    {
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*targetCloud, *targetCloud, indices);
        targetCloud=voxelCloud(targetCloud,0.4);
        segmentation(targetCloud,targetCloud);
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
        sourceCloud = voxelCloud(sourceCloud,0.4); // alleggerisce il calcolo, da capire se peggiora il risultato o meno.
        segmentation(sourceCloud,sourceCloud); // segmentazione del piano principale.
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Pcqc::getTargetComponentCloud(QString componentName)
{
    Component component = targetComponentsList.value(componentName);
    return component.getComponentCloud();
}

QMap<QString, Component>* Pcqc::getTargetComponentsList()
{
    return &targetComponentsList;
}

QMap<QString, Component>* Pcqc::getSourceComponentsList()
{
    return &sourceComponentsList;
}

//SETTERS
void Pcqc::setClusterSegThreshold(int threshold)
{
    cluThreshold = threshold/1000;
}

void Pcqc::setColorSegThreshold(int threshold)
{
    colThreshold = threshold;
}

//FUNCTIONS
void Pcqc::colorIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr indices, int r, int g, int b)
{
    for (int i =0; i<indices->indices.size(); i++)
    {
        int currentPointIndex= indices->indices.at(i);
        cloud->at(currentPointIndex).r=r;
        cloud->at(currentPointIndex).g=g;
        cloud->at(currentPointIndex).b=b;
    }
}

void Pcqc::colorComponents(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QMap<QString, Component> *components, int r, int g, int b)
{
    QMapIterator<QString, Component> iter(*components);
    Component currentComponent;
    while (iter.hasNext())
    {
        currentComponent = iter.next().value();
        colorIndices(cloud, currentComponent.getIndices(), r, g, b);
    }
}

void Pcqc::componentSelection(int selectedPointIndex)
{
    cout << "Component Segmentation... "<<flush; // DEBUG PRINT

    lastClickedPointIndex = selectedPointIndex; // save the last clicked point index
    pcl::copyPointCloud(*targetCloud, *newComponentCloud); // start from a new copy of the cloud
    newComponentPointIndices->indices.clear();// and a new point indices

    pcl::PointIndices::Ptr tempClusterIndices(new pcl::PointIndices);
    segmentCluster(newComponentCloud, tempClusterIndices, selectedPointIndex, cluThreshold ); // CLUSTER SEGMENTATION

    pcl::PointIndices::Ptr tempColorIndices(new pcl::PointIndices);
    segmentColor(newComponentCloud, tempColorIndices, selectedPointIndex, colThreshold ); // COLOR SEGMENTATION

    intersectIndices(tempClusterIndices,tempColorIndices,newComponentPointIndices);

    colorIndices(newComponentCloud, newComponentPointIndices,0,255,0);

    cout << "OK! Selected "<< newComponentPointIndices->indices.size() <<" points for this component\n"<<flush; // DEBUG PRINT
}

bool Pcqc::componentSegmentation()
{
    pcl::copyPointCloud(*targetCloud, *newComponentPointIndices, *newComponentCloud);
    return true;
}

bool Pcqc::componentSave(QString componentName)
{
    if(targetComponentsList.find(componentName) == targetComponentsList.end())
    {
        pcl::PointIndices::Ptr indicesToSave(new pcl::PointIndices); // create new pointer to a new point indices
        indicesToSave->indices = newComponentPointIndices->indices; // fill the new indices with the new component's indices
        Component newComponent(targetCloud, indicesToSave, lastClickedPointIndex, cluThreshold, colThreshold); // create a new component and fill it with information
        targetComponentsList.insert(componentName, newComponent); // instert the new component in the components list
        return true;
    }
    else return false;
}

bool Pcqc::componentDelete(QString componentName)
{
    if(targetComponentsList.find(componentName) != targetComponentsList.end())
    {
        targetComponentsList.remove(componentName); // se è presente quella chiave, cancello l'item, altrimenti no
        return true;
    }
    else return false;
}

int Pcqc::findSourceComponents()
{
    QMapIterator<QString,Component> iter(targetComponentsList);
    Component componentToFind;
    Component componentFound;
    while (iter.hasNext())
    {
        pcl::PointIndices::Ptr indicesFound (new pcl::PointIndices); // we need a new pointer every time, otherwise we would save only the last component
        componentToFind=iter.next().value();
        componentMatch(registeredCloud,
                       targetCloud,
                       componentToFind.getIndices(),
                       componentToFind.getGeneratingIndex(),
                       componentToFind.getClusterThreshold(),
                       componentToFind.getColorThreshold(),
                       indicesFound);
        componentFound = Component(registeredCloud, indicesFound);
        sourceComponentsList.insert(iter.key(), componentFound); // use the same key used in the target list
    }
//    colorIndices(registeredCloud,indicesFound,255,255,0);
}

void Pcqc::registration()
{
    registerSourceToTarget(sourceCloud, targetCloud, registeredCloud, 1, 0);
}
