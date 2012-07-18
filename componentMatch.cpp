#include "componentMatch.h"

void componentMatch
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredSource,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud,
    pcl::PointIndices::Ptr targetComponentIndices,
    int clickedIndex,
    double cluThreshold,
    int colThreshold,
    pcl::PointIndices::Ptr sourceComponentIndices
)
{
    // Click simulation
    pcl::PointXYZRGB clickedPoint = targetCloud->at(clickedIndex); // extract the originally clicked point from the target cloud
    int sourceClickedPointIndex = registeredSource->size(); // the size of the cloud is the index of the next point that i add to it (che clicked point)
    registeredSource->push_back(clickedPoint); //aggiungo il punto cliccato, usandolo per la segmentazione

    // Component selection
    pcl::PointIndices::Ptr cluIndices(new pcl::PointIndices);
    segmentCluster(registeredSource, cluIndices, sourceClickedPointIndex, cluThreshold);
    pcl::PointIndices::Ptr colIndices(new pcl::PointIndices);
    segmentColor(registeredSource, colIndices, sourceClickedPointIndex, colThreshold);
    intersectIndices(cluIndices,colIndices,sourceComponentIndices);

    cout << "Size component on target cloud:"<<targetComponentIndices->indices.size()<<endl; // DEBUG
    cout << "Size component on source cloud:"<<sourceComponentIndices->indices.size()<<endl; // DEBUG
}
