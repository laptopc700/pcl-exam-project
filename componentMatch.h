#ifndef COMPONENTMATCH_H
#define COMPONENTMATCH_H

#include "componentSelection.h"

using namespace std;

// searches the correspondent of a given target component in the registeredSource cloud, simulating the click of the component selection and returns its indices in outputComponentIndices
void componentMatch
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredSource,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud,
    pcl::PointIndices::Ptr targetComponentIndices,
    int clickedIndex,
    double cluThreshold,
    int colThreshold,
    pcl::PointIndices::Ptr sourceComponentIndices
);

bool colorOk(pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2); // cosa fa questo metodo?

float dist(pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2); // dafuq?

#endif // COMPONENTMATCH_H
