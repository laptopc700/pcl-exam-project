#include "componentSelection.h"


vector<int> segmentComponent (
pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
int selectedPointIndex,
double threshold
        )
{
vector<int> indexes;
return indexes;
}



vector<int>
segmentColor (
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
        int selectedPointIndex,
        int threshold
        )

{
    vector<int> indexes;
    pcl::PointXYZRGB selectedPoint = input[selectedPointIndex];
    int r = selectedPoint.r;
    int g = selectedPoint.g;
    int b = selectedPoint.b;

    for (int i = 0; i < input->size(); i++){
        if(
                abs((*input)[i].r-r)<threshold &&
                abs((*input)[i].g-g)<threshold &&
                abs((*input)[i].b-b)<threshold
                )
        indexes.push_back(i);
    }

    return indexes;
    }
