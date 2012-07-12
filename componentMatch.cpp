#include "componentMatch.h"
#include "componentSelection.h"
#include <pcl/common/geometry.h>
#include <math.h>

void
matchComponent
(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud,
    pcl::PointIndices::Ptr inputComponentIndices,
    int clickedIndex,
    double cluThreshold,
    int colThreshold,
    pcl::PointIndices::Ptr outputComponentIndices
)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd (new pcl::search::KdTree<pcl::PointXYZRGB> ());
registerSourceToTarget(sourceCloud,targetCloud, registeredCloud,1,1);

pcl::PointXYZRGB clickedPoint = targetCloud->at(clickedIndex);
int guessedPointIndex=0;//parto dal primo punto della cloud
float distGuessedToRef=10000;//distanza iniziale infinita

//cerca il punto della source piu' vicino (dist&color) a quello cliccato nella target
for (int i=0; i<sourceCloud->size();i++)
    {
    bool matchColor=colorOk(clickedPoint,sourceCloud->at(i));
    float distCurrentToRef=dist(clickedPoint,sourceCloud->at(i));//pcl::geometry::distance(clickedPoint,sourceCloud->at(i));
    if (distCurrentToRef<distGuessedToRef  && matchColor)//current piu' vicino
        {
        guessedPointIndex=i;
        distGuessedToRef=dist(clickedPoint,sourceCloud->at(guessedPointIndex));//pcl::geometry::distance(clickedPoint,sourceCloud->at(guessedPointIndex));
        }
    }//end for

//component selection

//segmentCluster(registeredCloud, ,guessedPointIndex,cluThreshold);

}

//returns true if the color of the two points are similar (+-10 RGB)
bool colorOk(pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2){
    int threshold=10;
    if
        (
         abs(p1.r-p2.r)<threshold &&
         abs(p1.g-p2.g)<threshold &&
         abs(p1.b-p2.b)<threshold
        )
        return true;
    else
        return false;
}

float dist (pcl::PointXYZRGB &p1,pcl::PointXYZRGB &p2)
    {
    return sqrt(
                (p1.x-p2.x)*(p1.x-p2.x) +
                (p1.y-p2.y)*(p1.y-p2.y) +
                (p1.z-p2.z)*(p1.z-p2.z)
                );
    }
