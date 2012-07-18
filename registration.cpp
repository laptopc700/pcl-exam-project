#include "registration.h"

using namespace std;

//#################################################################################################
//########################################################   F  U  N  C  T  I  O  N  S   ##########
//#################################################################################################


void
changeColor (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, int r, int g, int b){
    for (size_t i = 0; i < input->size(); ++i){
        (*input)[i].r=r;
        (*input)[i].g=g;
        (*input)[i].b=b;
    }
}

void detectKeypoints ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, int verbosity)
{

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    sift3D->setScales(0.1, 3, 2);//0.1
    sift3D->setMinimumContrast(0.0);
    sift3D->setSearchMethod(tree);
    if (verbosity) cout << "Keypoint detection..." << flush;
    sift3D->setInputCloud(input);
    sift3D->compute(*keypoints);
    if (verbosity) cout << "OK! Keypoints found: " << keypoints->points.size() << endl;
}

void extractDescriptors ( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr features, int verbosity)
{
    pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr feature_extractor_ (new pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
    feature_extractor_->setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    feature_extractor_->setRadiusSearch (25);//25

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
    kpts->points.resize(keypoints->points.size());

    pcl::copyPointCloud(*keypoints, *kpts);
    //passo da XYZI a XYZRGB
    pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> > (feature_extractor_);
    feature_extractor_->setInputCloud(kpts);
    //mettere come input le keypoints, non la cloud
    if (verbosity) cout << "Normals estimation and Descriptors extraction... " << flush;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    normal_estimation.setRadiusSearch (20);//20
    normal_estimation.setInputCloud (kpts);//input, ho messo kpts per accelerare
    normal_estimation.compute (*normals);
    feature_from_normals->setInputNormals(normals);
    feature_extractor_->compute (*features);
    if (verbosity) cout << "OK! Features found: " << features->points.size() << endl;
}

void findCorrespondences (
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr source,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr target,
        vector<int>& correspondences, int verbosity)
{
    if (verbosity) cout << "Finding correspondences..." << flush;
    correspondences.resize (source->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    vector<int> k_indices (k);
    vector<float> k_squared_distances (k);
    for (size_t i = 0; i < source->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
    }
    if (verbosity) cout << "OK! Correspondences found: " << correspondences.size() << endl;
}

void filterCorrespondences ( pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_ ,
                             vector<int>& source2target_ , vector<int>& target2source_ ,
                             pcl::CorrespondencesPtr correspondences_, int verbosity)
{
    if (verbosity) cout << "Filtering correspondences..." << flush;
    vector<pair<unsigned, unsigned> > correspondences;
    for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
        if (target2source_[source2target_[cIdx]] == cIdx)
            correspondences.push_back(make_pair(cIdx, source2target_[cIdx]));

    correspondences_->resize (correspondences.size());
    for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
    {
        (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
        (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
    }

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
    rejector.setInputCloud(source_keypoints_);
    rejector.setTargetCloud(target_keypoints_);
    rejector.setMaxIterations(50);
    rejector.setInlierThreshold(2);
    rejector.setInputCorrespondences(correspondences_);
    rejector.getCorrespondences(*correspondences_);
    if (verbosity) cout << "OK! Correspondences filtered: " << correspondences.size()<< endl;
}

Eigen::Matrix4f determineInitialTransformation (
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_ ,
        pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_ ,
        pcl::CorrespondencesPtr correspondences_,
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_segmented_ ,
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_transformed_,
        int verbosity
        )
{
    if (verbosity) cout << "Initial matrix calculation..." << flush;
    Eigen::Matrix4f initial_transformation_matrix_ = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);
    transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);
    if (verbosity) cout << "OK" << endl;
//    pcl::transformPointCloud(*source_segmented_, *source_transformed_, initial_transformation_matrix_);
    return initial_transformation_matrix_;
}

Eigen::Matrix4f determineFinalTransformation (
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_transformed_ ,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_registered_,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_segmented_,
        int verbosity
        )
{
    if (verbosity) cout << "Final matrix calculation..." << flush;
    Eigen::Matrix4f final_transformation_matrix_=Eigen::Matrix4f::Identity ();
    pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
    registration->setInputCloud(source_transformed_);
    registration->setInputTarget (target_segmented_);
    registration->setMaxCorrespondenceDistance(4); //4
    registration->setRANSACOutlierRejectionThreshold (4); //4
    registration->setTransformationEpsilon (0.0001); //0.0001
    registration->setMaximumIterations (100); //1
    registration->align(*source_registered_);
    final_transformation_matrix_ = registration->getFinalTransformation();
    if (verbosity) cout << "OK" << endl;
    return final_transformation_matrix_;
}

//#################################################################################################
//##############################################################   R E G I S T E R   ##############
//#################################################################################################
int
registerSourceToTarget (
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source ,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered,
        int verbosity, int compute_target)
{
    time_t start,end;
    time (&start);

    //per cancellare i warnings
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    //sift3d keypoint detector
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
    detectKeypoints (source, source_keypoints, verbosity);
    if(compute_target) {
        detectKeypoints (target, target_keypoints, verbosity);
        pcl::io::savePCDFileBinary("correct/target_keypoints.pcd",*target_keypoints);}
    else pcl::io::loadPCDFile("correct/target_keypoints.pcd",*target_keypoints);

    //FPFH feature extractor
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features (new pcl::PointCloud<pcl::FPFHSignature33>);
    extractDescriptors (source, source_keypoints, source_features, verbosity);
    if(compute_target) {
        extractDescriptors (target, target_keypoints, target_features, verbosity);
        pcl::io::savePCDFileBinary("correct/target_features.pcd",*target_features);}
    else pcl::io::loadPCDFile("correct/target_features.pcd",*target_features);

    //Find Correspondences
    vector<int> source2target ;
    vector<int> target2source ;
    findCorrespondences (source_features, target_features, source2target, verbosity);
    findCorrespondences (target_features, source_features, target2source, verbosity);

    //Filter Correnspondences
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);
    filterCorrespondences (source_keypoints, target_keypoints , source2target , target2source, correspondences, verbosity);

    //Initial transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix4f initial_transformation_matrix = determineInitialTransformation (source_keypoints, target_keypoints, correspondences,
                                                                                    /*source, source_transformed,*/ verbosity);
    if (verbosity) cout << "Source cloud initial alignment..." << flush;
    pcl::transformPointCloud(*source, *source_transformed, initial_transformation_matrix);
    if (verbosity) cout << "OK" << endl;

    //Final transformation
//    Eigen::Matrix4f final_transformation_matrix = determineFinalTransformation (source_transformed , registered, target, verbosity);
    pcl::copyPointCloud(*source_transformed, *registered);  //per non star li a cambiare le variabili se si attiva la final transformation

    //questa trasformazione non serve, la fa già registration->align riga 148
//    if (verbosity) cout << "Source cloud final alignment..." << flush;
//    pcl::transformPointCloud(*registered, *registered, final_transformation_matrix);
//    if (verbosity) cout << "OK" << endl;

//    changeColor(source, 255, 0, 0);
//    changeColor(target, 0, 0, 255);
//    changeColor(registered,0, 255, 0);
//    if (verbosity) cout << "Restored "<< source->points.size() << " points source"<< endl;
//    if (verbosity) cout << "Restored "<< target->points.size() << " points target"<< endl;
//    if (verbosity) cout << "REGISTERED "<< registered->points.size() << " points"<< endl;


    time (&end);
    double dif = difftime (end,start);
    if (verbosity) cout << "------------------------------------->  Registered in " << dif << " seconds"<< endl;
}
