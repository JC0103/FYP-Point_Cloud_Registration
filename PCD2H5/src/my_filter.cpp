#include "my_filter.h"

MyFilter::MyFilter(){

}

MyFilter::~MyFilter(){

}

int MyFilter::filterPointClouds(const string &pcd_names_file){
    ifstream in;
    in.open(pcd_names_file);
    if (!in.is_open())
        return -1;

    string textline;

    while(getline(in, textline)){
        PCT::Ptr raw_cloud(new PCT);
        PCT::Ptr filtered_cloud (new PCT);
        PCT::Ptr cleaned_cloud (new PCT);
    

        pcl::io::loadPCDFile(textline, *raw_cloud);
        raw_cloud -> is_dense = false;

        // Filter out nan point data
        cout << textline << " Raw point cloud size: " << raw_cloud->points.size() << endl;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*raw_cloud, *filtered_cloud, indices);
        cout << textline << " Filtered point cloud size: " << filtered_cloud->points.size() << endl;

        // Filter outliers
        pcl::StatisticalOutlierRemoval<PT> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cleaned_cloud);
        cout << textline << " Cleaned (inliers) point cloud size: " << cleaned_cloud->points.size() << endl;


        // Save proccessed data files
        pcl::io::savePCDFileASCII("/home/jcchia/FYP/PCD2H5/filtered/" + textline , *cleaned_cloud);

        // sor.setNegative (true);
        // sor.filter(*cleaned_cloud);
        // pcl::io::savePCDFileASCII("/home/jcchia/FYP/PCD2H5/filtered/" + textline , *cleaned_cloud);
    }
    return 0;
}