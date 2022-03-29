#ifndef MYIO_H
#define MYIO_H
#include "H5Cpp.h"
 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
 
#include <iostream>
#include <string>
#include <cstdio>
#include <fstream>
 
using namespace std;
 
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PCT;
typedef pcl::PCLPointCloud2 PC2;
typedef pcl::Normal PN;
typedef pcl::PointCloud<PN> PCN;
typedef pcl::NormalEstimation<PT, PN> PTN;
 
class MyIO
{
public:
    MyIO();
    ~MyIO();
 
    int markDownStoredPCDNameAndItsLabel(const string &pcd_name, const int &label, const string &pcd_names_file, const string &labels_file);
    int combinePCDsAndLabelsIntoH5File(const string &h5_file,  const string &pcd_names_file, const string &labels_file);
    int readFileAndCountHowManyClouds(const string &pcd_names_file);
 
private:
    int readPCDs(const string &pcd_names_file, float *data, const unsigned int &pt_num);
    int estimateNormals(const string &pcd_names_file, float *normal_array, const unsigned int &pt_num);
    int readLabels(const string &labels_file, int *data);

 
};
 
#endif // MYIO_H

