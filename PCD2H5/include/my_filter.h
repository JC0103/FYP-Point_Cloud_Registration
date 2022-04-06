#ifndef MY_FILTER_H
#define MY_FILTER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "my_io.h"

using namespace std;

class MyFilter{
public:
    MyFilter();
    ~MyFilter();

    int filterPointClouds(const string &pcd_dir, const string &pcd_names_file, const string &write_dir);
    int Resample(const string &pcd_names_file);
private:


};



#endif