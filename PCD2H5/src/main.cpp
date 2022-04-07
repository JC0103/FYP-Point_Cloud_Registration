
#include <iostream>
#include <string>
#include <time.h>
#include <fstream>
#include "my_io.h"
#include "my_filter.h"

using namespace std;

int main(int argc, char** argv){
    MyFilter my_filter;
    MyIO my_io;
    string h5_filename = "trainset1.h5";
    bool two_clouds = false;

    
    if (two_clouds){
        my_filter.filterPointClouds(first_src_pcd_dir, "pcd_names.txt", second_src_pcd_dir);
        my_filter.filterPointClouds(first_ref_pcd_dir, "pcd_names.txt", second_ref_pcd_dir);
        my_io.combinePCDsAndLabelsIntoH5File(h5_filename, second_src_pcd_dir, 
                                            second_ref_pcd_dir,
                                            "pcd_names.txt", 
                                            second_src_pcd_dir + "labels_file.txt");
    }

    else{
        my_filter.filterPointClouds(first_src_pcd_dir, "pcd_names.txt", second_src_pcd_dir);
        my_io.combinePCDsAndLabelsIntoH5File(h5_filename, second_src_pcd_dir,
                                            "pcd_names.txt",  second_src_pcd_dir + "labels_file.txt");    }
    
    cout << "data is writed to " << h5_filename << endl;
    return 0;
}