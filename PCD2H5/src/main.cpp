
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
    string h5_filename = "two_clouds.h5";
    string pcd_names_file = "test_pcd_names.txt";
    string labels_file = "test_labels.file.txt";

    bool two_clouds = true;
    
    if (two_clouds){
        my_filter.filterPointClouds(first_src_pcd_dir, pcd_names_file, second_src_pcd_dir);
        my_filter.filterPointClouds(first_ref_pcd_dir, pcd_names_file, second_ref_pcd_dir);
        my_io.combinePCDsAndLabelsIntoH5File(h5_filename, second_src_pcd_dir, 
                                            second_ref_pcd_dir,
                                            pcd_names_file, 
                                            second_src_pcd_dir + labels_file);
    }

    else{
        my_filter.filterPointClouds(first_src_pcd_dir, pcd_names_file, second_src_pcd_dir);
        my_io.combinePCDsAndLabelsIntoH5File(h5_filename, second_src_pcd_dir,
                                            pcd_names_file,  second_src_pcd_dir + labels_file);    }
    
    cout << "data is writed to " << h5_filename << endl;
    return 0;
}