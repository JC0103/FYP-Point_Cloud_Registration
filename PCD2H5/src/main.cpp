
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
    
    my_filter.filterPointClouds("eclair/pcd_names.txt");
    // my_io.combinePCDsAndLabelsIntoH5File(h5_filename, "eclair/pcd_names.txt", "eclair/labels_file.txt");
 
    // cout << "data is writed to " << h5_filename << endl;
    return 0;
}