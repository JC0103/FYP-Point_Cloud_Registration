#References: http://www.open3d.org/docs/release/tutorial/geometry/file_io.html

import numpy as np
from open3d import *    

def main():
    cloud = open3d.io.read_point_cloud("/home/jcchia/Pictures/FYP/cam2/pcd/broccoli/broccoli_1.pcd") # Read the point cloud
    open3d.visualization.draw_geometries([cloud]) # Visualize the point cloud     

if __name__ == "__main__":
    main()
