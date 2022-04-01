import numpy as np
import os
import math
import open3d as o3d



def eulerAnglesToRotMatrixSelfRotate(theta) :
    
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
                       
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                                       
    R = np.dot(R_x, np.dot( R_y, R_z))

    return R


def getTransform(cam1_l1, cam1_l2, cam2_l1, cam2_l2, L, angle_cam1, angle_cam2):
#---------------------------------------------------------#

    cam1_l = cam1_l1 + cam1_l2
    cam2_l = cam2_l1 + cam2_l2

    T = np.array([(cam2_l-cam1_l), L, 0])

    thera = np.zeros(3)

    thera[0] = angle_cam1 + angle_cam2
    thera[2] = np.pi

    R = eulerAnglesToRotMatrixSelfRotate(thera)

    return R, T





def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    picked_points = vis.get_picked_points()

    return picked_points



if __name__ == '__main__':

    cloudroi = np.array([[-0.14, -0.13, 0.68],
                        [0.03, 0.11, 0.68]])

    #source_dir = "../11_Oct/cam1/pcd/1633939443632570.pcd"
    source_dir = "./"
    pcd_source = o3d.io.read_point_cloud(os.path.join(source_dir,"camrrc.pcd"))
    #o3d.visualization.draw_geometries([pcd_source])
    np_source = np.asarray(pcd_source.points)


    # GetROI=0 ROI predefined, 1: Get ROI by the function
    GetROI = 0
    if GetROI:
        #get roi point cloud
        picked_points = pick_points(pcd_source)
        leftupper = pcd_source.points[picked_points[0]]
        rightbottom = pcd_source.points[picked_points[1]]
        print("leftupper rightbottom:  ", leftupper,rightbottom)
    else:
        leftupper = cloudroi[0]
        rightbottom = cloudroi[1]
        print("start pcl processing:  ", leftupper,rightbottom)


    bounding_ploy = np.array([
                          [leftupper[0],leftupper[1], 0],
                          [rightbottom[0], leftupper[1], 0],
                          [rightbottom[0], rightbottom[1], 0],
                          [leftupper[0], rightbottom[1], 0]
                         ], dtype = np.float32).reshape([-1, 3]).astype("float64")

    bounding_polygon = np.array(bounding_ploy, dtype = np.float64)
    vol = o3d.visualization.SelectionPolygonVolume()

    #The Z-axis is used to define the height of the selected region
    vol.orthogonal_axis = "Z"
    vol.axis_max = np_source[:,2].max()
    vol.axis_min =np_source[:,2].min()

    vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
    pcd_food = vol.crop_point_cloud(pcd_source)
    print("crop roi point cloud ....")

    o3d.visualization.draw_geometries([pcd_food])

   





