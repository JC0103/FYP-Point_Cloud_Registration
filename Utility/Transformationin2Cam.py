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


def voxel_filter(point_cloud,leaf_size,mode ='random'):
    #Build voxel grid
    x_max ,y_max ,z_max = np.max(point_cloud,axis=0)
    x_min ,y_min ,z_min = np.min(point_cloud,axis=0)
    D_x = (x_max - x_min)/leaf_size
    D_y = (y_max - y_min)/leaf_size
    D_z = (z_max - z_min)/leaf_size
    
    #Obtain corresponding voxel grid of every points
    point_cloud = np.asarray(point_cloud)
    h = []
    for i in range(point_cloud.shape[0]):
        h_x = np.floor((point_cloud[i][0]-x_min)/leaf_size)
        h_y = np.floor((point_cloud[i][1]-y_min)/leaf_size)
        h_z = np.floor((point_cloud[i][2]-z_min)/leaf_size)
        H = h_x + h_y * D_x + h_z *D_x *D_y
        h.append(H)
        
    #Sort the points according to the voxel grid ID
    h = np.asarray(h)
    voxel_index = np.argsort(h)
    h_sort = h[voxel_index]

    #random
    if mode == 'random':
        filtered_points = []
        index_begin = 0
        for i in range(len(voxel_index)-1):
            if h_sort[i] == h_sort[i+1]:
                continue
            else:
                point_index = voxel_index[index_begin:(i+1)]
                random_index = np.random.choice(point_index)
                random_choice = point_cloud[random_index]
                filtered_points.append(random_choice)
                index_begin = i
    #centroid
    if mode == 'centroid':
        filtered_points = []
        index_begin = 0
        for i in range(len(voxel_index)-1):
            if h_sort[i] == h_sort[i+1]:
                continue
            else:
                point_index = voxel_index[index_begin:(i+1)]
                filtered_points.append(np.mean(point_cloud[point_index],axis=0))
                index_begin = i
    filtered_points = np.array(filtered_points, dtype=np.float64)
    return filtered_points




if __name__ == '__main__':

    cloudroi = np.array([[-0.14, -0.13, 0.68],
                        [0.03, 0.11, 0.68]])

    source_dir = "/home/jcchia/Pictures/FYP/cam1/pcd/second_ver/"
    write_dir = "/home/jcchia/Pictures/FYP/cam1/pcd/1st_preprocess/"
    with open (source_dir + 'pcd_names.txt') as f:
        line = f.readline
        while line:
            line = f.readline().strip()
            print(source_dir + line)
            pcd_source = o3d.io.read_point_cloud(source_dir + line)
            #o3d.visualization.draw_geometries([pcd_source])
            np_source = np.asarray(pcd_source.points)


            # GetROI=0 ROI predefined, 1: Get ROI by the function
            GetROI = 1
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
            print("number of points: ", len(pcd_food.points))

            filtered_cloud = voxel_filter(pcd_food.points, 0.007)
            print("type: ", type(filtered_cloud))

            point_cloud_filtered = o3d.geometry.PointCloud()
            point_cloud_filtered.points = o3d.utility.Vector3dVector(filtered_cloud)
            
            print("number of points after filter: ", len(point_cloud_filtered.points))
            o3d.io.write_point_cloud(write_dir + line, point_cloud_filtered)
            o3d.visualization.draw_geometries([pcd_food])
            o3d.visualization.draw_geometries([point_cloud_filtered])


   





