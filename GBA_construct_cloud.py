import numpy as np
import open3d as o3d
import cv2
import json

def rgb_depth_to_point_clouds(rgb_img, depth_img, json_file):
    # Convert the images to Open3D format
    rgb_o3d = o3d.geometry.Image(rgb_img)
    depth_o3d = o3d.geometry.Image(depth_img)
    # Create a RGBD image
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_o3d, depth_o3d)
    # Load the camera parameters from the JSON file
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        json_file['width'],
        json_file['height'],
        json_file['fx'],
        json_file['fy'],
        json_file['px'],
        json_file['py']
    )
    # Create a point cloud from the RGBD image
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        pinhole_camera_intrinsic)
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd

def pixel_to_world(camera_params, pixel_x, pixel_y, depth_value):
    fx = camera_params["fx"]
    fy = camera_params["fy"]
    px = camera_params["px"]
    py = camera_params["py"]
    x = (pixel_x - px) * depth_value / fx
    y = (pixel_y - py) * depth_value / fy
    z = depth_value
    return np.array([x, y, z])

def depth_pixels_2_world_coords(json_file, depth_pixels):
    # depth_pixels: list[x,y,d]
    # rtype: list of lists of coords, [[x, y, z]]
    world_coords = []
    for depth_pixel in depth_pixels:
        world_coord = pixel_to_world(json_file, depth_pixel[0], depth_pixel[1], depth_pixel[2])
        world_coords.append(world_coord)
    world_coords = np.array(world_coords)
    return world_coords


