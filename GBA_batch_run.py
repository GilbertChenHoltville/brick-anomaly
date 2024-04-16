import GBA_construct_cloud
import GBA_pose_estimation

import json
import cv2

if __name__ == '__main__':
    DATA_ROOT_PATH = "./data/place_quality_inputs/"
    with open('./cfg/seg_res.json', 'r') as f:
        config_file = json.load(f)
    for single_config in config_file:
        brick_id = single_config["id"]
        res_seg_rect = single_config["seg_pts"] # xxyy
        path_rgb_img = DATA_ROOT_PATH + str(brick_id) + "/" + "color.png"
        path_depth_img = DATA_ROOT_PATH + str(brick_id) + "/" + "depth.png"
        path_cam_json = DATA_ROOT_PATH + str(brick_id) + "/" + "cam.json"
        rgb_img = cv2.imread(path_rgb_img)
        depth_img = cv2.imread(path_depth_img, cv2.IMREAD_UNCHANGED)
        with open(path_cam_json, 'r') as f:
            cam_json_file = json.load(f)
        depth_pixels = []
        for x in range(res_seg_rect[0], res_seg_rect[1] + 1):
            for y in range(res_seg_rect[2], res_seg_rect[3] +1):
                if depth_img[y][x] != 0:
                    depth_pixels.append([x, y, depth_img[y][x]])
        surface_world_coord = GBA_construct_cloud.depth_pixels_2_world_coords(cam_json_file, depth_pixels)
        plane_params = GBA_pose_estimation.fit_plane(surface_world_coord)
        # Ax + By + Cz + D = 0
        # z = (-Ax-By-D)/C
        # may fail, returning None
        [right_arrow, down_arrow, surface_center] = GBA_pose_estimation.Regression4pts(
            [
                (res_seg_rect[0],res_seg_rect[2]), (res_seg_rect[1],res_seg_rect[2]), 
                (res_seg_rect[1],res_seg_rect[3]), (res_seg_rect[0],res_seg_rect[3])
                ], 
            depth_img, cam_json_file
            )
        GBA_pose_estimation.PoseEst(right_arrow, down_arrow, surface_center)
