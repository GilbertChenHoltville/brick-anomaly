import numpy as np
import GBA_construct_cloud

def fit_plane(points):
    # Convert the list of points to a numpy array
    points_array = np.array(points)
    # Extract x, y, and z coordinates
    x = points_array[:, 0]
    y = points_array[:, 1]
    z = points_array[:, 2]
    # Create the A matrix for the least squares fit
    A = np.column_stack((x, y, np.ones_like(x)))
    # Solve the least squares problem
    coeffs, _, _, _ = np.linalg.lstsq(A, -z, rcond=None)
    # Extract coefficients
    A, B, D = coeffs
    # The equation of the plane is Ax + By + Dz + E = 0
    # where C = -1
    # To make it in the form Ax + By + Cz + D = 0, we need to multiply each coefficient by -1
    A, B, D = -A, -B, -D
    return [A, B, -1, D]  # Coefficients A, B, C, and D of the plane equation
def correct_depth(plane_params, x, y, z):
    # z = (-Ax-By-D)/C
    res = (-plane_params[0] * x - plane_params[1] * y - plane_params[3]) / plane_params[2]
    return res

def Regression4pts(_4pts, depth_img, json_file):
    # rtype: List[(x,y,z)], order LT RT RB LB
    #next version: sliding for anchor points
    [LT_asn, RT_asn, RB_asn, LB_asn] = _4pts # assigned 4 anchor points
    [b_LT_ori_pix_exist, b_RT_ori_pix_exist, b_RB_ori_pix_exist, b_LB_ori_pix_exist] = [False, False, False, False]
    if depth_img[LT_asn[1]][LT_asn[0]] != 0:
        b_LT_ori_pix_exist = True
        LT_world_coord = GBA_construct_cloud.pixel_to_world(json_file, LT_asn[0], LT_asn[1], depth_img[LT_asn[1]][LT_asn[0]])
        # print("LT", LT_world_coord)
    if depth_img[RT_asn[1]][RT_asn[0]] != 0:
        b_RT_ori_pix_exist = True
        RT_world_coord = GBA_construct_cloud.pixel_to_world(json_file, RT_asn[0], RT_asn[1], depth_img[RT_asn[1]][RT_asn[0]])
        # print("RT", RT_world_coord)
    if depth_img[RB_asn[1]][RB_asn[0]] != 0:
        b_RB_ori_pix_exist = True
        RB_world_coord = GBA_construct_cloud.pixel_to_world(json_file, RB_asn[0], RB_asn[1], depth_img[RB_asn[1]][RB_asn[0]])
        # print("RB", RB_world_coord)
    if depth_img[LB_asn[1]][LB_asn[0]] != 0:
        b_LB_ori_pix_exist = True
        LB_world_coord = GBA_construct_cloud.pixel_to_world(json_file, LB_asn[0], LB_asn[1], depth_img[LB_asn[1]][LB_asn[0]])
        # print("LB", LB_world_coord)
    if b_LT_ori_pix_exist and b_RT_ori_pix_exist:
        right_arrow = RT_world_coord - LT_world_coord
    elif b_LB_ori_pix_exist and b_RB_ori_pix_exist:
        right_arrow = RB_world_coord - LB_world_coord
    else:
        print("regression version 1 failed")
        return None
    if b_LT_ori_pix_exist and b_LB_ori_pix_exist:
        down_arrow = LB_world_coord - LT_world_coord
    elif b_RT_ori_pix_exist and b_RB_ori_pix_exist:
        down_arrow = RB_world_coord - RT_world_coord
    else:
        print("regression version 1 failed")
        return None
    if b_LT_ori_pix_exist:
        center = LT_world_coord + right_arrow/2. + down_arrow/2.
    elif b_RT_ori_pix_exist:
        center = RT_world_coord - right_arrow/2. + down_arrow/2.
    else:
        print("regression version 1 failed")
        return None
    # print("right_arrow", right_arrow)
    # print("down arrow", down_arrow)
    # print("center", center)
    return [right_arrow, down_arrow, center]

def PoseEst(vec1, vec2, center):
    # rtype:
    # Gram-Schmidt process to orthogonalize and normalize the vectors
    v1 = vec1 / np.linalg.norm(vec1)
    v2 = vec2 - np.dot(vec2, v1) * v1
    v2 = v2 / np.linalg.norm(v2)
    # Calculate the cross product to find the third orthogonal vector
    v3 = np.cross(v1, v2)
    # Construct the rotation matrix
    rotation_matrix = np.column_stack((v1, v2, v3))
    print(rotation_matrix, center)
    return rotation_matrix, center
