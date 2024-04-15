import numpy as np

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

