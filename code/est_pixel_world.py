import numpy as np

def est_pixel_world(pixels, R_wc, t_wc, K):
    """
    Estimate the world coordinates of a point given a set of pixel coordinates.
    The points are assumed to lie on the x-y plane in the world.
    Input:
        pixels: N x 2 coordiantes of pixels
        R_wc: (3, 3) Rotation of camera in world
        t_wc: (3, ) translation from world to camera
        K: 3 x 3 camara intrinsics
    Returns:
        Pw: N x 3 points, the world coordinates of pixels
    """


    N = pixels.shape[0]
    H_Point = np.hstack((pixels, np.ones((N, 1))))

    # Here calculate the inverse of the rotation matrix and tranlation matrix
    R_inv = np.linalg.inv(R_wc)
    t_inv = -np.dot(R_inv, t_wc.reshape(3, 1))

    # Create the [R t] matrix
    R_matrix = np.hstack((R_inv[:, :2], t_inv))

    # Initialize the world coordinates
    Pw = np.zeros([H_Point.shape[0], 3])

    # Calculate world coordinates for each pixel
    for i in range(pixels.shape[0]):
        Pw[i, :] = np.dot(np.linalg.inv(K @ R_matrix), H_Point[i, :].reshape(3, 1)).flatten()
        Pw[i, :] = Pw[i, :] / Pw[i, -1]
        Pw[i, -1] = 0


    return Pw    
