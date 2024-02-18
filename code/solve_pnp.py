from est_homography import est_homography
import numpy as np

def PnP(Pc, Pw, K=np.eye(3)):
    """
    Estimate camera pose using Perspective-n-Point (PnP) algorithm.

    Input:
        Pc: 4x2 numpy array of pixel coordinates of the April tag corners in (x, y) format
        Pw: 4x3 numpy array of world coordinates of the April tag corners in (x, y, z) format
        K: 3x3 camera intrinsic matrix (default is identity matrix)

    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3, ) numpy array describing camera translation in the world (t_wc)
    """
    
    # Homography Approach
    Hval = est_homography(Pw[:,:-1], Pc)
    Hprime = np.linalg.inv(K) @ Hval
    matrix_ab = Hprime[:, :2]
    
    # Singular Value Decomposition (SVD)
    U, S, V = np.linalg.svd(matrix_ab, full_matrices=False)
    
    # Extract rotation and translation
    s1, s2 = S[0], S[1]
    lam = (s1 + s2) / 2
    r = U @ V
    t = Hprime[:, 2] / lam
    r1, r2 = r[:, 0], r[:, 1]
    cross = np.cross(r1, r2)
    R = np.array([r1, r2, cross])
    t = -R @ t

    return R, t
