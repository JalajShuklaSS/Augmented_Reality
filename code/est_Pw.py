import numpy as np

def est_Pw(s):
    """
    Estimates the world coordinates of the April tag corners, assuming the world origin
    is at the center of the tag, and that the xy plane is in the plane of the April
    tag with the z axis in the tag's facing direction. See world_setup.jpg for details.
    Input:
        s: side length of the April tag

    Returns:
        Pw: 4x3 numpy array describing the world coordinates of the April tag corners
            in the order of a, b, c, d for row order. See world_setup.jpg for details.

    """


    
    Point_a = np.array([-s/2,-s/2,0])
    Point_b = np.array([s/2,-s/2,0])
    Point_c = np.array([s/2,s/2,0])
    Point_d = np.array([-s/2,s/2,0])
    Pw = np.array ([Point_a,Point_b,Point_c,Point_d])
    print (Pw)


    return Pw
