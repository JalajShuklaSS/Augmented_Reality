from re import X
import numpy as np

def P3P(Pc, Pw, K=np.eye(3)):
    """
    Solve Perspective-3-Point problem, given correspondence and intrinsic

    Input:
        Pc: 4x2 numpy array of pixel coordinate of the April tag corners in (x,y) format
        Pw: 4x3 numpy array of world coordinate of the April tag corners in (x,y,z) format
    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3,) numpy array describing camera translation in the world (t_wc)

    """

    # Invoke Procrustes function to find R, t
    # You may need to select the R and t that could transoform all 4 points correctly. 
    # R,t = Procrustes(Pc_3d, Pw[1:4])
    #extracting the coordinates to solve according to the grunerts method
    x1 = Pw[1,0]
    x2 = Pw[2,0]
    x3 = Pw[3,0]

    y1 = Pw[1,1]
    y2 = Pw[2,1]
    y3 = Pw[3,1]
    
    z1 = Pw[1,2]
    z2 = Pw[2,2]
    z3 = Pw[3,2]
    
    
    p1 = np.array ([[x1],[y1],[z1]])
    p2 = np.array ([[x2],[y2],[z2]])
    p3 = np.array ([[x3],[y3],[z3]])

    # to calculate the side length of the triangle 
    a = np.linalg.norm(p2 - p3)
    b = np.linalg.norm(p1 - p3)
    c = np.linalg.norm(p1 - p2)
    
    u = Pc[:, 0]
    v = Pc[:, 1]
    u_v_homogeneous = np.column_stack((u, v, np.ones(4)))
    points_cali = (np.linalg.inv(K) @ u_v_homogeneous.T).T #points are calibrated after and taken the transpose to get the value of the array as 4x3
    


    j1 = np.array([(1/ np.linalg.norm(points_cali[1,:])) * points_cali[1,:]])
    j2 = np.array([(1/ np.linalg.norm(points_cali[2,:])) * points_cali[2,:]])
    j3 = np.array([(1/ np.linalg.norm(points_cali[3,:])) * points_cali[3,:]])
    

    # defining the angles alpha, beta and gama
    cos_alpha = np.dot(j2, j3.T)
    cos_beta = np.dot(j1, j3.T)
    cos_gama = np.dot(j1, j2.T)

# defining the equations to find the coefficients

    A0 = (1 + (a ** 2 - c ** 2) / b ** 2) ** 2 - 4 * (a ** 2 / b ** 2) * cos_gama ** 2
    A1 = 4 * (-((a ** 2 - c ** 2) / b ** 2) * (1 + ((a ** 2 - c ** 2) / b ** 2)) * cos_beta + 2 * (a ** 2 / b ** 2) * (cos_gama ** 2) * cos_beta - (1 - ((a ** 2 + c ** 2) / b ** 2)) * cos_alpha * cos_gama)
    A2 = 2 * (((a ** 2 - c ** 2) / b ** 2) ** 2 - 1 + 2 * ((a ** 2 - c ** 2) / b ** 2) ** 2 * cos_beta** 2 + 2 * ((b ** 2 - c ** 2) / b ** 2) * cos_alpha ** 2 - 4 * ((a ** 2 + c ** 2) / b ** 2) * cos_alpha * cos_beta * cos_gama + 2 * ((b ** 2 - a ** 2) / b ** 2) * cos_gama ** 2)
    A3 = 4 * (((a ** 2 - c ** 2) / b ** 2) * (1 - ((a ** 2 - c ** 2) / b ** 2)) * cos_beta - (1 - ((a ** 2 + c ** 2) / b ** 2)) * cos_alpha * cos_gama + 2 * (c ** 2 / b ** 2) * (cos_alpha ** 2) * cos_beta)
    A4 = ((a ** 2 - c ** 2) / b ** 2 - 1) ** 2 - 4 * (c ** 2 / b ** 2) * cos_alpha ** 2
    Cof = [A4, A3, A2, A1, A0] 
    print (Cof)
    A_final = np.ravel(np.array(Cof))
    print (A_final)

    # Now to get roots
    Roots = np.array([])
    Roots = np.real(np.roots(A_final))
    Roots_final = Roots[Roots > 0]
    print(Roots_final)

    correct_rotation = None
    correct_translation = None
    min_error = float('inf')

    for i in range(Roots_final.shape[0]):
        u = ((-1 + (a**2 - c**2) / b**2) * Roots_final[i]**2 - 2 * ((a**2 - c**2) / b**2) * cos_beta * Roots_final[i] + 1 + ((a**2 - c**2) / b**2)) / (2 * (cos_gama - Roots_final[i] * cos_alpha))
        s1 = np.sqrt((b**2) / (1 + Roots_final[i]**2 - 2 * Roots_final[i] * cos_beta))
        s2 = u * s1
        s3 = Roots_final[i] * s1

        o = s1 * j1
        l = s2 * j2
        q = s3 * j3

        p = np.vstack((o, l, q))

        R, t = Procrustes(Pw[1:], p)

        P = np.dot(K, np.dot(R, Pw[0, :].T) + t)
        P_l = P[-1]
        P_normalized = P / P_l
        P_final = P_normalized[:-1]

        difference = P_final - Pc[0]
        error = np.linalg.norm(difference)

        if error < min_error:
            min_error = error
            correct_rotation= R
            correct_translation= t

    inverse_correct_r = np.linalg.inv(correct_rotation)
    final = - inverse_correct_r @ correct_translation
    matrix = (inverse_correct_r, final)

    return matrix

   
    

def Procrustes(X, Y):
    """
    Solve Procrustes: Y = RX + t

    Input:
        X: Nx3 numpy array of N points in camera coordinate (returned by your P3P)
        Y: Nx3 numpy array of N points in world coordinate
    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3,) numpy array describing camera translation in the world (t_wc)

    """

    # Centering the values
    m_x = np.mean(X, axis=0)
    m_y = np.mean(Y, axis=0)

    X_c = (X - m_x).T
    Y_c = (Y - m_y).T

    # Calculating the rotation matrix
    R = Y_c @ X_c.T

    # Taking the singular value decomposition
    U, S, V1 = np.linalg.svd(R)
    V = V1.T

    # Calculating the determinant to maintain the orientation
    mat_a = np.eye(3)
    mat_a[2, 2] = np.linalg.det(V @ U.T)

    # Calculating the final rotation matrix now and also getting the translation vector
    R1 = np.dot(mat_a, V.T)
    R = np.dot(U,R1)
    t1 = m_y - R @ m_x
    t = np.array(t1).squeeze() 

    return R, t


 
