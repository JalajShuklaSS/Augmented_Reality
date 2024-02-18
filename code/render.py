import pyrender
from pyrender import Mesh
import numpy as np
import trimesh
import json
import glob
import pdb

class Renderer:
    """
    Code adapted from https://github.com/vchoutas/smplify-x
    """
    def __init__(self, intrinsics, img_w, img_h):
        """
        Initialize the Renderer class.

        Parameters:
        - intrinsics: The camera intrinsics matrix (3x3).
        - img_w: Width of the image.
        - img_h: Height of the image.
        """
        self.renderer = pyrender.OffscreenRenderer(viewport_width=img_w,
                                                   viewport_height=img_h,
                                                   point_size=1.0)
        self.focal_x = intrinsics[0, 0]
        self.focal_y = intrinsics[1, 1]
        self.center_x = intrinsics[0, 2]
        self.center_y = intrinsics[1, 2]
        self.img_w = img_w
        self.img_h = img_h

    def render(self, meshes, R, t, img):
        """
        Render the scene with given meshes, rotation, translation, and original image.

        Parameters:
        - meshes: List of pyrender Mesh objects representing the objects in the scene.
        - R: Rotation matrix.
        - t: Translation vector.
        - img: Original image on which the scene is rendered.

        Returns:
        - output_img: Rendered image with valid pixel values.
        - rend_depth: Depth map of the rendered scene.
        """
        # Construct a scene with ambient light
        scene = pyrender.Scene(ambient_light=(0.5, 0.5, 0.5))

        for mesh in meshes:
            scene.add(mesh)

        # Convert rotation from OpenGL to OpenCV
        gl2cv_rot = trimesh.transformations.rotation_matrix(np.radians(180), [1, 0, 0])
        T_w_c = np.eye(4)
        T_w_c[:3, :3] = R
        T_w_c[:3, 3] = t
        T_w_gl = T_w_c @ gl2cv_rot

        # Test camera coordinate
        camera = pyrender.IntrinsicsCamera(fx=self.focal_x, fy=self.focal_y,
                                           cx=self.center_x, cy=self.center_y,
                                           zfar=1000)
        # Add camera to the scene
        scene.add(camera, pose=T_w_gl)

        # Render image
        color, rend_depth = self.renderer.render(scene, flags=pyrender.RenderFlags.RGBA)
        color = color.astype(np.uint8)

        # Some pixel values are invalid
        # Paste the rendered image onto the original image based on a valid depth mask
        valid_mask = (rend_depth > 0)[:, :, None]
        output_img = (color[:, :, :3] * valid_mask +
                      (1 - valid_mask) * img)

        return output_img, rend_depth
