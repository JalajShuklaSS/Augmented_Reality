# Import necessary libraries and modules
from render import Renderer
import imageio
import numpy as np
import matplotlib.pyplot as plt
import trimesh
import glob
from pyrender import Mesh
from solve_pnp import PnP
from solve_p3p import P3P
from est_Pw import est_Pw
from tqdm import tqdm
from est_pixel_world import est_pixel_world
from image_click import ImageClicker
import copy
import argparse
import sys

# Define a function for reprojecting points and rendering
def run_reproject(pixels, R_wc, t_wc, K, image, obj_meshes=None, shift=None):
    assert(len(pixels) == len(obj_meshes))

    # Estimate the object center point
    if shift is None:
        shift = est_pixel_world(pixels, R_wc, t_wc, K)

    # Modify mesh vertices
    all_meshes = []
    for i in range(len(pixels)):
        obj_meshes[i].vertices = obj_meshes[i].vertices + shift[i:i+1, :]
        pyrender_mesh = Mesh.from_trimesh(obj_meshes[i], smooth=False)
        all_meshes.append(pyrender_mesh)

    # Render the scene
    result_image, depth_map = renderer.render(all_meshes, R_wc, t_wc, image)
    return result_image, shift

# Main execution block
if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--solver', type=str, default='PnP', help="Algorithm to use, PnP or P3P", choices=["PnP", "P3P"])
    parser.add_argument('--click_points', action='store_true', help="Whether to click points for placing the objects")
    parser.add_argument('--debug', action='store_true', help="Helper flag for debugging")
    args = parser.parse_args()

    # Click points interactively or use pre-defined locations
    if args.click_points:
        fig, ax = plt.subplots()
        ax.set_title('Click to build line segments')
        frame = imageio.imread("../data/frames/frame0000.jpg")
        ax.set_title('Click two points to place objects')
        image = ax.imshow(frame)
        pixel_selector = ImageClicker(image)
        plt.show()
        click_points = np.stack(pixel_selector.points).astype(int)
    else:
        click_points = np.array([[220, 330], [550, 260]])  # Pre-defined locations

    # Process each frame
    print('Processing each frame...')
    frames = glob.glob('../data/frames/*.jpg')
    frames.sort()
    images = [np.array(imageio.imread(f)) for f in frames]

    # Extrinsic corners and intrinsic matrix
    corners = np.load('../data/corners.npy')
    K = np.array([[823.8, 0.0, 304.8],
                  [0.0, 822.8, 236.3],
                  [0.0, 0.0, 1.0]])

    # Dictionary of solvers
    solvers_dict = {'PnP': PnP, 'P3P': P3P}

    # Construct renderer
    renderer = Renderer(intrinsics=K, img_w=640, img_h=480)

    # Load mesh models
    axis_mesh = trimesh.creation.axis(origin_size=0.02)
    vertices = axis_mesh.vertices

    drill_mesh = trimesh.load("../data/models/drill.obj")
    fuze_mesh = trimesh.load("../data/models/fuze.obj")
    vr_meshes = [drill_mesh, fuze_mesh]

    # Estimate world points
    tag_size = 0.14
    Pw = est_Pw(tag_size)

    # Initialize arrays for final results
    final = np.zeros([len(images), 480, 640, 3], dtype=np.uint8)
    shift = None  # Translations of the objects that will be rendered on the table

    # Loop through each frame
    for i, f in enumerate(tqdm(images)):
        # Get pose with PnP or P3P
        Pc = corners[i]
        R_wc, t_wc = solvers_dict[args.solver](Pc, Pw, K)

        # Render based on click points
        # Note that all pixels are in (x, y) format
        meshes = [copy.deepcopy(vr_meshes[m % len(vr_meshes)]) for m in range(click_points.shape[0])]
        if i == 0:
            reproject_image, shift = run_reproject(click_points, R_wc, t_wc, K, images[i], obj_meshes=meshes)
        else:
            reproject_image, shift = run_reproject(click_points, R_wc, t_wc, K, images[i],
                                                  obj_meshes=meshes, shift=shift)
        final[i] = reproject_image

        if args.debug:
            # Visualize the rendered and original images side by side
            pyrender_mesh = Mesh.from_trimesh(axis_mesh, smooth=False)
            axis_image, depth_map = renderer.render([pyrender_mesh], R_wc, t_wc, images[i])
            plt.subplot(1, 2, 1)
            plt.imshow(axis_image)
            plt.subplot(1, 2, 2)
            plt.imshow(reproject_image)
            plt.show()
            sys.exit()
        elif i == 0:
            # Save the first frame that will be included in your report
            plt.imsave('vis.png', reproject_image.astype(np.uint8))

    print('Complete.')
    print('Saving GIF...')
    with imageio.get_writer('VR_res.gif', mode='I') as writer:
        for i in range(len(final)):
            img = final[i]
            writer.append_data(img)

    print('Complete.')
