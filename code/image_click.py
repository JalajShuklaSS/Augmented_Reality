from matplotlib import pyplot as plt
import imageio
import numpy as np

class ImageClicker:
    """
    ImageClicker is a class for interactively clicking on points in an image.

    Usage:
    - Instantiate the class with an image and an optional number of points to click.
    - Click on points in the displayed image.
    - The clicked points will be printed to the console and marked with red dots in the image.
    - Pressing any key will close the image.

    Example:
    ```python
    img = imageio.imread('path/to/your/image.jpg')
    clicker = ImageClicker(img, num_points=4)
    ```

    Note: Ensure that the matplotlib backend is interactive.

    Attributes:
    - image: The image on which points are to be clicked.
    - num_points: The number of points to be clicked (default is 2).
    - count: The current count of clicked points.
    - points: A list to store the clicked (x, y) coordinates.
    - cid: The connection ID for the button press event.
    """
    def __init__(self, image, num_points=2):
        """
        Initialize the ImageClicker instance.

        Parameters:
        - image: The image on which points are to be clicked.
        - num_points: The number of points to be clicked (default is 2).
        """
        self.image = image
        self.num_points = num_points
        self.count = 0
        self.points = []
        self.cid = self.image.figure.canvas.mpl_connect('button_press_event', self)

    def __call__(self, event):
        """
        Callback function to handle button press events.

        Parameters:
        - event: The button press event object.
        """
        if (event.xdata is None) or (event.ydata is None):
            return
        print(event.xdata, event.ydata)
        plt.scatter(event.xdata, event.ydata, s=10, c='red')
        self.points.append(np.array([event.xdata, event.ydata]))
        self.count += 1
        self.image.figure.canvas.draw_idle()
        if self.count >= self.num_points:
            plt.pause(0.2)
            self.image.figure.canvas.mpl_disconnect(self.cid)
            plt.close()
