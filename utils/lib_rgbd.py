import numpy as np
import simplejson


class RgbdImage(object):
    def __init__(self, color, depth,
                 camera_info,
                 camera_pose=np.identity(4),  # in world frame.
                 depth_unit=0.001,
                 ):
        '''
            color {image}.
        Arguments:
            depth {image}: depth image of type np.uint16.
            camera_info {CameraInfo}:
            camera_pose {2d matrix}: 4x4 transformation matrix.
            camera_pose {2d matrix}: 4x4 transformation matrix.
        '''
        assert(len(depth.shape) == 2 and len(color.shape) == 3)
        self._depth = depth.astype(np.float32) * depth_unit
        self._camera_info = camera_info
        self._color = color
        self._row, self._col, self._fx, self._fy, self._cx, self._cy = camera_info.get_cam_params()
        self._camera_pose = camera_pose

    def get_3d_pos(self, x, y):
        '''
        Get the 3d position of the pixel (y, x) from depth image.
        The pixel is at yth row and xth column.
        '''
        row, col = self._xy_to_row_col(x, y)
        d = self._depth[row, col]
        xyz = [
            (col - self._cx)*d/self._fx,
            (row - self._cy)*d/self._fy,
            d]
        return xyz

    def is_depth_valid(self, x, y):
        row, col = self._xy_to_row_col(x, y)
        return self._depth[row, col] >= 0.00001

    def _xy_to_row_col(self, x, y):
        # row, col = round(self._row * y), round(self._col * x)
        return int(round(y)), int(round(x))

    def camera_pose(self):
        return self._camera_pose

    def set_camera_pose(self, camera_pose):
        self._camera_pose = camera_pose


class CameraInfo():

    def __init__(self, camera_info_json_file_path):
        data = read_json_file(camera_info_json_file_path)
        self._width = int(data["width"])  # int.
        self._height = int(data["height"])  # int.
        self._intrinsic_matrix = data["intrinsic_matrix"]  # list of float.
        # The list extracted from the matrix **column by column** !!!.
        # If the intrinsic matrix is:
        # [fx,  0, cx],
        # [ 0, fy, cy],
        # [ 0,  0,  1],
        # Then, self._intrinsic_matrix = [fx, 0, 0, 0, fy, 0, cx, cy, 1]

    def resize(self, ratio):
        r0, c0 = self._height, self._width
        if not (is_int(r0*ratio) and is_int(c0*ratio)):
            raise RuntimeError(
                "Only support resizing image to an interger size.")
        self._width = int(ratio * self._width)
        self._height = int(ratio * self._height)
        self._intrinsic_matrix[:-1] = [x*ratio
                                       for x in self._intrinsic_matrix[:-1]]

    def width(self):
        return self._width

    def height(self):
        return self._height

    def intrinsic_matrix(self, type="list"):
        if type == "list":
            return self._intrinsic_matrix
        elif type == "matrix":
            return np.array(self._intrinsic_matrix).reshape(3, 3).T
        else:
            raise RuntimeError("Wrong type in `def intrinsic_matrix()`")

    def get_img_shape(self):
        row, col = self._height, self._width
        return (row, col)

    def get_cam_params(self):
        ''' Get all camera parameters. 
        Notes: intrinsic_matrix:
            [0]: fx, [3]   0, [6]:  cx
            [1]:  0, [4]: fy, [7]:  cy
            [2]:  0, [5]   0, [8]:   1
        '''
        im = self._intrinsic_matrix
        row, col = self._height, self._width
        fx, fy, cx, cy = im[0], im[4], im[6], im[7]
        return row, col, fx, fy, cx, cy


def read_json_file(file_path):
    with open(file_path, 'r') as f:
        data = simplejson.load(f)
    return data


def is_int(num):
    ''' Is floating number very close to a int. '''
    # print(is_int(0.0000001)) # False
    # print(is_int(0.00000001)) # True
    return np.isclose(np.round(num), num)


def resize_color_and_depth(
        color, depth, ratio,
        is_size_after_resizing_should_be_interger=True):
    ''' Resize color and depth images by ratio. '''

    # -- Check input.
    if np.isclose(ratio, 1):
        return color, depth
    r0, c0 = color.shape[:2]
    if is_size_after_resizing_should_be_interger:
        if not (is_int(r0*ratio) and is_int(c0*ratio)):
            raise RuntimeError("Only support resizing image "
                               "to an interger size.")

    # -- Resize by cv2.INTER_NEAREST.
    def resize(img):
        return cv2.resize(
            src=img, dsize=None, fx=ratio, fy=ratio,
            interpolation=cv2.INTER_NEAREST)
    color = resize(color)
    depth = resize(depth)
    return color, depth
