'''
Example of usage:

'''
import open3d
import argparse
import cv2
if True:  # Add project root
    import sys
    import os
    ROOT = os.path.dirname(os.path.abspath(__file__))+'/../'
    sys.path.append(ROOT)

# -- Command line arguments.

CAMERA_INFO_EXAMPLE = '''{
    "width" : 640,
    "height" : 480,
    "intrinsic_matrix" :
    [
        617.25,
        0,
        0,
        0,
        617.5486450195312,
        0,
        317.3921203613281,
        245.98019409179688,
        1
    ]
}'''


def parse_command_line_args():
    parser = argparse.ArgumentParser(
        description="Draw point cloud from color and depth image and camera info.")

    parser.add_argument(
        "-c", "--color_image", type=str,
        default=ROOT+"data/image_i1/color/00083.png")

    parser.add_argument(
        "-d", "--depth_image", type=str,
        default=ROOT+"data/image_i1/depth/00083.png")

    parser.add_argument(
        "-i", "--camera_info", type=str,
        default=ROOT+"data/image_i1/cam_params_realsense.json",
        help="Json file that contains the camera info."
        "An example is shown below: " + CAMERA_INFO_EXAMPLE)

    parser.add_argument(
        "-u", "--depth_unit", type=float,
        default=0.001,
        help="if depth_img[i, j] is x, then the real depth is x*depth_unit meters.")

    parser.add_argument(
        "-t", "--depth_trunc", type=float,
        default=3.0,
        help="Depth value larger than this will be truncated.")

    args = parser.parse_args()
    return args

# -- Function to create pointcloud.


def create_open3d_point_cloud_from_rgbd(
        color_img, depth_img,
        camera_info_file_name,
        depth_unit=0.001,
        depth_trunc=3.0):
    ''' Create pointcreate_open3dpoint_cloud_from_rgbd cloud of open3d format, given opencv rgbd images and camera info.
    Arguments:
        color_img {np.ndarry, np.uint8}:
            3 channels of BGR. Undistorted.
        depth_img {np.ndarry, np.uint16}:
            Undistorted depth image that matches color_img.
        cam_info {CameraInfo}
        depth_unit {float}:
            if depth_img[i, j] is x, then the real depth is x*depth_unit meters.
        depth_trunc {float}:
            Depth value larger than ${depth_trunc} meters
            gets truncated to 0.
    Output:
        open3d_point_cloud {open3d.geometry.PointCloud}
            See: http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html
    Reference:
    '''

    # Create `open3d.geometry.RGBDImage` from color_img and depth_img.
    # http://www.open3d.org/docs/0.7.0/python_api/open3d.geometry.create_rgbd_image_from_color_and_depth.html#open3d.geometry.create_rgbd_image_from_color_and_depth
    rgbd_image = open3d.geometry.RGBDImage.create_from_color_and_depth(
        color=open3d.geometry.Image(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)),
        depth=open3d.geometry.Image(depth_img),
        depth_scale=1.0/depth_unit,
        convert_rgb_to_intensity=False)

    # Read camera info: `class open3d.camera.PinholeCameraIntrinsic`.
    # http://www.open3d.org/docs/release/python_api/open3d.camera.PinholeCameraIntrinsic.html
    pinhole_camera_intrinsic = open3d.io.read_pinhole_camera_intrinsic(
        camera_info_file_name)

    # Project image pixels into 3D world points.
    # Output type: `class open3d.geometry.PointCloud`.
    # http://www.open3d.org/docs/0.6.0/python_api/open3d.geometry.create_point_cloud_from_rgbd_image.html#open3d.geometry.create_point_cloud_from_rgbd_image
    open3d_point_cloud = open3d.geometry.PointCloud.create_from_rgbd_image(
        image=rgbd_image,
        intrinsic=pinhole_camera_intrinsic)

    return open3d_point_cloud


if __name__ == '__main__':
    args = parse_command_line_args()
    color = cv2.imread(args.color_image, cv2.IMREAD_COLOR)
    depth = cv2.imread(args.depth_image, cv2.IMREAD_UNCHANGED)
    open3d_point_cloud = create_open3d_point_cloud_from_rgbd(
        color, depth, args.camera_info,
        args.depth_unit, args.depth_trunc)
    open3d.visualization.draw_geometries(
        [open3d_point_cloud])
