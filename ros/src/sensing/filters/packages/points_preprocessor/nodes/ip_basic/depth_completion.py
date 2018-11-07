#! /usr/bin/env python
import sys
import collections
import cv2
import numpy as np

import rospy
import roslib

from copy import deepcopy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Full kernels
FULL_KERNEL_3 = np.ones((3, 3), np.uint8)
FULL_KERNEL_5 = np.ones((5, 5), np.uint8)
FULL_KERNEL_7 = np.ones((7, 7), np.uint8)
FULL_KERNEL_9 = np.ones((9, 9), np.uint8)
FULL_KERNEL_31 = np.ones((31, 31), np.uint8)

# 3x3 cross kernel
CROSS_KERNEL_3 = np.asarray(
    [
        [0, 1, 0],
        [1, 1, 1],
        [0, 1, 0],
    ], dtype=np.uint8)

# 5x5 cross kernel
CROSS_KERNEL_5 = np.asarray(
    [
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [1, 1, 1, 1, 1],
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
    ], dtype=np.uint8)

# 5x5 diamond kernel
DIAMOND_KERNEL_5 = np.array(
    [
        [0, 0, 1, 0, 0],
        [0, 1, 1, 1, 0],
        [1, 1, 1, 1, 1],
        [0, 1, 1, 1, 0],
        [0, 0, 1, 0, 0],
    ], dtype=np.uint8)

# 7x7 cross kernel
CROSS_KERNEL_7 = np.asarray(
    [
        [0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0],
    ], dtype=np.uint8)

# 7x7 diamond kernel
DIAMOND_KERNEL_7 = np.asarray(
    [
        [0, 0, 0, 1, 0, 0, 0],
        [0, 0, 1, 1, 1, 0, 0],
        [0, 1, 1, 1, 1, 1, 0],
        [1, 1, 1, 1, 1, 1, 1],
        [0, 1, 1, 1, 1, 1, 0],
        [0, 0, 1, 1, 1, 0, 0],
        [0, 0, 0, 1, 0, 0, 0],
    ], dtype=np.uint8)

class DepthCompleter:
    def __init__(self):
        self.__APP_NAME__ = "depth_completer"
        self._image_pub = rospy.Publisher("/points_depth", Image, queue_size=10)

        self._cv_bridge = CvBridge()
        self._image_sub = rospy.Subscriber("/camera0_rotated/image_cloud",Image, self._callback, queue_size=10)

        # Fast fill with Gaussian blur @90Hz (paper result)
        self.fill_type = 'fast'
        self.extrapolate = True
        self.blur_type = 'gaussian'

        # Fast Fill with bilateral blur, no extrapolation @87Hz (recommended)
        #self.fill_type = 'fast'
        #self.extrapolate = False
        #self.blur_type = 'bilateral'

        # Multi-scale dilations with extra noise removal, no extrapolation @ 30Hz
        self.fill_type = 'multiscale'
        self.extrapolate = False
        self.blur_type = 'bilateral'

    def _callback(self, data):
        try:
            input_data = deepcopy(data)
            tmp_image = self._cv_bridge.imgmsg_to_cv2(input_data, "mono16")
            depth_image = np.copy(tmp_image)
        except CvBridgeError as e:
            print(e)

        # Load depth projections from uint16 image
        projected_depths = np.float32(depth_image / 256.0)

        if self.fill_type == 'fast':
            final_depths = self.fill_in_fast(projected_depths, extrapolate=self.extrapolate, blur_type=self.blur_type)
        elif self.fill_type == 'multiscale':
            final_depths = self.fill_in_multiscale(projected_depths, extrapolate=self.extrapolate, blur_type=self.blur_type)
        else:
            raise ValueError('Invalid fill_type {}'.format(self.fill_type))

        final_image = (final_depths * 256).astype(np.uint16)

        try:
            self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(final_image, "mono16"))
        except CvBridgeError as e:
            print(e)

    def fill_in_fast(self, depth_map, max_depth=100.0, custom_kernel=DIAMOND_KERNEL_5,
                     extrapolate=False, blur_type='bilateral'):
        """Fast, in-place depth completion.

        Args:
            depth_map: projected depths
            max_depth: max depth value for inversion
            custom_kernel: kernel to apply initial dilation
            extrapolate: whether to extrapolate by extending depths to top of
                the frame, and applying a 31x31 full kernel dilation
            blur_type:
                'bilateral' - preserves local structure (recommended)
                'gaussian' - provides lower RMSE

        Returns:
            depth_map: dense depth map
        """

        # Invert
        valid_pixels = (depth_map > 0.1)
        depth_map[valid_pixels] = max_depth - depth_map[valid_pixels]

        # Dilate
        depth_map = cv2.dilate(depth_map, custom_kernel)

        # Hole closing
        depth_map = cv2.morphologyEx(depth_map, cv2.MORPH_CLOSE, FULL_KERNEL_5)

        # Fill empty spaces with dilated values
        empty_pixels = (depth_map < 0.1)
        dilated = cv2.dilate(depth_map, FULL_KERNEL_7)
        depth_map[empty_pixels] = dilated[empty_pixels]

        # Extend highest pixel to top of image
        if extrapolate:
            top_row_pixels = np.argmax(depth_map > 0.1, axis=0)
            top_pixel_values = depth_map[top_row_pixels, range(depth_map.shape[1])]

            for pixel_col_idx in range(depth_map.shape[1]):
                depth_map[0:top_row_pixels[pixel_col_idx], pixel_col_idx] = \
                    top_pixel_values[pixel_col_idx]

            # Large Fill
            empty_pixels = depth_map < 0.1
            dilated = cv2.dilate(depth_map, FULL_KERNEL_31)
            depth_map[empty_pixels] = dilated[empty_pixels]

        # Median blur
        depth_map = cv2.medianBlur(depth_map, 5)

        # Bilateral or Gaussian blur
        if blur_type == 'bilateral':
            # Bilateral blur
            depth_map = cv2.bilateralFilter(depth_map, 5, 1.5, 2.0)
        elif blur_type == 'gaussian':
            # Gaussian blur
            valid_pixels = (depth_map > 0.1)
            blurred = cv2.GaussianBlur(depth_map, (5, 5), 0)
            depth_map[valid_pixels] = blurred[valid_pixels]

        # Invert
        valid_pixels = (depth_map > 0.1)
        depth_map[valid_pixels] = max_depth - depth_map[valid_pixels]

        return depth_map


    def fill_in_multiscale(self, depth_map, max_depth=100.0,
                           dilation_kernel_far=CROSS_KERNEL_3,
                           dilation_kernel_med=CROSS_KERNEL_5,
                           dilation_kernel_near=CROSS_KERNEL_7,
                           extrapolate=False,
                           blur_type='bilateral'):
        """Slower, multi-scale dilation version with additional noise removal that
        provides better qualitative results.

        Args:
            depth_map: projected depths
            max_depth: max depth value for inversion
            dilation_kernel_far: dilation kernel to use for 30.0 < depths < 80.0 m
            dilation_kernel_med: dilation kernel to use for 15.0 < depths < 30.0 m
            dilation_kernel_near: dilation kernel to use for 0.1 < depths < 15.0 m
            extrapolate:whether to extrapolate by extending depths to top of
                the frame, and applying a 31x31 full kernel dilation
            blur_type:
                'gaussian' - provides lower RMSE
                'bilateral' - preserves local structure (recommended)

        Returns:
            depth_map: dense depth map
            process_dict: OrderedDict of process images
        """

        # Convert to float32
        depths_in = depth_map

        # Calculate bin masks before inversion
        valid_pixels_near = (depths_in > 0.1) & (depths_in <= 15.0)
        valid_pixels_med = (depths_in > 15.0) & (depths_in <= 30.0)
        valid_pixels_far = (depths_in > 30.0)

        # Invert (and offset)
        s1_inverted_depths = np.copy(depths_in)
        valid_pixels = (s1_inverted_depths > 0.1)
        s1_inverted_depths[valid_pixels] = \
            max_depth - s1_inverted_depths[valid_pixels]

        # Multi-scale dilation
        dilated_far = cv2.dilate(
            np.multiply(s1_inverted_depths, valid_pixels_far),
            dilation_kernel_far)
        dilated_med = cv2.dilate(
            np.multiply(s1_inverted_depths, valid_pixels_med),
            dilation_kernel_med)
        dilated_near = cv2.dilate(
            np.multiply(s1_inverted_depths, valid_pixels_near),
            dilation_kernel_near)

        # Find valid pixels for each binned dilation
        valid_pixels_near = (dilated_near > 0.1)
        valid_pixels_med = (dilated_med > 0.1)
        valid_pixels_far = (dilated_far > 0.1)

        # Combine dilated versions, starting farthest to nearest
        s2_dilated_depths = np.copy(s1_inverted_depths)
        s2_dilated_depths[valid_pixels_far] = dilated_far[valid_pixels_far]
        s2_dilated_depths[valid_pixels_med] = dilated_med[valid_pixels_med]
        s2_dilated_depths[valid_pixels_near] = dilated_near[valid_pixels_near]

        # Small hole closure
        s3_closed_depths = cv2.morphologyEx(
            s2_dilated_depths, cv2.MORPH_CLOSE, FULL_KERNEL_5)

        # Median blur to remove outliers
        s4_blurred_depths = np.copy(s3_closed_depths)
        blurred = cv2.medianBlur(s3_closed_depths, 5)
        valid_pixels = (s3_closed_depths > 0.1)
        s4_blurred_depths[valid_pixels] = blurred[valid_pixels]

        # Calculate a top mask
        top_mask = np.ones(depths_in.shape, dtype=np.bool)
        for pixel_col_idx in range(s4_blurred_depths.shape[1]):
            pixel_col = s4_blurred_depths[:, pixel_col_idx]
            top_pixel_row = np.argmax(pixel_col > 0.1)
            top_mask[0:top_pixel_row, pixel_col_idx] = False

        # Get empty mask
        valid_pixels = (s4_blurred_depths > 0.1)
        empty_pixels = ~valid_pixels & top_mask

        # Hole fill
        dilated = cv2.dilate(s4_blurred_depths, FULL_KERNEL_9)
        s5_dilated_depths = np.copy(s4_blurred_depths)
        s5_dilated_depths[empty_pixels] = dilated[empty_pixels]

        # Extend highest pixel to top of image or create top mask
        s6_extended_depths = np.copy(s5_dilated_depths)
        top_mask = np.ones(s5_dilated_depths.shape, dtype=np.bool)

        top_row_pixels = np.argmax(s5_dilated_depths > 0.1, axis=0)
        top_pixel_values = s5_dilated_depths[top_row_pixels,
                                             range(s5_dilated_depths.shape[1])]

        for pixel_col_idx in range(s5_dilated_depths.shape[1]):
            if extrapolate:
                s6_extended_depths[0:top_row_pixels[pixel_col_idx],
                pixel_col_idx] = top_pixel_values[pixel_col_idx]
            else:
                # Create top mask
                top_mask[0:top_row_pixels[pixel_col_idx], pixel_col_idx] = False

        # Fill large holes with masked dilations
        s7_blurred_depths = np.copy(s6_extended_depths)
        for i in range(6):
            empty_pixels = (s7_blurred_depths < 0.1) & top_mask
            dilated = cv2.dilate(s7_blurred_depths, FULL_KERNEL_5)
            s7_blurred_depths[empty_pixels] = dilated[empty_pixels]

        # Median blur
        blurred = cv2.medianBlur(s7_blurred_depths, 5)
        valid_pixels = (s7_blurred_depths > 0.1) & top_mask
        s7_blurred_depths[valid_pixels] = blurred[valid_pixels]

        if blur_type == 'gaussian':
            # Gaussian blur
            blurred = cv2.GaussianBlur(s7_blurred_depths, (5, 5), 0)
            valid_pixels = (s7_blurred_depths > 0.1) & top_mask
            s7_blurred_depths[valid_pixels] = blurred[valid_pixels]
        elif blur_type == 'bilateral':
            # Bilateral blur
            blurred = cv2.bilateralFilter(s7_blurred_depths, 5, 0.5, 2.0)
            s7_blurred_depths[valid_pixels] = blurred[valid_pixels]

        # Invert (and offset)
        s8_inverted_depths = np.copy(s7_blurred_depths)
        valid_pixels = np.where(s8_inverted_depths > 0.1)
        s8_inverted_depths[valid_pixels] = \
            max_depth - s8_inverted_depths[valid_pixels]

        depths_out = s8_inverted_depths

        return depths_out

    def cv2_show_image(self, window_name, image,
                       size_wh=None, location_xy=None):
        """Helper function for specifying window size and location when
        displaying images with cv2.

        Args:
            window_name: str window name
            image: ndarray image to display
            size_wh: window size (w, h)
            location_xy: window location (x, y)
        """

        if size_wh is not None and cv2.__version__.split(".")[0] == 2:
            cv2.namedWindow(window_name,
                            cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow(window_name, *size_wh)
        else:
            cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

        if location_xy is not None:
            cv2.moveWindow(window_name, *location_xy)

        cv2.imshow(window_name, image)

def main(args):
    depth_completer = DepthCompleter()

    rospy.init_node('depth_completer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)