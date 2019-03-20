import almath
import numpy
import cv2
import cv2.aruco
import vision_definitions as vd

BACKUP2HOME_VECTOR = [0., -3.5, almath.PI]

CAMERA_DISTORTION_COEFF = numpy.array(
    [[0.13086823, -0.44239733, 0.0004841, -0.00322714, 0.16996254]])

CAMERA_MATRIX_RESOLUTION_2560_1920 = numpy.array([
    [2.41523736e+03, 0.00000000e+00, 1.25128063e+03],
    [0.00000000e+00, 2.41690366e+03, 9.94791007e+02]])

CAMERA_MATRIX_RESOLUTION_INDEPENDANT = numpy.array([
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

CAMERA_RESOLUTIONS = [vd.k16VGA, vd.k4VGA, vd.kVGA, vd.kQVGA, vd.kQQVGA, vd.kQQQVGA, vd.kQQQQVGA]
K16VGA_RESOLUTION = {"x": 2560, "y": 1920}

CAMERA_DATAS_AT_RESOLUTION = {camera_resolution: {
    "matrix": numpy.append(CAMERA_MATRIX_RESOLUTION_2560_1920 / (2. ** i), CAMERA_MATRIX_RESOLUTION_INDEPENDANT,
                           axis=0),
    "image_size": (K16VGA_RESOLUTION["x"] / (2 ** i), K16VGA_RESOLUTION["y"] / (2 ** i)),
    "fps": 5,
}
    for i, camera_resolution in enumerate(CAMERA_RESOLUTIONS)}

CAMERAS = {
    vd.kTopCamera: "CameraTop",
    vd.kBottomCamera: "CameraBottom",
}

CAMERA_PARAMETERS = {
    "AutoExposition": 11,
    "AutoWhiteBalance": 1,
    "Brightness": 0,
    "Contrast": 1,
    "Saturation": 2,
    "Exposure": 17,
}

DEFAULT_PARAMS = {
    "camera": vd.kTopCamera,
    "resolution": vd.kVGA,
    "ids": list(),  # ARuco ids, if empty, return all detected ids
    "size": 0.25,  # ARuco real size in meters
    "color": list(),  # ARuco real color in RGB, if empty not thresholding
    "dictionary": cv2.aruco.DICT_4X4_1000,  # ARuco dictionary
    "position": "ceiling",  # ARuco marker position "floor", "wall" or "ceiling"
    "color_space_and_channels": [vd.kYuvColorSpace, 1],  # Default is gray level 1 channel
    "exposure": 0,  # default exposure is 400, if 0, exposure not changed
}

SUBSCRIBER_ID = "camera_image"
