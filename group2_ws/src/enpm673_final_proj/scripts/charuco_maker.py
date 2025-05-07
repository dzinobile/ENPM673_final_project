# import cv2
# import numpy as np
# from cv2 import aruco
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_pdf import PdfPages

# # Letter size dimensions (in mm)
# letter_width_mm = 215.9   # 8.5 inches
# letter_height_mm = 279.4  # 11 inches

# # Print resolution (600 DPI for high quality)
# dpi = 1200
# pixels_per_mm = dpi / 25.4

# # ChArUco Board Parameters
# squares_x = 5
# squares_y = 7
# square_length_mm = 29     # Adjusted for letter size
# marker_length_mm = 19     # Marker size in mm

# # Convert mm to pixels
# square_length_px = int(square_length_mm * pixels_per_mm)
# marker_length_px = int(marker_length_mm * pixels_per_mm)

# # Create the ArUco dictionary
# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# # Create ChArUco Board
# charuco_board = aruco.CharucoBoard((squares_x, squares_y), 
#                                   square_length_px, 
#                                   marker_length_px, 
#                                   aruco_dict)

# # Calculate board dimensions
# board_width_px = squares_x * square_length_px
# board_height_px = squares_y * square_length_px

# # Letter size in pixels
# letter_width_px = int(letter_width_mm * pixels_per_mm)
# letter_height_px = int(letter_height_mm * pixels_per_mm)

# # Create a white background image of letter size
# letter_image = np.ones((letter_height_px, letter_width_px), dtype=np.uint8) * 255

# # Calculate centering offsets
# x_offset = (letter_width_px - board_width_px) // 2
# y_offset = (letter_height_px - board_height_px) // 2

# # Generate ChArUco Board Image
# board_image = charuco_board.generateImage((board_width_px, board_height_px), 
#                                          marginSize=0, 
#                                          borderBits=1)

# # Place the board on the letter-sized background
# letter_image[y_offset:y_offset+board_height_px, 
#              x_offset:x_offset+board_width_px] = board_image

# # Save as PNG
# cv2.imwrite("charuco_board_letter_size_1200dpi.png", letter_image)
