import cv2
import webcam_calibration_aruco_board as ab

aruco_dict = ab.aruco_dict
board = ab.board

########
# From
# https://papersizes.online/paper-size/letter/
#
# "Letter size in pixels when using 600 DPI: 6600 x 5100 pixels."
# "A4 size in pixels when using 600 DPI: 7,016 x 4,960 pixels."
########

########
# From
# https://docs.opencv.org/4.8.0/d4/db2/classcv_1_1aruco_1_1Board.html
#
# Parameters
# outSize	size of the output image in pixels.
# img	        output image with the board. The size of this image will be outSize and the board will be on the center, keeping the board proportions.
# marginSize	minimum margins (in pixels) of the board in the output image
# borderBits	width of the marker borders.
########

image_size = (4960, 7016)
margin_size = int(image_size[1]/20)
border_bits = 1

board_image = board.generateImage(
    outSize=image_size,
    marginSize=margin_size,
    borderBits=border_bits)

cv2.imwrite('webcam_aruco_calibration_board.png', board_image)
