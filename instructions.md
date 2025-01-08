[x] Clone teleop_tong repository ```git clone git@github.com-carpit680:carpit680/teleop_tongs.git```
[x] ```./install_dex_teleop.sh```
[x] ```pip install -r requirements.txt```
[x] ```pip install -e .```
[] Prepare a Clean URDF
[] ```python3 webcam_calibration_create_board.py```
[] Print out calibration board.
[] Setup lighting
[] ```python3 webcam_calibration_collect_images.py```
[] ```python3 webcam_calibration_process_images.py```
[] ```python3 webcam_teleop_interface.py```
[] ```python3 dex_teleop.py```
[] ```sudo usermod -a -G dialout $USER```
[] ```newgrp dialout```