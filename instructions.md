*  Clone teleop_tong repository ```git clone git@github.com-carpit680:carpit680/teleop_tong```
*  Install miniconda
* Create a conda env with ```conda create -n giraffe python=3.10 -c conda-forge```
* ```conda activate giraffe```
* ```./install_dex_teleop.sh```
* ```pip install -r requirements.txt```
* Prepare a Clean URDF
* ```python3 webcam_calibration_create_board.py```
* Print out calibration board.
* Setup light
* ```pip install pyusb```
* ```python3 webcam_calibration_collect_images.py```
* ```python3 webcam_calibration_process_images.py```
* ```python3 webcam_teleop_interface.py```
* ```pip install urchin```
* 
