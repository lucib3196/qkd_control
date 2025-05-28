e# qkd_control

# Recreate it with access to system packages
python3 -m venv ~/ros2_jazzy/myenv 
# Activate virtual environment 
source myenv/bin/activate
# pip install requirements
## Use a time out to install requirements
pip install -r requirements.txt --timeoute 300


source ~/ros2_pyenv/bin/activate
cd ~/ros2_jazzy
export PYTHONPATH=$VIRTUAL_ENV/lib/python3.11/site-packages:$PYTHONPATH
source install/setup.bash
sudo pigpiod

ros2 launch qkd_control/qkd_launches/launch/main_launch.py