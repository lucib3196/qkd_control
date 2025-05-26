# qkd_control

c
# Recreate it with access to system packages
python3 -m venv ~/ros2_jazzy/myenv --system-site-packages

# Activate it
cd ~/ros2_jazzy
source ~/ros2_jazzy/qkd_control/myenv/bin/activate
export PYTHONPATH=$VIRTUAL_ENV/lib/python3.11/site-packages:$PYTHONPATH
source install/setup.bash