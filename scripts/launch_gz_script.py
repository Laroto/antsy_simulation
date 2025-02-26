#!/usr/bin/env python3
import os
import sys
import subprocess

def set_environment_variables():
    """ Set required environment variables for Gazebo """
    os.environ['PATH'] += ':/usr/bin'
    os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    os.environ['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'

def main():
    """ Main function to start Gazebo """
    set_environment_variables()

    # Extract parameters from ROS 2 launch file
    if len(sys.argv) < 4:
        print("Usage: launch_gz_script.py <world> <run_headless> <run_on_start>")
        sys.exit(1)

    world = sys.argv[1]
    run_headless = sys.argv[2] == 'true'
    run_on_start = sys.argv[3] == 'true'

    # Construct proper Gazebo launch command
    gz_command = ["/usr/bin/gz", "sim", world]
    if run_headless:
        gz_command.append("-s")
    if run_on_start:
        gz_command.append("-r")

    print(f"Starting Gazebo: {' '.join(gz_command)}")

    # Run Gazebo with correct environment variables
    subprocess.run(gz_command, env=os.environ, shell=True)

if __name__ == "__main__":
    main()
