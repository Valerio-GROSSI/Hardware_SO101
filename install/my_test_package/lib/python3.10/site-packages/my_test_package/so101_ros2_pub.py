import subprocess

def main():
    return subprocess.call(["ros2","run","my_test_package_pub_cpp","my_executable"])