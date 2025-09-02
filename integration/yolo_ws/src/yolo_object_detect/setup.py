from setuptools import find_packages, setup
from glob import glob

package_name = 'yolo_object_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/best.pt', 'resource/IMG_5153.JPG']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linkedata',
    maintainer_email='yejunwu123@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detect = yolo_object_detect.object_detect_node:main',
            'object_detect_client = yolo_object_detect.object_detect_client_node:main',
        ],
    },
)
