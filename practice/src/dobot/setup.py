from setuptools import find_packages, setup
from glob import glob

package_name = 'dobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/mesh', glob('mesh/*')),
        ('share/' + package_name + '/mjcf', glob('mjcf/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
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
            'joint_state_subscriber = dobot.joint_state_subscriber:main',
        ],
    },
)
