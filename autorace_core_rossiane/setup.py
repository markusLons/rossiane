import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autorace_core_rossiane'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
     (os.path.join('share', package_name, 'calibration'), glob('calibration/*')),
     (os.path.join('share', package_name, 'map'), glob('map/*'))
   ],
    install_requires=['setuptools',
                      'ultralytics',],
    zip_safe=True,
    maintainer='vladimir',
    maintainer_email='v.leshin@g.nsu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_tracking=autorace_core_rossiane.path_tracking:main',
            'take_picture=autorace_core_rossiane.take_picture:main',
            'pid_controller=autorace_core_rossiane.pid_controller:main'
        ],
    },
)
