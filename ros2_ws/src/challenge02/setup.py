from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'challenge02'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Llamamos ela rchivo de lanzamiento (launch)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Llamamos los archivos de configuracion (YMAL)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='josea',
    maintainer_email='josea@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = challenge02.controller:main',
            'dc_motor= challenge02.dc_motor:main',
            'set_point = challenge02.set_point:main',
            'plant_test = challenge02.plant_test:main'
        ],
    },
)
