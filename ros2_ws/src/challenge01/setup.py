"""
Opciones de operacion de nodos de ROS
"""

from setuptools import find_packages, setup
# Import paralaucnh files
import os
from glob import glob

package_name = 'challenge01'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')) # Ruta nevesario para launch
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
            'signal_generetor = challenge01.signal_generetor:main',
            'proccess = challenge01.proccess:main'
        ],
    },
)
