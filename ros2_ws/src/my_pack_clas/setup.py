from setuptools import find_packages, setup
# Import paralaucnh files
import os
from glob import glob

package_name = 'my_pack_clas'

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
            'my_pack = my_pack_clas.my_pack:main',
            'my_publisher = my_pack_clas.my_publisher:main',
            'my_subscriber = my_pack_clas.my_subscriber:main'
        ],
    },
)
