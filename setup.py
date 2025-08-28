from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ransac_seg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altairzhambyl',
    maintainer_email='altairzhambyl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'astar = ransac_seg.astar:main',
        'dfs = ransac_seg.dfs:main',
        'bfs = ransac_seg.bfs:main',
        'curve = ransac_seg.curve:main',
        'djikstra = ransac_seg.djikstra:main',
        'ransac_node = ransac_seg.ransac_node:main',
    ],
},

)
