from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'venom_overtake_manager'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='venom',
    maintainer_email='liyihan.xyz@gmail.com',
    description='Perception-triggered overtake manager for UGV road behavior.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tracker_node = venom_overtake_manager.tracker_node:main',
            'lead_selector_node = venom_overtake_manager.lead_selector_node:main',
            'overtake_manager_node = venom_overtake_manager.overtake_manager_node:main',
        ],
    },
)
