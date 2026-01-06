from setuptools import setup
import os
from glob import glob

package_name = 'cobot_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
       
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        ('share/' + package_name, ['package.xml']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    
    maintainer='Salim Samet Kotaş',
    maintainer_email='salimsamet@example.com',
    
    description='Professional PyQt5 control interface for the 6-DOF Cobot Pro.',

    license='Apache-2.0',
    
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'cobot_gui_node = cobot_gui.gui_app:main'
        ],
    },
)
