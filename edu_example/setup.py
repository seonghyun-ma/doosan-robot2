from setuptools import find_packages, setup
from glob import glob

package_name = 'edu_example'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob('launch/*')),
        (share_dir + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asd',
    maintainer_email='asd@todo.todo',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [       
                 
            # Force Reacion
            'force_reaction     = edu_example.force_reaction:main',
                       
            # Multi Robots
            'multi_robot_1      = edu_example.multi_robot_1:main',
            'multi_robot_2      = edu_example.multi_robot_2:main',
            
            
            # Keyboard Jog
            'keyboard_control   = edu_example.keyboard_control:main',
            'keyboard_publisher = edu_example.keyboard_publisher:main',

        ],
    },
)

