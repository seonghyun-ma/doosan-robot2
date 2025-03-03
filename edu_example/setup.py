
# ma 241011 1435

from setuptools import find_packages, setup
from glob import glob # 추가

package_name = 'edu_example'
share_dir = 'share/' + package_name # 추가

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']), # 수정
        (share_dir + '/launch', glob('launch/*')), # 추가
        (share_dir + '/rviz', glob('rviz/*')), # 추가
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

            # Training nodes
            'tr_current_posx        = edu_example.tr_current_posx',
            'tr_joint_states        = edu_example.tr_joint_states',
            'tr_move                = edu_example.tr_move',
            'tr_tool_force          = edu_example.tr_tool_force',
            'tr_tool_force_reaction = edu_example.tr_tool_force_reaction',

            # Keyboard jog
            'keyboard_publisher     = edu_example.keyboard_publisher:main',
            'keyboard_control       = edu_example.keyboard_control:main',
            
        ],
    },
)

