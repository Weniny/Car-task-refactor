from glob import glob
from setuptools import setup

setup(name='forward_rejoin_bringup',version='0.1.0',packages=['forward_rejoin_bringup'],
    data_files=[('share/ament_index/resource_index/packages',['resource/forward_rejoin_bringup']),
                ('share/forward_rejoin_bringup',['package.xml']),
                ('share/forward_rejoin_bringup/launch',glob('launch/*.launch.py')),
                ('share/forward_rejoin_bringup/rviz',glob('rviz/*.rviz'))],
    install_requires=['setuptools'],zip_safe=True,maintainer='agilex',
    maintainer_email='agilex@todo.todo',description='Isolated forward rejoin trial',license='Apache-2.0')
