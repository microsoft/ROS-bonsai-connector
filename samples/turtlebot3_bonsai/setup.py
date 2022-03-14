import os
from glob import glob
from setuptools import setup

package_name = 'turtlebot3_bonsai'
sim_main = 'turtlebot3_sim'
policy_main = 'turtlebot3_policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.json')),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Abby Harrison',
    maintainer_email='abby.harrison@microsoft.com',
    description='Microsoft Bonsai integrated turtlebot3 simulator framework',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            sim_main + ' = ' + package_name + '.' + sim_main+ ':main',
            policy_main + ' = ' + package_name + '.' + policy_main+ ':main',
        ],
    },
)
