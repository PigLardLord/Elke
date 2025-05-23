from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resources', glob('resources/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master',
    maintainer_email='piglardlord@elkeproject.com',
    description='Voice interface for Elke robot',
    license='Creative Commons Attribution-NonCommercial-ShareAlike 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wake_listener = audio.elke_listener:main',
            'stt_whisper_node = audio.stt_whisper_node:main',
            'voice_session_manager = audio.voice_session_manager:main',
        ],
    },
)

