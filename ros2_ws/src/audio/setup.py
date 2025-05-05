from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (
            f'share/{package_name}/resources',
            glob(f'src/{package_name}/{package_name}/resources/*.ppn')
        ),
    ],
    include_package_data=True,
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

