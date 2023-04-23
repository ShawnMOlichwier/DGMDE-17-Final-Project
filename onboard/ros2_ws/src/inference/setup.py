from setuptools import setup

import os
import glob

package_name = 'inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
#        (os.path.join('share/', package_name, 'models'), ["models/best_model.onnx"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='charleslariviere',
    maintainer_email='charleslariviere1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference = inference.inference:main'
            ],
    },
)
