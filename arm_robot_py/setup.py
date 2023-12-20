from setuptools import find_packages, setup

package_name = 'arm_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jtaboadab',
    maintainer_email='jtaboadaberlanga@gmail.com',
    description='Arm Robot Node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_robot=arm_robot_py.arm_robot_py:main'
        ],
    },
)
