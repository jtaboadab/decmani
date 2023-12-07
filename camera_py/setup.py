from setuptools import find_packages, setup

package_name = 'camera_py'

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
    description='Capturadora de imagenes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = camera_py.camera_py:main'
        ],
    },
)
