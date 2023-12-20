from setuptools import find_packages, setup

package_name = 'coord_trans_py'

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
    description='Calcula las coordenadas del objeto con respecto al sistema de referencia del robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coord_trans = coord_trans_py.coord_trans_py:main'
        ],
    },
)
