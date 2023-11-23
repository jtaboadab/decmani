from setuptools import find_packages, setup

package_name = 'detectron2_py'

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
    description='Detectron2 Node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detectron2 = detectron2_py.detectron2_py:main',
            'bbox = detectron2_py.bbox_py:main',
            'pruebasz = detectron2_py.pruebas_coordenada_z_py:main',
            'pruebasbbox = detectron2_py.pruebas_bbox_py:main'
        ],
    },
)
