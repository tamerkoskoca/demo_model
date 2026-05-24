from setuptools import find_packages, setup

package_name = 'burkut_perception'

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
    maintainer='bugra',
    maintainer_email='bugrakallat@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pole_ai_test = burkut_perception.pole_ai_test:main',
            'lidar_perception = burkut_perception.lidar_perception:main',
            'yolo_perception = burkut_perception.yolo_perception:main',
        ],
    },
)
