from setuptools import find_packages, setup

package_name = 'robot_perception'

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
    maintainer='turtle',
    maintainer_email='calamia.dan@gmail.com',
    description='Perceptions du robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "avoid = robot_perception.obstacle_avoidance2:main", # nom executable node évitement d'obstacle avec le capteur Lidar
            "follow = robot_perception.lane_following:main", # nom executable node suivie de voie avec la camera
        ],
    },
)
