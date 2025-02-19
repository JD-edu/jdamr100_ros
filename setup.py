from setuptools import find_packages, setup

package_name = 'jdamr100_ros'

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
    maintainer='slam',
    maintainer_email='jdedu.kr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jdamr100_teleop = jdamr100_ros.jdamr100_teleop:main',
        	'jdamr100_bridge = jdamr100_ros.jdamr100_bridge:main',
        ],
    },
)
