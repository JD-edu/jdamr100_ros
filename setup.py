from setuptools import find_packages, setup

package_name = 'jdamr_100_teleop'

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
            'jdamr_100_teleop = jdamr_100_teleop.jdamr_100_teleop:main',
        	'jdamr_100_bridge = jdamr_100_teleop.jdamr_100_bridge:main',
        ],
    },
)
