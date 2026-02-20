from setuptools import find_packages, setup

package_name = 'state_estimator_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaime',
    maintainer_email='jloaiza505@gmail.com',
    description='Extended Kalman Filter state estimator for differential-drive odometry fusion.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_node = state_estimator_pkg.ekf_node:main',
        ],
    },
)
