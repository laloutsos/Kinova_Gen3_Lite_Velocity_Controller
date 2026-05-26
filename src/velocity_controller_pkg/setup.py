from setuptools import find_packages, setup

package_name = 'velocity_controller_pkg'

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
    maintainer='laloutsos nikos',
    maintainer_email='laloutsosnikos@gmail.com',
    description='Joints Velocity Controller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_velocity_controller = velocity_controller_pkg.joint_velocity_controller:main',
            'ee_velocity_controller = velocity_controller_pkg.ee_velocity_controller:main',
            'auto_joint_velocity_controller = velocity_controller_pkg.auto_joint_velocity_controller:main',
            'ee_trajectory_controller = velocity_controller_pkg.ee_trajectory_controller:main'
        ],
    },
)
