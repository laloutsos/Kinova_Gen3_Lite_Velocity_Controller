from setuptools import find_packages, setup

package_name = 'tests_pkg'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Tests and Logs about the velocity of the manipulator',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_state_logger = tests_pkg.joints_state_logger:main',
            'steady_state_velocity_test = tests_pkg.steady_state_velocity_test:main'
        ],
    },
)
