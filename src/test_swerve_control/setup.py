from setuptools import setup

package_name = 'test_swerve_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='helios',
    maintainer_email='gagemiller155@gmail.com',
    description='Control the robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish = robot_control.publish_joint_command:main',
        ],
    },
)
