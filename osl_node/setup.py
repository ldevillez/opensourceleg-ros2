from setuptools import find_packages, setup

package_name = 'osl_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ldevillez',
    maintainer_email='louis.devillez@gmail.com',
    description='OSL node for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "knee = osl_node.osl:osk_knee",
            "ankle = osl_node.osl:osk_ankle",
        ],
    },
)
