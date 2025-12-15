from setuptools import find_packages, setup

package_name = 'osl_node'

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
    maintainer='ldevillez',
    maintainer_email='louis.devillez@gmail.com',
    description='ROS2 node for OSL',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ankle = osl_node.osl:osl_ankle",
            "knee = osl_node.osl:osl_knee",
        ],
    },
)
