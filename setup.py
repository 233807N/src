from setuptools import setup

package_name = 'follow_me'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdul Latif',
    maintainer_email='tep@email.com',
    description='Leg-following ROS 2 node using MediaPipe.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_me = follow_me.follow_me:main',
        ],
    },
)
