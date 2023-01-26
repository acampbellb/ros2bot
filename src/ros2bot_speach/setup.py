from setuptools import setup

package_name = 'ros2bot_speach'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Campbell',
    maintainer_email='abcampbellb@gmail.com',
    description='ros2bot speach expansion board driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2bot_speach = ros2bot_speach.ros2bot_speach:main'
        ],
    },
)
