from setuptools import setup

package_name = 'master_driver_package'

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
    maintainer='ros',
    maintainer_email='acampbellb@hotmail.com',
    description='master expansion board driver package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_driver_node = master_driver_package.master_driver_node:main'
        ],
    },
)
