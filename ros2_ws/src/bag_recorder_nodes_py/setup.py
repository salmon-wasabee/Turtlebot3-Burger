from setuptools import find_packages, setup

package_name = 'bag_recorder_nodes_py'

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
    maintainer='johnsmith',     
    maintainer_email='jsmith@todo.todo',     
    description='This is the package description for Task 7',     
    license='Apache License 2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
