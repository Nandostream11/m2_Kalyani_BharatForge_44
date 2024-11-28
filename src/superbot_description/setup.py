from setuptools import setup, find_packages

package_name = 'superbot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anandv',
    maintainer_email='anandv@todo.todo',
    description='Superbot Description Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_listener = superbot_description.ros_nodes.robot_listener:main',
            'data_publisher = superbot_description.ros_nodes.data_publisher:main'
        ],
    },
)