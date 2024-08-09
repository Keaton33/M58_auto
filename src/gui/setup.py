from setuptools import find_packages, setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),        
        ('share/' + package_name, ['launch/goliath_crane_ac_launch.py']),
        ('share/' + package_name, ['rviz_cfg/goliath_crane.rviz']),
        ('share/' + package_name, ['param/param.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = gui.gui_node:main'
        ],
    },
)
