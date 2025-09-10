from setuptools import find_packages, setup

package_name = 'ur5_custom_control'

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
    maintainer='ubunutulinux',
    maintainer_email='ubunutulinux@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_dot_tracker = ur5_custom_control.red_dot_tracker:main',
            'plot_error = ur5_custom_control.plot_error:main',
            'trajectory_sender = ur5_custom_control.trajectory_sender:main',
            'mover = ur5_custom_control.mover:main'
        ],
    },
)
