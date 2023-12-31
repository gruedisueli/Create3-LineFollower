from setuptools import setup

package_name = 'line_follower_controller'

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
    maintainer='gruedi',
    maintainer_email='gruedi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "line_follower = line_follower_controller.line_follower:main",
            "serial_repeater = line_follower_controller.serial_repeater:main",
        ],
    },
)
