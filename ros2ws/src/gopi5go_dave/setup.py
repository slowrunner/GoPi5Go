from setuptools import find_packages, setup

package_name = 'gopi5go_dave'

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
    maintainer='slowrunner',
    maintainer_email='slowrunner@users.noreply.github.com',
    description='Collection of GoPi5Go-Dave specific nodes',
    license='wild abandon',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'say_node = gopi5go_dave.say_node:main',
            'battery_node = gopi5go_dave.battery_node:main',
            'docking_node = gopi5go_dave.docking_node:main',
            'dave_node = gopi5go_dave.dave_node:main',
            'test_dock = gopi5go_dave.test_dock:main',
            'say_client = gopi5go_dave.say_svc_client:main',

        ],
    },
)
