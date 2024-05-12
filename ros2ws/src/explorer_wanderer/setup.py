from setuptools import setup

package_name = 'explorer_wanderer'

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
    maintainer='slowrunner',
    maintainer_email='slowrunner@noreply.users.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wanderer = explorer_wanderer.wanderer:main',
            'wanderer_server = explorer_wanderer.wanderer_server:main',
            'discoverer_server = explorer_wanderer.discoverer_server:main'
        ],
    },
)
