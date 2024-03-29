from setuptools import find_packages, setup

package_name = 'IWISUM_project'

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
    maintainer='adrian',
    maintainer_email='adrian.besciak99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = IWISUM_project.subscriber_example:main',
            'talker = IWISUM_project.publisher_example:main',
            'f1tenth_controller = IWISUM_project.f1tenth_controller:main',
        ],
    },
)
