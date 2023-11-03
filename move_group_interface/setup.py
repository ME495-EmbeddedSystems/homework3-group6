from setuptools import find_packages, setup

package_name = 'move_group_interface'

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
    maintainer='Luke Batteas',
    maintainer_email='LukeBatteas2027@u.northwestern.edu',
    description='Offers a simpler way of interacting with move_group node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_group_test = move_group_interface.move_group_test:main'
        ],
    },
)
