from setuptools import find_packages, setup

package_name = 'moveit_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/moveit_api.launch.xml',
         'config/panda_params.yaml', 'config/interbotix_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohan',
    maintainer_email='rohankota@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mv_api = moveit_api.moveit_api:main',
        ],
    },
)
