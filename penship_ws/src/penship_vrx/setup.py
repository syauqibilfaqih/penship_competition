from setuptools import find_packages, setup

package_name = 'penship_vrx'

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
    maintainer='syauqibilfaqih',
    maintainer_email='alisyauqibilfaqih@gmail.com',
    description='The package of vrx simulation 2023 by penship',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stationkeeping = penship_vrx.penship_stationkeeping:main',
            'scandockdeliver = penship_vrx.penship_scandockdeliver:main',
            'teleopkey = penship_vrx.penship_teleopkey:main',
            'accousticpreception = penship_vrx.penship_accousticpreception:main',
            'accoustictracking = penship_vrx.penship_accoustictracking:main',
        ],
    },
)
