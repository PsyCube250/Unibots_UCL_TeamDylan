from setuptools import find_packages, setup

package_name = 'safety_io'

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
    maintainer='jetson',
    maintainer_email='ksmxkaicsoc@gmail.com',
    description='Jetson GPIO pause switch and red/green LED publisher for Unibots',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_io = safety_io.safety_io:main',
        ],
    },
)
