from setuptools import find_packages, setup

package_name = 'april_tag_detector'

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
    description='AprilTag detector for Unibots arena localisation',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'april_tag_detector = april_tag_detector.april_tag_detector:main',
        ],
    },
)
