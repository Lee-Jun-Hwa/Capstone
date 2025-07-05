from setuptools import setup

package_name = 'angle_calculator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'angle_calculator'},
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Baeminseong',
    author_email='alstjd1118@hanyang.ac.kr',
    maintainer='Baeminseong',
    maintainer_email='alstjd1118@hanyang.ac.kr',
    description='ROS2 package for line angle and heading comparator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'line_angle_heading_comparator = my_package.line_angle_heading_comparator:main'
        ],
    },
)
