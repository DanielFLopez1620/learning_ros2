from setuptools import find_packages, setup

package_name = 'm03_tf2_with_py'

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
    maintainer='DanielFLopez1620',
    maintainer_email='dfelipe.lopez@gmail.com',
    description=
        'A simple package that search to serve as an example of the tf2 topic with Python.',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_broad = m03_tf2_with_py.static_broadcaster:main',
        ],
    },
)
