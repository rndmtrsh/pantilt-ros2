from setuptools import find_packages, setup

package_name = 'pan_tilt_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rndmtrsh',
    maintainer_email='t.maulanaakmal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test_node = pan_tilt_controller.first_node:main",
            "draw_circle = pan_tilt_controller.draw_circle:main"
        ],
    },
)
