from setuptools import find_packages, setup

package_name = 'vedita_bot_addon'

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
    maintainer='jovan',
    maintainer_email='jovan.josafat@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'us_serial_publisher = vedita_bot_addon.us_serial_publisher:main',
            'sensor_fusion = vedita_bot_addon.sensor_fusion:main',
            'send_goal = vedita_bot_addon.send_goal:main',
        ],
    },
)
