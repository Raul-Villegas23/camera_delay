from setuptools import setup

package_name = 'camera_delay'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Simulate camera delay',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_delay_node = camera_delay.camera_delay_node:main',
            'random_camera_delay_node = camera_delay.random_camera_delay_node:main ',
            'stereo_image_publisher = camera_delay.stereo_image_publisher:main',
            'predictive_display_node = camera_delay.predictive_display_node:main'
        ],
    },
)
