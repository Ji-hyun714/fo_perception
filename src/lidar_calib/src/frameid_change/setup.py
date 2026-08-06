from setuptools import setup

package_name = 'frameid_change'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/frameid_ransac.launch.py']),
        ('share/' + package_name + '/launch', ['launch/run_change.launch.py']),
        ('share/' + package_name + '/launch', ['launch/run_change.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wise',
    maintainer_email='wise@ubuntu',
    description='PointCloud2 frame_id change',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pcd_frameid_change = frameid_change.pcd_frameid_change:main',
            'pcd_frameid_change_ransac = frameid_change.pcd_frameid_change_ransac:main',
        ],
    },
)
