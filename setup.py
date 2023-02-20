from setuptools import setup

package_name = 'ros2_py_path_planning_server'

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
    maintainer='indy',
    maintainer_email='jonathandorsey1117@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"my_server = {package_name}.path_planning_service:main",
            f"my_client = {package_name}.path_planning_client:main"
        ],
    },
)
