from setuptools import find_packages, setup

package_name = 'tests'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Chen',
    maintainer_email='kykc8888@gmail.com',
    description='Integration scenarios and fault-injection test suite for the AMMS system.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
