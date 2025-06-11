from setuptools import setup

package_name = 'hello_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Simple hello world example package.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hello = hello_py.hello_node:main',
        ],
    },
)
