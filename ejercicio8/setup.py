from setuptools import find_packages, setup

package_name = 'ejercicio8'

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
    maintainer='noelia',
    maintainer_email='noelia.pverdugo@alumnos.upm.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'seguridad_lidar = ejercicio8.seguridad_lidar:main',
        ],
    },
)
