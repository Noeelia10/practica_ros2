from setuptools import find_packages, setup

package_name = 'ejercicio4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ejercicio4']),
        ('share/ejercicio4', ['package.xml']),
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
		'mover_configurable = ejercicio4.mover_configurable:main',
        ],
    },
)
