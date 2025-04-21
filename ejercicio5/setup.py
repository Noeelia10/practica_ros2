from setuptools import find_packages, setup

package_name = 'ejercicio5'

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
		'cliente_movimiento = ejercicio5.cliente_movimiento:main',
		'servidor_movimiento = ejercicio5.servidor_movimiento:main',
        ],
    },
)
