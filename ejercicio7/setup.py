from setuptools import find_packages, setup

package_name = 'ejercicio7'

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
    maintainer='noelia',
    maintainer_email='noelia.pverdugo@alumnos.upm.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'cliente_poligono = ejercicio7.cliente_poligono:main',
		'servidor_poligono = ejercicio7.servidor_poligono:main',
        ],
    },
)
