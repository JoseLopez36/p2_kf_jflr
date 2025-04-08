from setuptools import find_packages, setup

package_name = 'p2_kf_jflr'

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
    maintainer='student',
    maintainer_email='josloprui6@alum.us.es',
    description='Práctica 2 de la asignatura de Ampliación de Robótica',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kf_estimation = p2_kf_jflr.kf_estimation:main',
            'kf_estimation_vel = p2_kf_jflr.kf_estimation_vel:main',
        ],
    },
)
