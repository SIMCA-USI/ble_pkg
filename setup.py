from setuptools import find_packages, setup
from glob import glob

package_name = 'ble_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/{}'.format(package_name), glob('launch/*.launch.py')),
        ('share/conf/', glob('conf/*')),
        ('share/test/', glob('test/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simca',
    maintainer_email='simca.insia@upm.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ble_scanner_original = ble_pkg.ble_scanner:main',
            'tpms_monitor = ble_pkg.tpms_monitor:main',
        ],
    },
)
