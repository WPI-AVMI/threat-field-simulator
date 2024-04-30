from setuptools import find_packages, setup

package_name = 'avmi_lab_threat_field'

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
    maintainer='avmi-lab-user',
    maintainer_email='73857255+Michael-Beskid@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_test = avmi_lab_threat_field.move_test:main',
            'threat_field_navigator = avmi_lab_threat_field.threat_field_navigator:main'
        ],
    },
)
