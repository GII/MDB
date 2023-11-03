from setuptools import setup

package_name = 'core'

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
    maintainer='cristina',
    maintainer_email='cristina@udc.es',
    description='Intermediate layer to manage nodes.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'a_node = core.a_node:main',
            # 'b_node = core.b_node:main',
            # 'cognitive_node = core.cognitive_node:main',
            'commander = core.commander_node:main',
            'execution_node = core.execution_node:main',
            'ltm = core.ltm:main',
            'main_loop = core.main_loop:main',
            'publisher = core.publisher:main',
        ],
    },
)
