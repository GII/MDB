from setuptools import setup

package_name = 'mdb'

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
            'a_node = mdb.a_node:main',
            'publisher = mdb.publisher:main',
            'execution_node = mdb.execution_node:main',
            'create_node_client = mdb.create_node_client:main',
            'commander = mdb.commander:main',
            'cognitive_node = mdb.cognitive_node:main',
            'ltm = mdb.ltm:main'
        ],
    },
)