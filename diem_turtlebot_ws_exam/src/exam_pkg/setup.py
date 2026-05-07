from setuptools import setup

package_name = 'exam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/exam_pkg']),
        ('share/' + package_name + '/resource', ['resource/Cone.pt']),
        ('share/' + package_name + '/resource', ['resource/positions.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hellottie',
    maintainer_email='charlotte.boucherie@studenti.unisa.it',
    description='Package for the exam with the navigation, waypoint, cone detection and kidnapping features',
    license='No declared licence',
    extras_require={
        'test': ['pytest', 'flake8', 'ament_pep257', 'ament_copyright'],
    },
    entry_points={
        'console_scripts': [
            'cone_detector = exam_pkg.cone_detector_node:main',
            'navigation = exam_pkg.navigation_node:main',
            'waypoint_manager = exam_pkg.waypoint_manager_node:main'
        ],
    },
)
