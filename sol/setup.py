import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sol'

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['launch/', 'rviz/', 'params/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tn751',
    maintainer_email='tn751@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_manager = sol.multi_manager:main',
            'target_finder = sol.target_finder:main',
            'target_decider = sol.target_decider:main',
            'navigation_manager = sol.navigation_manager:main',
            'proximity_detector = sol.proximity_detector:main'
        ],
    },
)
