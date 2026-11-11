from setuptools import setup
from glob import glob

setup(name='pbb_carla', version='0.2.0', packages=['pbb_carla'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pbb_carla']),
                  ('share/pbb_carla', ['package.xml', 'README.md', 'NOTICE.md'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='PBB team', maintainer_email='leey00jay@users.noreply.github.com',
      description='PBB ROS 2 carla', license='CC-BY-4.0',
      tests_require=['pytest'], entry_points={'console_scripts': ['bridge = pbb_carla.node:main']})
