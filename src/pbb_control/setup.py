from setuptools import setup
from glob import glob

setup(name='pbb_control', version='0.2.0', packages=['pbb_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pbb_control']),
                  ('share/pbb_control', ['package.xml', 'README.md', 'NOTICE.md'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='PBB team', maintainer_email='leey00jay@users.noreply.github.com',
      description='PBB ROS 2 control', license='CC-BY-4.0',
      tests_require=['pytest'], entry_points={'console_scripts': ['control = pbb_control.node:main']})
