from setuptools import setup
from glob import glob

setup(name='pbb_visualization', version='0.2.0', packages=['pbb_visualization'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pbb_visualization']),
                  ('share/pbb_visualization', ['package.xml', 'README.md', 'NOTICE.md'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='PBB team', maintainer_email='leey00jay@users.noreply.github.com',
      description='PBB ROS 2 visualization', license='CC-BY-4.0',
      tests_require=['pytest'], entry_points={'console_scripts': ['display = pbb_visualization.node:main']})
