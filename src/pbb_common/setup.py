from setuptools import setup
from glob import glob

setup(name='pbb_common', version='0.2.0', packages=['pbb_common'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pbb_common']),
                  ('share/pbb_common', ['package.xml', 'README.md', 'NOTICE.md'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='PBB team', maintainer_email='leey00jay@users.noreply.github.com',
      description='PBB ROS 2 common', license='CC-BY-4.0',
      tests_require=['pytest'], entry_points={'console_scripts': []})
