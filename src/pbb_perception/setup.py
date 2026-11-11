from setuptools import setup
from glob import glob

setup(name='pbb_perception', version='0.2.0', packages=['pbb_perception'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pbb_perception']),
                  ('share/pbb_perception', ['package.xml', 'README.md', 'NOTICE.md']),
                  ('share/pbb_perception/models', glob('models/*.pth'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='PBB team', maintainer_email='leey00jay@users.noreply.github.com',
      description='PBB ROS 2 perception', license='CC-BY-4.0',
      tests_require=['pytest'], entry_points={'console_scripts': ['perception = pbb_perception.node:main']})
