from setuptools import setup
from glob import glob

setup(name='pbb_decision', version='0.2.0', packages=['pbb_decision'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pbb_decision']),
                  ('share/pbb_decision', ['package.xml', 'README.md', 'NOTICE.md'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='PBB team', maintainer_email='leey00jay@users.noreply.github.com',
      description='PBB ROS 2 decision', license='CC-BY-4.0',
      tests_require=['pytest'], entry_points={'console_scripts': ['decision = pbb_decision.node:main']})
