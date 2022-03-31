from setuptools import setup

setup(name='dariaspy',
      version='0.1.0',
      description='Library for interfacing robots.',
      url='https://samuelepolimi.github.io/DariasPy-Doc/',
      author='Intelligent Autonomous Systems Lab',
      author_email='samuele@robot-learning.de',
      license='MIT',
      packages=['dariaspy'],
      zip_safe=False,
      install_requires=[
          'numpy>=1.15.4',
          'rospkg>=1.1.10',
          'enum>=0.4.7',
          'scipy>=1.2.2',
          'pyusb'
      ])
