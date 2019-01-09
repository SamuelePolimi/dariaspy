from setuptools import setup

setup(name='dariaspy',
      version='0.1.0',
      description='Library for using darias.',
      url='https://samuelepolimi.github.io/DariasPy-Doc/',
      author='Intelligent Autonomous Systems Lab',
      author_email='samuele@robot-learning.de',
      license='MIT',
      packages=['dariaspy'],
      zip_safe=False,
      install_requires=[
          'numpy>=1.15.4'
      ])