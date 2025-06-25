try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup
from distutils.extension import Extension
import numpy

# Get numpy include directory.
numpy_include_dir = numpy.get_include()

_packages = find_packages()

packages = []
for p in _packages:
    if p.startswith('contact_graspnet_pytorch'):
        packages.append(p)

for p in packages:
    assert p.startswith('contact_graspnet_pytorch')
    
setup(
    name='contact_graspnet_pytorch',
    author='multiple',
    packages=packages,
    package_data={},
)