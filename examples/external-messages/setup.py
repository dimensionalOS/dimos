from pathlib import Path

from setuptools import setup
from dimos_cdrgen.source_package import setup_arguments

setup(**setup_arguments(Path(__file__).resolve().parent))
