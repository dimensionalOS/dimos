from setuptools import setup, find_packages

setup(
    name='go2-webrtc-connect',
    version='1.0.0',
    author='legion1581',
    author_email='legion1581@gmail.com',
    packages=find_packages(),
    install_requires=[
        'aiortc',
        'pycryptodome',
        'opencv-python',
        'sounddevice',
        'pyaudio',
        'requests',
        'wasmtime'
    ],
)
