from setuptools import setup, find_packages

setup(
    name='ABBRobotEGM',
    version='1.0.0',
    description='A Python library for real-time control and data streaming of ABB robots using EGM protocol, enabling high-frequency (250Hz) communication through UDP for industrial automation applications.',
    long_description=open('README.md', encoding='utf-8').read(),
    long_description_content_type='text/markdown',
    author='Florian LOBERT',
    url='https://github.com/FLo-ABB/ABB-EGM-Python',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License',  # Fixed classifier
        'Operating System :: OS Independent',
        'Intended Audience :: Developers',
    ],
    python_requires='>=3.6',  # Specify minimum Python version
    install_requires=[
        'numpy',
        'protobuf',
    ],
)
