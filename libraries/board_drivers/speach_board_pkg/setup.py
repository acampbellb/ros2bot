import setuptools
 
with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="speach_driver",
    version="0.0.1",
    author="Adam Campbell",
    author_email="acampbellb@hotmail.com",
    description="Package to create speach board driver",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7',
)