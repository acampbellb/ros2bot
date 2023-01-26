import setuptools
 
with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="ros2bot_master_lib",
    version="0.0.1",
    author="Adam Campbell",
    author_email="abcampbellb@gmail.com",
    description="ros2bot master board driver library",
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