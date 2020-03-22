from distutils.core import setup

setup(
    name="robot_api",
    version="0.0.1",
    description="Simple API for four-legged robot",
    author="Anton Kolomeytsev",
    author_email="tonykolomeytsev@gmail.com",
    packages=["robot_api", "robot_api.core"],
    classifiers=["Programming Language :: Python :: 3"]
)