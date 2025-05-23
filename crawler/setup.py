from setuptools import find_namespace_packages, setup

package_name = "crawler"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_namespace_packages(exclude=["test"]),
    package_data={"": ["web-ui/**", "web-ui/*/**"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jupit",
    maintainer_email="hello@jupiterpi.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "crawler_web_api = crawler.web_api:main",
            "crawler_blinker = crawler.blinker:main",
            "crawler_motors = crawler.motors:main",
            "crawler_encoders = crawler.encoders:main",
            "crawler_rl_environment = crawler.rl_environment:main",
            "crawler_q_learning = crawler.mock_q_learning:main",
        ],
    },
)
