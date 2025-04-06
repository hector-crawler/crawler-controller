from setuptools import find_packages, setup

package_name = 'crawler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={'': ['*/web-ui/**', '*/web-ui/*/**']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jupit',
    maintainer_email='hello@jupiterpi.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'crawler_test = crawler.crawler_test:main',
            'crawler_web_api = crawler.crawler_web_api.crawler_web_api:main',
            'crawler_blinker = crawler.crawler_blinker:main'
        ],
    },
)
