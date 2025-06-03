from setuptools import find_packages, setup

package_name = 'misty_interaction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        "coqui-tts[server]>=0.22.0",
        "soundfile",
        "pydub",
        'audio-common-msgs',
        'PyAudio',
    ],

    zip_safe=True,
    maintainer='Anthony Contreras',
    maintainer_email='contrant@oregonstate.edu',
    description='interacting with misty',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_input_node = misty_interaction.TextInputNode:main',
            'gtts_tts_node = misty_interaction.gtts_tts_node:main',
            'tts_node = misty_interaction.TTSNode:main',
            'mic_input_node = misty_interaction.MicInputNode:main',
        ],
    },
)
