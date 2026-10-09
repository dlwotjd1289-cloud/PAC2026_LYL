from setuptools import setup, find_packages

setup(
    name="pac_common",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/pac_common"]),
        ("share/pac_common", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AHEAD team",
    maintainer_email="maintainers@example.invalid",
    description=(
        "PAC 2026 common contracts"
        if "pac_common" == "pac_common"
        else "AHEAD planning stages 5-3 to 5-6"
    ),
    license="LicenseRef-All-Rights-Reserved",
)
