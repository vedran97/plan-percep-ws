# README for setting up the  workspace:

[Click here!](./README.md)

# Sub Modules

* <a href="rpi-rt/index.html">rpi-rt</a>
* <a href="rosserial_terpbot/index.html">rosserial_terpbot</a>

# How the documentation pipeline works:

1. Packages in this repo are assumed to manage their own documentation.
2. The documentation for each package is generated using doxygen.
3. To generate this documentation, please roscd to the package and run ```doxygen Doxyfile``` ,if a Doxyfile exists.
4. All the packages are assumed to have the HTML documentation generated in the ```docs``` folder of the package, and not in ```docs/html``` of the package.
5. Run ```python3 combine_docs.py``` to copy all the documentation from the ```docs``` folder of each package to the ```docs``` folder of the workspace. In the docs folder of the workspace, each package's documentation is stored in a folder with the package name.
6. For any packages with documentation which you want to include in the final website, please add the package name and link to it's index.html file in the Sub Modules section of this file. Basic syntax is as follows: ```* <a href="package_name/index.html">package_name</a>```
7. Once this is done, run ```doxygen Doxyfile``` in the workspace folder to generate the documentation for the workspace.
8. Git Push the changes to the docs branch of the repo, this triggers a rebuild of the website.
9. Disclaimer : The combine_docs.py script has been taken from chatgpt.