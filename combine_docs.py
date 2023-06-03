import os
import shutil
## Direct output from chatgpt
def create_doxygen_website(workspace_path, output_path):
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # Create the index.html file for the combined website
    index_path = os.path.join(output_path, 'index.html')
    with open(index_path, 'w') as index_file:
        index_file.write('<html>\n')
        index_file.write('<head>\n')
        index_file.write('<title>Combined Doxygen Documentation</title>\n')
        index_file.write('</head>\n')
        index_file.write('<body>\n')
        index_file.write('<h1>Combined Doxygen Documentation</h1>\n')
        index_file.write('<ul>\n')

        # Iterate over the packages in the workspace
        packages = os.listdir(workspace_path)
        for package in packages:
            package_path = os.path.join(workspace_path, package)

            # Check if the package has a Doxygen output (docs/index.html)
            index_html_path = os.path.join(package_path, 'docs', 'index.html')
            if os.path.exists(index_html_path):
                # Copy the entire contents of the package's Doxygen output to the combined website
                package_output_path = os.path.join(output_path, package)
                if os.path.exists(package_output_path):
                    shutil.rmtree(package_output_path)
                shutil.copytree(os.path.join(package_path, 'docs'), package_output_path)

                # Update the links in the copied index.html to point to the correct paths
                update_links(package_output_path, package)

                # Add an entry to the index.html file with a link to the package's documentation
                index_file.write('<li><a href="{}/index.html">{}</a></li>\n'.format(package, package))

        index_file.write('</ul>\n')
        index_file.write('</body>\n')
        index_file.write('</html>\n')

def update_links(output_path, package):
    # Get a list of all HTML files in the output directory
    html_files = [file for file in os.listdir(output_path) if file.endswith('.html')]

    # Iterate over the HTML files and update the links
    for html_file in html_files:
        html_path = os.path.join(output_path, html_file)

        # Read the content of the HTML file
        with open(html_path, 'r') as file:
            content = file.read()

        # Replace the links to other packages' index.html with the correct paths
        for other_package in os.listdir(output_path):
            if other_package != package:
                other_package_path = os.path.join(output_path, other_package)
                other_index_path = os.path.join(other_package_path, 'index.html')
                content = content.replace(other_index_path, f'{other_package}/index.html')

        # Write the updated content back to the HTML file
        with open(html_path, 'w') as file:
            file.write(content)

# Example usage
create_doxygen_website('./src', './docs')
