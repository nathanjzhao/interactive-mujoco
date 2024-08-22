import xml.etree.ElementTree as ET

def convert_mjcf_to_urdf(mjcf_file):
    # Parse MJCF file
    tree = ET.parse(mjcf_file)
    root = tree.getroot()

    # Create URDF root element
    urdf_root = ET.Element('robot', name='converted_robot')

    # Conversion logic
    for child in root:
        if child.tag == 'worldbody':
            for body in child:
                # URDF body tag
                urdf_body = ET.SubElement(urdf_root, 'link', name=body.get('name', 'unknown'))
                
                for geom in body.findall('geom'):
                    # URDF visual tag
                    urdf_visual = ET.SubElement(urdf_body, 'visual')
                    urdf_geometry = ET.SubElement(urdf_visual, 'geometry')
                    
                    # Convert geom type
                    geom_type = geom.get('type', 'box')
                    if geom_type == 'sphere':
                        ET.SubElement(urdf_geometry, 'sphere', radius=geom.get('size', '1.0'))
                    elif geom_type == 'cylinder':
                        ET.SubElement(urdf_geometry, 'cylinder', radius=geom.get('size', '1.0'), length=geom.get('length', '1.0'))
                    else:  # default to box
                        ET.SubElement(urdf_geometry, 'box', size=geom.get('size', '1.0 1.0 1.0'))
                    
                    # URDF material tag (if applicable)
                    material = geom.get('material')
                    if material:
                        urdf_material = ET.SubElement(urdf_visual, 'material', name=material)
                        ET.SubElement(urdf_material, 'color').text = '0.5 0.5 0.5 1.0'  # Example color

                # Add more tags and attributes as needed

    # Save URDF file
    urdf_file = mjcf_file.replace('.xml', '-test.urdf')
    urdf_tree = ET.ElementTree(urdf_root)
    urdf_tree.write(urdf_file, encoding='utf-8', xml_declaration=True)

if __name__ == "__main__":
    # Define the input and output file names
    input_file = 'examples/scenes/warehouse-env.xml'
    output_file = 'examples/scenes/warehouse-env-test.urdf'

    # Convert the MJCF file to URDF
    convert_mjcf_to_urdf(input_file)
    print(f'Converted {input_file} to {output_file}.')
