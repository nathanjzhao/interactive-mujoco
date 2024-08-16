import trimesh
import os
import argparse

def convert_stl_to_obj(input_dir, output_dir):
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Iterate over all files in the input directory
    for filename in os.listdir(input_dir):
        if filename.endswith('.stl') or filename.endswith('.STL'):
            # Load the STL file
            mesh = trimesh.load_mesh(os.path.join(input_dir, filename))
            
            # Define the output OBJ file path
            obj_filename = os.path.splitext(filename)[0] + '.obj'
            obj_filepath = os.path.join(output_dir, obj_filename)
            
            # Export the mesh to OBJ format
            mesh.export(obj_filepath)
            print(f"Converted {filename} to {obj_filename}")

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Convert STL files to OBJ files.')
    parser.add_argument('--inp', type=str, help='Directory containing STL files')
    parser.add_argument('--out', type=str, help='Directory to save OBJ files')

    # Parse arguments
    args = parser.parse_args()

    # Convert STL files to OBJ files
    convert_stl_to_obj(args.inp, args.out)

if __name__ == '__main__':
    main()