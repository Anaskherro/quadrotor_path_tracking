def add_z_coordinate_to_file(input_file, output_file):
    with open(input_file, 'r') as f:
        path_with_heading = [tuple(map(float, line.strip().split(','))) for line in f]

    path_with_xyz_heading = [(x, y, 2.0, heading) for x, y, heading in path_with_heading]

    with open(output_file, 'w') as f:
        for point in path_with_xyz_heading:
            f.write(','.join(map(str, point)) + '\n')

# Provide the file names
input_file = 'path2heading.txt'
output_file = 'path_with_xyz_heading.txt'

# Call the function
add_z_coordinate_to_file(input_file, output_file)

