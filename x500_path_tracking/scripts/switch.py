def switch_x_and_y(input_file, output_file):
    with open(input_file, 'r') as f:
        path_data = [line.strip().split(',') for line in f]

    switched_path_data = [(y, x, z,heading) for x, y,z, heading in path_data]

    with open(output_file, 'w') as f:
        for point in switched_path_data:
            f.write(','.join(point) + '\n')

# Provide the file names
input_file = 'zigzag.txt'
output_file = 'zigzag2.txt'

# Call the function
switch_x_and_y(input_file, output_file)
