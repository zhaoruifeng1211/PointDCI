from plyfile import PlyData
import os
import random
import numpy as np
from scipy.linalg import svd
from sys import argv
from scipy.linalg import lstsq

def generate_random_rgb():
    r = random.uniform(0, 1)
    g = random.uniform(0, 1)
    b = random.uniform(0, 1)
    return r, g, b

def fit_plane(points):
    # points: numpy array of shape (n, 3)
    # returns: coefficients a, b, c, d of the plane equation ax + by + cz + d = 0

    # move points to their centroid
    centroid = np.mean(points, axis=0)
    points -= centroid

    # singular value decomposition
    _, _, Vt = svd(points)

    # the normal of the plane is the last row of Vt
    a, b, c = Vt[-1]

    # compute d
    d = -centroid.dot([a, b, c])

    return a, b, c, d
def fit_plane_new(point_cloud):
    # Build the design matrix A
    A = np.column_stack((point_cloud[:, 0], point_cloud[:, 1], np.ones_like(point_cloud[:, 0])))

    # Use z coordinates as the right-hand-side vector
    b = point_cloud[:, 2]

    # Fit the plane with least squares
    # The returned vector x contains the coefficients (a, b, c)
    # of the plane equation ax + by + c = z
    x, residuals, rank, s = lstsq(A, b)

    # Convert to the general plane form
    a, b, c = x
    A_general=a 
    B_general=b
    C_general=-1
    D_general=c

    print("Fitted plane equation: {:.2f}x + {:.2f}y + {:.2f} = z".format(a, b, c))
    return A_general,B_general,C_general,D_general
if len(argv) < 2:
    raise SystemExit("Usage: python plyTOvg.py <input.ply>")

# Open the .ply file
file_path = argv[1]

with open(file_path, 'rb') as f:
    # Load the .ply data
    plydata = PlyData.read(f)

    # Access the vertex data
    vertices = plydata['vertex'] #x,y,x,nx,ny,nz,red,green,blue,alpha

    # Iterate over each vertex

    point_list = []
    for vertex in vertices:
        x = vertex['x']
        y = vertex['y']
        z = vertex['z']
        nx=vertex['nx']
        ny=vertex['ny']
        nz=vertex['nz']
        red=vertex['red']
        green=vertex['green']
        blue=vertex['blue']

        point_list.append((x, y, z, nx, ny, nz, red, green, blue))

# Read numbers from text file with line
numbers = []
map_label=dict()
i=0
with open(file_path[:-4]+".txt", 'r') as file:
    for line in file:
        key=int(line.strip())
        if(map_label.get(key)==None):
            map_label[key] = [i]
        else:
            map_label[key].append(i)   
        i=i+1

print("Number of different numbers:", len(map_label))
for key in map_label.keys():
    print(key, len(map_label[key]))

file_name = f"{file_path[:-4]}.vg"
# points_list2=points_list
with open(file_name, "w") as output_file:
    
    # x y z
    output_file.write("num_points: "+str(len(point_list))+"\n")
    for points in point_list:
        output_file.write(str(points[0])+" "+str(points[1])+" "+str(points[2])+"\n")
    #r g b
    output_file.write("num_colors: "+str(len(point_list))+"\n")
    for points in point_list:
        output_file.write(str(points[6])+" "+str(points[7])+" "+str(points[8])+"\n")
    #nx ny nz
    output_file.write("num_normals: "+str(len(point_list))+"\n")
    for points in point_list:
        output_file.write(str(points[3])+" "+str(points[4])+" "+str(points[5])+"\n")
    
    #group
    output_file.write("num_groups: "+str(len(map_label.keys()))+"\n")
    
    for key in map_label.keys():

        output_file.write("group_type: 0"+"\n")
        output_file.write("num_group_parameters: 4"+"\n")
        # ...

        # for key in map_label.keys():
        # Get the points for the current key
        points = [point_list[i][:3] for i in map_label[key]]
        
        # Fit a plane to the points
        if key==-1:
            cut=0
            if len(points)>10000:
                cut=int(len(points)*0.01)
            points=points[:cut]
        points_array = np.array(points)
        plane_params = fit_plane_new(points_array)
        # Write the plane parameters to the output file
        output_file.write("group_parameters: {} {} {} {}\n".format(plane_params[0], plane_params[1], plane_params[2], plane_params[3]))
        
        # ...
        # output_file.write("group_parameters: a b c d"+"\n")
        output_file.write("group_label: unknown"+"\n")
        rgb_color = generate_random_rgb()
        output_file.write("group_color: "+str(rgb_color[0])+" "+str(rgb_color[1])+" "+str(rgb_color[2])+" "+"\n")
        output_file.write("group_num_point: "+str(len(map_label[key]))+"\n")
        for i in map_label[key]:
            output_file.write(str(i)+" ")
        output_file.write("\n")
        output_file.write("num_children: 0"+"\n")

print("successed")  
