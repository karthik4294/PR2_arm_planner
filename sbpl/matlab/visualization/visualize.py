"""
Plots paths outputted from the xytheta example. 
 
Usage: 
    $ python visualize.py [cfg file] [solution file]
 
"""
import sys
import re
import numpy as np
import matplotlib.pyplot as plt
 
def get_map(filename):
    f = open(filename, 'r')
    lines = f.readlines()
    try:
        map_size_str = [p for p in lines if "discretization" in p][0]
    except IndexError:
        raise Exception("Couldn't find discretization parameter in cfg file.")
 
    try:
        cell_size_str = [p for p in lines if "cellsize" in p][0]
    except IndexError:
        raise Exception("Couldn't find cell size in cfg file")
    
    try:
        size_x, size_y = re.compile('([0-9]+) ([0-9]+)').findall(map_size_str)[0]
        size_x = int(size_x)
        size_y = int(size_y)
    except IndexError:
        raise Exception("Couldn't parse distretization param in cfg file.")
 
    try:
        search = re.compile('([0-9]*\.[0-9]*)')
        cell_size = float(search.findall(cell_size_str)[0])
    except IndexError:
        raise Exception("Couldn't parse cell size")
 
    map_index = [p[0]+1 for p in enumerate(lines) if "environment" in p[1]]
    map_lines = lines[map_index[0]:]
    map_lines = [p.strip().split() for p in map_lines]
    flattened_values = map(int, [item for sublist in map_lines for item in 
                                 sublist])
    map_values = np.array(flattened_values).reshape(size_y, size_x)
    return (map_values, cell_size)
 
 
def plot_planned_path(filename, ax):
    f = open(filename, 'r')
    lines = f.readlines()
    sol_values = np.array([map(int, p.strip().split()) for p in lines])
    print sol_values
    ax.plot(sol_values[:,0], sol_values[:,1], 'y-') 
 
def plot_cont_planned_path(filename, cell_size, ax):
    f = open(filename, 'r')
    lines = f.readlines()
    sol_values = np.array([map(float, p.strip().split(',')) for p in lines if
                           len(p.strip().split(',')) == 3])
    ax.plot(sol_values[:,0]/cell_size, sol_values[:,1]/cell_size, 'y-') 
 
if __name__ == '__main__':
    (map_values, cell_size) = get_map(sys.argv[1])
    fig = plt.figure()
    ax = fig.add_subplot(111)
    bx = fig.add_subplot(111)
    plt.imshow(map_values, vmin=0, vmax=1)
    plt.ylim([0, map_values.shape[0]])
    plt.xlim([0, map_values.shape[1]])
 
    plot_planned_path(sys.argv[2], ax)
    #plot_planned_path(sys.argv[3], bx)
    plt.show()
    #plt.show()

