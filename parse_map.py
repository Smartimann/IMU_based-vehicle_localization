from lxml import etree 
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse

from progress.bar import ChargingBar


# function to parse the xml document
def save_lanes_as_dict(lanes): 
    lanes_dict = {
        'laneOffset': None,
        'laneSection': None, 
    }
    
    lane_section_dict = {
        's': None,
        'left': None, 
        'center': None, 
        'right': None,
    }
    left_dict = {
        'lane': None
    }
    
    link_dict = {
        'predecessor': None,
        'successor': None
    }
    
    lane_dict = {
        'id': None, 
        'type': None, 
        'level': None, 
        'link': None,
        'width': None, 
        'roadMark': None, 
        'userData': None
    }
    for child in lanes: 
        if (child.tag == 'laneOffset'):
            lanes_dict['laneOffset'] = {
                's' : child.get('s'),
                'a' : child.get('a'),
                'b' : child.get('b'),
                'c' : child.get('c'),
                'd' : child.get('d')
            }
            
        elif (child.tag == 'laneSection'): 
            lane_section_dict['s'] = child.get('s')
            left_lanes = []
            center_lanes = []
            right_lanes = []
            for direction in child: 
                for lane in direction:
                    widths = []

                    lane_dict['id'] = lane.get('id')
                    lane_dict['type'] = lane.get('type')
                    lane_dict['level'] = lane.get('level')
                    
                    ## iterate through lanes
                    for child in lane: 
                     
                        #Parse all links in lanes
                        if (child.tag == 'link'): 
                            for link in child: 
                                if (link.tag == 'predecessor'): 
                                    link_dict['predecessor'] = {
                                        'id': link.get('id')
                                    }
                                elif (link.tag == 'successor'): 
                                    link_dict['successor'] = {
                                        'id': link.get('id')
                                    }
                            lane_dict['link'] = link_dict
                        elif(child.tag == 'width'): 
                            widths.append({
                                'sOffset': child.get('sOffset'), 
                                'a': child.get('a'), 
                                'b': child.get('b'), 
                                'c': child.get('c'),
                                'd': child.get('d')
                            })
                        
                    
                    lane_dict['width'] = widths
                    if (direction.tag == 'left'): 
                        left_lanes.append(lane_dict)
                    elif(direction.tag == 'center'): 
                        center_lanes.append(lane_dict)
                    elif(direction.tag=='right'): 
                        right_lanes.append(lane_dict)
                    
                lane_section_dict['left'] = left_lanes
                lane_section_dict['center'] = center_lanes
                lane_section_dict['right'] = right_lanes
    lanes_dict['laneSection'] = lane_section_dict
    return lanes_dict
def save_links_as_dict(link): 
    link_dict = {
        'predecessor': None, 
        'successor': None
    }
    for child in link: 
        if (child.tag == 'predecessor'): 
            link_dict['predecessor'] = {
                'elementType': child.get('elementType'),
                'elementId': child.get('elementId'),
                'contactPoint': child.get('contactPoint')
            }
        elif (child.tag == 'successor'): 
            link_dict['successor'] = {
                'elementType': child.get('elementType'),
                'elementId': child.get('elementId'),
                'contactPoint': child.get('contactPoint')
            }
    
    return link_dict
def save_type_as_dict(road_type): 
    type_dict = {
       's': None, 
       'type': None, 
       'speed': None
    }
        
    speed_dict = {
        'max': None, 
        'unit': None
    }
    
    type_dict['s'] = road_type.get('s')
    type_dict['type'] = road_type.get('type')
    speed_dict['max'] = road_type[0].get('max')
    speed_dict['unit'] = road_type[0].get('unit')  
    type_dict['speed'] = speed_dict
def save_planView_as_dict(planView): 
    geometries = []
    for child in planView: 
        if (child.tag=='geometry'): 
            line = None
            arc = {
                'curvature': None
            }
            if (child[0].tag == 'line'): 
                line = True
            elif(child[0].tag == 'arc'): 
                arc['curvature'] = child[0].get('curvature')
            geometries.append({
                's': child.get('s'),
                'x': child.get('x'),
                'y': child.get('y'),
                'hdg': child.get('hdg'),
                'length': child.get('length'),
                'line': line, 
                'arc': arc
            })
    return geometries    
def save_road_as_dict(road): 
    road_dict = {
        'name': road.get('name'), 
        'length': road.get('length'),
        'id': road.get('id'), 
        'junction': road.get('junction'),
        'link': None, 
        'type': None,
        'planView': None, 
        'elevationProfile': None, 
        'lateralProfile': None, 
        'lanes': None
    }
    for child in road: 
        if (child.tag == 'link'): 
            road_dict['link'] = save_links_as_dict(child)
        
        elif (child.tag == 'type'): 
            road_dict['type'] = save_type_as_dict(child)
                
        elif (child.tag == 'planView'): 
            road_dict['planView'] = save_planView_as_dict(child)
        #elif (child.tag == 'elevationProfile'): 
            
        #elif (child.tag == 'lateralProfile'): 
        elif (child.tag == 'lanes'): 
            road_dict['lanes'] = save_lanes_as_dict(child)
  
    return road_dict

# mathematical lambda functions to calculate the actual lines
def calculate_line(length, hdg, x,y):      
    return lambda t: (np.array([np.cos(hdg), np.sin(hdg)]) * length * t) + np.array([x,y])

def calculate_arc(length, hdg, curvature, x,y): 
    return lambda t: np.array([x,y]) + 1/curvature * np.array([np.cos(t/(1/curvature)), np.sin(t/(1/curvature))])

def deriv_arc(length, hdg, curvature, x,y): 
    return lambda t: np.array([-np.sin(t*curvature), np.cos(t*curvature)])

def deriv_line(length, hdg): 
    return lambda t: np.array([-np.sin(hdg), np.cos(hdg)])*length

def get_hdg_vec(hdg): 
    return np.array([np.cos(hdg), np.sin(hdg)])

    
# returns the arc with correct heading and length
def get_adjusted_arc(x,y,hdg, length, curvature):
    # get the derivation of this arc
    deriv_of_arc = deriv_arc(length, hdg, curvature, x,y)
    # get the derivation of according tangent
    deriv_of_line = deriv_line(length, hdg)
    # get the tempo vector
    tempo_vec = deriv_of_line(0)
    #get the gradient of the line
    line_gradient = tempo_vec[1] / tempo_vec[0]
    #get the stepsize
    step = 0.001
    # get all the T values
    arc_Ts = np.arange(0, 2*np.pi + step, step)
    
    arc_gradients = []
    line_gradients = []
    #iterate over t values
    for t in arc_Ts: 
        arc_tempo_vec = deriv_of_arc(t)
        if (arc_tempo_vec[0] == 0): 
            arc_tempo_vec[0] = 0.00000000001
        # append the gradients for each t value of the curve in a list
        arc_gradients.append(arc_tempo_vec[1] / arc_tempo_vec[0])
        line_gradients.append(line_gradient)
    
    #find the index with the lowest difference between gradients
    gradient_diffs = np.array(arc_gradients)-np.array(line_gradients)
    min_diff_index = np.argmin(gradient_diffs)
    t = arc_Ts[min_diff_index]
    

    # start at the according t value
    arc_Ts = np.arange(t, 10000 + step, step)
    arc = calculate_arc(length, hdg, curvature, x,y)
    adjusted_arc_points = []
    arc_length = 0
    # Do the parametrisation and translate all points so that start of the curve is at x,y
    for t in arc_Ts: 
        adjusted_arc_points.append(arc(t))
        arc_length += np.linalg.norm(deriv_of_arc(t)*step)
        # draw arc as long as it stays in length
        if (arc_length >= length): 
            break
        
    
    # translate points
    adjusted_arc_points = np.array(adjusted_arc_points)
    difference_vec = np.array(adjusted_arc_points[0] - np.array([x,y])) 
    adjusted_arc_points = adjusted_arc_points - difference_vec
    return adjusted_arc_points

# returns array of points with all geometries 
def get_all_geoms_as_points(geometries): 
    plot_lines = []
    plot_arcs = np.empty([0,2])
    step = 0.001
    Line_Ts = np.arange(0,1+step,step)
    Arc_Ts = np.arange(0, 2*np.pi + step, step)
    for geometry in geometries: 
        current = geometries[geometry] 
        for g in current: 
            #print(g)
            if (g['line']): 
                #print("Line")
                line = calculate_line(float(g['length']), float(g['hdg']), float(g['x']),float(g['y']))
                points = [line(t) for t in Line_Ts]
                plot_lines += points
            elif (g['line'] != True and g['arc']['curvature'] != None): 
                #print(g['arc']['curvature'])
                #arc = calculate_arc(float(g['length']), float(g['hdg']), float(g['arc']['curvature']),float(g['x']),float(g['y']))
                points = get_adjusted_arc(float(g['x']),float(g['y']), float(g['hdg']), float(g['length']), float(g['arc']['curvature']))
                plot_arcs = np.concatenate([plot_arcs, points])  
    plot_lines = np.array(plot_lines)
    plot_arcs = np.array(plot_arcs)
    return plot_lines, plot_arcs

def plot_map(lines, arcs): 
    plt.scatter(lines[:,0],lines[:,1], c='blue', label="Lines",s=0.1)
    plt.scatter(arcs[:,0],arcs[:,1], c='orange', label="Arcs", s= 1)
    plt.show()







def main(): 
    argparser = argparse.ArgumentParser(
        description='Script to create and plot the map')
    argparser.add_argument(
        '--filename',
        metavar='f',
        dest='filename',
        default='data',
        help='Typt the name of the xodr file')

    args = argparser.parse_args()

    print("Opening File: " + args.filename+".xodr")
    filename = "data/"+args.filename+".xodr"
    with open(filename) as fi: 
        xml_root = etree.parse(fi).getroot()
    all_roads = {}
    print("Parsing all files")
    for child in xml_root: 
        if (child.tag == 'road'): 
            current_road = save_road_as_dict(child)
            all_roads[int(current_road['id'])] = current_road
    print("Creating dataframe from dict")
    df = pd.DataFrame.from_dict(all_roads, orient='index')
    geometries = {}
    print("Extracting all geometries")
    for index, row in df.iterrows(): 
        geometries[index] = row['planView']
    print("Calculating parametric curves")
    lines, arcs = get_all_geoms_as_points(geometries=geometries)
    print("Plotting Curves")
    plot_map(lines, arcs)
if __name__ == '__main__':

    main()
