import time
from dronekit import LocationGlobalRelative

# import to do path gen stuff automatically
import folium
import csv
from folium import plugins
import webbrowser  # so folium can make a map display
import numpy as np
import math
from math import radians, cos
from geopy.distance import geodesic
from geopy.point import Point

# Gives this file "project import context"
import sys, os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)


FRAME_SIZE_OVERLAP = 0.9  # overlap of the frames in the search pattern
ALTITUDE = 4


def verify_box_distances(box, expected_distance_yards=30):
    """
    Verify if the distance between each corner of the box is as expected.
    :param box: List of points [(id, lat, lon), ...] representing the corners of the box
    :param expected_distance_yards: The expected distance between adjacent points in yards (default: 30 yards)
    :return: A list of booleans where True means the distance matches the expected value
    """
    yard_to_meter = 0.9144  # Conversion factor
    expected_distance_meters = expected_distance_yards * yard_to_meter
    
    # Convert box points into a sequence of tuples (latitude, longitude)
    coords = [(point[1], point[2]) for point in box]
    
    # Compare distances between adjacent points (loop around to the start for the last segment)
    results = []
    for i in range(len(coords)):
        coord1 = coords[i]
        coord2 = coords[(i + 1) % len(coords)]  # Next point, loop around to first
        distance = equirectangular_approximation(coord1, coord2)
        results.append(abs(distance - expected_distance_meters) <= .1)  # Allow a small tolerance
    
    return results

def generate_box(current_location, heading):
    """
    Generate a 30x30 yard box with the current location as the bottom-right corner.
    :param current_location: tuple (latitude, longitude) of the current position
    :param heading: heading angle in degrees, where 0 is north
    :return: list of tuples representing the four corner points (latitude, longitude)
    """
    # Define 30 yards in meters
    yard_to_meter = 0.9144
    side_offset = 30 * yard_to_meter  # 30 yards in meters
    
    # Bottom-Right (starting location)
    bottom_right = Point(current_location[0], current_location[1])
    
    # Bottom-Left: 30 yards to the left
    left_heading = (heading + 270) % 360
    bottom_left = geodesic(meters=side_offset).destination(bottom_right, left_heading)
    
    # Top-Left: 30 yards forward from the Bottom-Left
    top_left = geodesic(meters=side_offset).destination(bottom_left, heading)
    
    # Top-Right: 30 yards forward from the Bottom-Right
    top_right = geodesic(meters=side_offset).destination(bottom_right, heading)
    
    # Return points as a list of tuples
    return [
        [1, top_left.latitude, top_left.longitude],  # Top-Left
        [2, top_right.latitude, top_right.longitude],  # Top-Right
        [3, bottom_right.latitude, bottom_right.longitude],  # Bottom-Right
        [4, bottom_left.latitude, bottom_left.longitude],  # Bottom-Left
    ]


def generate_folium_map(waypoints, fence_waypoint_array):
    global home_location
    global home_latitude
    global home_longitude
    # Create a Folium map centered on home location
    my_map = folium.Map(location=[waypoints[0][0], waypoints[0][1]], zoom_start=20, max_zoom=25)

    """
    # add a manually acquired image overlay to the map
    folium.raster_layers.ImageOverlay(
        image="file:///C:/Users/Admin/PycharmProjects/TestUartUax/Stuff/lot.jpg",
        bounds=[[folium_bbox[1], folium_bbox[0]], [folium_bbox[3], folium_bbox[2]]],
        opacity=0.7,
        name='Landsat Image Overlay'
    ).add_to(my_map)
    """
    print(fence_waypoint_array)

    thefencepoint = (fence_waypoint_array[0][1], fence_waypoint_array[0][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {1}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[1][1], fence_waypoint_array[1][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {2}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[2][1], fence_waypoint_array[2][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {3}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[3][1], fence_waypoint_array[3][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {4}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    # Create a PolyLine
    polyline = folium.PolyLine(locations=waypoints, color='blue', weight=3, opacity=0.0).add_to(my_map)

    # Add arrows to the PolyLine
    plugins.PolyLineTextPath(
        polyline,
        '\u2192',  # Unicode arrow character (right arrow)
        repeat=True,
        offset=6,
        attributes={'fill': 'red', 'font-weight': 'bold', 'font-size': '35'}
    ).add_to(my_map)

    '''
    new_waypoint_index = 0
    # Add waypoints to the map
    for waypoint in waypoints:
        new_waypoint_index += 1
        print(f"waypoint: {waypoint}")
        folium.Marker(waypoint, popup=f'Waypoint No. {new_waypoint_index}: {waypoint}',
                      icon=folium.Icon(color='green')).add_to(my_map)
    '''
    # Save the map as a html file
    my_map.save('generated_search_pattern_waypoints.html')

    # Optionally, open the HTML file in a web browser
    open_in_browser = True
    if open_in_browser is True:
        webbrowser.open('generated_search_pattern_waypoints.html')

    return


def equirectangular_approximation(coord1, coord2,alt = None):
    """
    Calculate the approximate straight-line distance between two GPS coordinates using the Equirectangular approximation.
    Parameters:
    - coord1: Tuple of (latitude, longitude) for the first point.
    - coord2: Tuple of (latitude, longitude) for the second point.
    Returns:
    - Distance in meters.
    """
    # Earth radius in meters
    R = 6371000 
    # Convert latitude and longitude from degrees to radians
    lat1, lon1 = map(radians, coord1)
    lat2, lon2 = map(radians, coord2)
    # Equirectangular approximation
    x = (lon2 - lon1) * cos((lat1 + lat2) / 2)
    y = lat2 - lat1
    # Return Distance in kilometers
    if alt is not None:
        distance = R * math.sqrt(x ** 2 + y ** 2)+ abs(alt/1000.0)
    else:
        distance = R * math.sqrt(x ** 2 + y ** 2)
    return distance


def save_waypoints_to_csv(waypoints, csv_filename):
    waypoint_index = 0
    with open(csv_filename, 'w', newline='') as csvfile:
        # create and write the header
        fieldnames = ['latitude', 'longitude']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        # run thru each waypoint and add to the csv file
        for waypoint in waypoints:
            waypoint_index += 1
            writer.writerow({'latitude': waypoint[0], 'longitude': waypoint[1]})  # add waypoint to file

def generate_zig_zag_path_waypoints(point1, point2, point3, num_cols, num_rows):
    # Calculate the side vectors
    side_vector1 = np.array(point2) - np.array(point1)
    side_vector2 = np.array(point3) - np.array(point2)

    # Calculate the angle between the sides
    dot_product = np.dot(side_vector1, side_vector2)
    norm_product = np.linalg.norm(side_vector1) * np.linalg.norm(side_vector2)
    cosine_angle = dot_product / norm_product
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))

    # Choose the side with an angle greater than 180 degrees so the parallelogram doesnt cross itself
    if angle > 180:
        fourth_point = (point3[0] + side_vector1[0], point3[1] + side_vector1[1])
    else:
        fourth_point = (point1[0] + side_vector2[0], point1[1] + side_vector2[1])

    x = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)  # Width of the grid
    y = math.sqrt((point2[0] - point3[0]) ** 2 + (point2[1] - point3[1]) ** 2)  # Height of the grid

    print(x)
    print(y)

    # Define the corners of the rectangle
    rectangle = np.array([
        [0, 0],  # Bottom-left corner
        [x, 0],  # Bottom-right corner
        [x, y],  # Top-right corner
        [0, y]  # Top-left corner
    ])

    # Define the corners of the known parallelogram
    parallelogram = np.array([
        [point1[0], point1[1]],  # Bottom-left corner
        [point2[0], point2[1]],  # Bottom-right corner
        [point3[0], point3[1]],  # Top-right corner
        [fourth_point[0], fourth_point[1]]  # Top-left corner
    ])

    # Create coordinate vectors for the columns and rows
    cols = np.linspace(0, x, num_cols)
    rows = np.linspace(0, y, num_rows)

    # Create the meshgrid
    col_mesh, row_mesh = np.meshgrid(cols, rows, indexing='xy')

    # Flatten the meshgrid arrays and combine them into one array of xy pairs
    grid_points = np.column_stack((col_mesh.ravel(), row_mesh.ravel()))

    # reverse every other row in the array
    # n = int(math.floor(x) / col_spacing)
    n = num_cols
    result = grid_points.copy()
    for i in range(0, len(grid_points), n * 2):
        result[i:i + n] = np.flip(result[i:i + n], axis=0)

    grid_points = result

    # Convert rectangle and parallelogram to numpy arrays
    rectangle = np.array(rectangle)
    parallelogram = np.array(parallelogram)

    # Append a column of ones to the rectangle coordinates for the translation component
    rectangle_with_ones = np.hstack([rectangle, np.ones((4, 1))])

    # Find the transformation matrix that maps the rectangle onto the parallelogram
    transform_matrix, _, _, _ = np.linalg.lstsq(rectangle_with_ones, parallelogram, rcond=None)

    # Apply the transformation matrix to the rectangle
    skewed_rectangle = np.dot(rectangle_with_ones, transform_matrix)

    # Append a column of ones to the rectangle coordinates for the translation component
    grid_with_ones = np.hstack([grid_points, np.ones((len(grid_points), 1))])

    skewed_grid = np.dot(grid_with_ones, transform_matrix)
    print(skewed_grid)
    skewed_rectangle, skewed_grid_points = skewed_rectangle[:, :2], skewed_grid[:,
                                                                    :2]  # Exclude the last column of ones

    skewed_grid_points = np.flip(skewed_grid_points, axis=0)

    return skewed_grid_points


def load_waypoints_from_csv(file_path):
    global ALTITUDE
    csv_loaded_waypoints = []
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            latitude = float(row['latitude'])
            longitude = float(row['longitude'])
            altitude = float(ALTITUDE)
            csv_loaded_waypoints.append(LocationGlobalRelative(latitude, longitude, altitude))
    return csv_loaded_waypoints

def extract_gv_waypoints(waypoints, rows, cols):
    '''
    For each coordinate pair that makes up a line (parallel to rows) in the UAV flight path,
    save the midpoint as a GV waypoint. As a result, the GV waypoints should make a line straight
    down and in center of the zig-zag UAV search flightpath.
    '''
    result = [[0 for i in range(rows)] for k in range(cols)]
    p = 0
    for i in range(0, len(waypoints), 2):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i+1]
        
        result[p][0] = (x1+x2)/2
        result[p][1] = (y1+y2)/2
        p += 1
    
    print(f"Waypoints for GV extracted: \n{result}")
    return result
    

def run_path_generation(vehicle, heading, frame_width_meters, frame_height_meters):
    # Wait for the vehicle to have a GPS fix
    fence_waypoint_array = []
    while not vehicle.gps_0.fix_type:
        print("Waiting for the GPS to have a fix...")
        time.sleep(1)
    print("gpsfixed")

    # Download the vehicle waypoints (commands). Wait until download is complete.
    print("getting the cmds position")

    # Request the total number of mission items
    vehicle.commands.download()

    # Wait for the mission download to complete
    vehicle.commands.wait_ready()

    # Get the list of downloaded mission items
    cmds = vehicle.commands
    print(f"cmds: {cmds}")

    # Print waypoint data
    if len(cmds) > 0:
        print("Mission commands:")
        for cmd in cmds:
            print(f"{cmd.seq}, {cmd.x}, {cmd.y}, {cmd.z}")
            fence_waypoint_array.append([cmd.seq, cmd.x, cmd.y, cmd.z])
        # Specify the rectangular area with four corners defined
        top_left_corner = (fence_waypoint_array[0][1], fence_waypoint_array[0][2])  # all copied from mission planner
        top_right_corner = (fence_waypoint_array[1][1], fence_waypoint_array[1][2])
        bottom_right_corner = (fence_waypoint_array[2][1], fence_waypoint_array[2][2])
        bottom_left_corner = (fence_waypoint_array[3][1], fence_waypoint_array[3][2])
        landing_zone_waypoint = (fence_waypoint_array[4][1], fence_waypoint_array[4][2])  # The location to land the drone at after finishing search and shoot

    else:
        print("No mission commands downloaded.")
        print("Using auto generated coordinates.")
        autoBoxWpArray =  generate_box((vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon),
                                       heading)
        top_left_corner = (autoBoxWpArray[0][1], autoBoxWpArray[0][2])
        top_right_corner = (autoBoxWpArray[1][1], autoBoxWpArray[1][2])
        bottom_right_corner = (autoBoxWpArray[2][1], autoBoxWpArray[2][2])
        bottom_left_corner = (autoBoxWpArray[3][1], autoBoxWpArray[3][2])

        landing_zone_waypoint = (autoBoxWpArray[3][1], autoBoxWpArray[3][2])  # Landing Zone in autobox is just designated as Bottom Left

    # Wait for the home location to be set
    while vehicle.home_location is None:
        print("Waiting for the home location to be set...")
        time.sleep(1)

    # Retrieve home location
    home_location = vehicle.home_location

    # home location is not very useful because it changes to current loc every arming
    # Parse and print home location information
    if home_location is not None:
        home_latitude = home_location.lat
        home_longitude = home_location.lon
        home_altitude = home_location.alt
        print(f"Home Location: Latitude={home_latitude}, Longitude={home_longitude}, Altitude={home_altitude}")
    else:
        print("Home location is not available.")
        sys.exit("Exiting due to no home position")

    # print the fence data defined as 4 normal waypoints in mission planner


    # get width and length of the search area in meters
    horizontal_distance = equirectangular_approximation(top_left_corner, top_right_corner)
    vertical_dist = equirectangular_approximation(top_right_corner, bottom_right_corner)

    print(f"Horiz: {horizontal_distance}, Vert: {vertical_dist} ")
    # Number of rows and columns in the zigzag grid based on the size of the field and radius of points
    try:
        cols = int(horizontal_distance // (frame_width_meters * FRAME_SIZE_OVERLAP))
        rows = int(vertical_dist // (frame_height_meters * FRAME_SIZE_OVERLAP))
    except Exception as e:
        print("rows and cols too smol using 2 atleast")
        cols = 2
        rows = 2

    if rows < 2 or cols < 2:
        cols = 2
        rows = 2

    rows = 2

    # Generate zigzag waypoints
    # waypoints = generate_zigzag_waypoints(bottom_left_corner, top_right_corner, rows, cols)
    waypoints = generate_zig_zag_path_waypoints(top_left_corner, top_right_corner, bottom_right_corner, rows, cols)
    
    # Find waypoints for GV based on scout UAV flight path
    gv_waypoints = extract_gv_waypoints(waypoints, rows, cols)
    '''
    INSERT COMMS LOGIC HERE
    '''

    # Save waypoints to CSV
    csv_filename = 'generated_search_pattern_waypoints.csv'
    save_waypoints_to_csv(waypoints, csv_filename)

    print(f'Waypoints saved to {csv_filename}')

    # should the Code generate a HTML map also?
    genMap = True
    if genMap is True:
        if len(fence_waypoint_array) > 0: 
            generate_folium_map(waypoints, fence_waypoint_array)
        else:
            generate_folium_map(waypoints, autoBoxWpArray)
    # Load waypoints from CSV file
    waypoints = load_waypoints_from_csv('generated_search_pattern_waypoints.csv')

    return waypoints, top_left_corner, top_right_corner, landing_zone_waypoint