import pandas as pd
import matplotlib.pyplot as plt
from SearchAlgoScript import *

# Load the CSV file
search_waypoints = load_waypoints_from_csv('generated_search_pattern_waypoints.csv')
search2_waypoints = load_second_waypoints_from_csv('generated_search_pattern_waypoints.csv')

# Convert to DataFrames
df = pd.DataFrame([(wp.lat, wp.lon) for wp in search_waypoints], columns=['latitude', 'longitude'])
df2 = pd.DataFrame([(wp.lat, wp.lon) for wp in search2_waypoints], columns=['latitude', 'longitude'])

# Extract coordinates
latitudes = df['latitude']
longitudes = df['longitude']
latitudes2 = df2['latitude']
longitudes2 = df2['longitude']

# Plotting both paths
plt.figure(figsize=(8, 8))
plt.plot(longitudes, latitudes, marker='o', linestyle='-', label='Original Path')
plt.plot(longitudes2, latitudes2, marker='x', linestyle='--', label='Offset Path')
plt.title('GPS Waypoints Comparison')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
