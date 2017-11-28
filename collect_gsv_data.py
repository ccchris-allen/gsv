# standard libraries
from math import atan2, cos, sin, radians, degrees
from functools import partial
import matplotlib.pyplot as plt

# non-standard libraries (may need to install)
import streetview as sv
import numpy as np
from PIL import Image
from pyproj import Proj, transform
from shapely.geometry import Point, LineString
from shapely.ops import transform as geom_transform
#
# config.py is ignored by git so will need to be created with your own key
from config import API_KEY


def calculate_bearing(x1, y1, x2, y2):
    """
    Calculates the bearing between two points.

    The formula used is the following:

        bearing = atan2(sin(dx).cos(y2),
                        cos(y1).sin(y2) - sin(y1).cos(y2).cos(dx)

    Returns: 
        Bearing in degrees (float)

    CREDIT: https://gist.github.com/jeromer/2005586
    """
    
    inputs_are_floats = all([(type(n) is float) for n in (x1, y1, x2, y2)]) 

    if not inputs_are_floats:
        raise TypeError("Arguments must be floats")

    y1, y2 = map(radians, (y1, y2))
    dx = radians(x2 - x1)

    x = sin(dx) * cos(y2)
    y = cos(y1) * sin(y2) - (sin(y1) * cos(y2) * cos(dx))

    # bearing will be between [-180, 180]
    bearing = degrees(atan2(x, y))

    # convert bearing to [0, 360] 
    return (bearing + 360) % 360 


def reproject(geometry, from_crs, to_crs):
    """
    Reproject geometry to another crs.

    CREDIT: https://gis.stackexchange.com/questions/127427/
                transforming-shapely-polygon-and-multipolygon-objects
    """
    
    from_crs_str = "epsg:{}".format(from_crs)
    to_crs_str = "epsg:{}".format(to_crs)

    project = partial(transform, Proj(init=from_crs_str), Proj(init=to_crs_str))

    return geom_transform(project, geometry)


def save_image(img, fname):
    img = Image.open(img)
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(np.asarray(img))
    
    print "Saving", fname
    plt.savefig(fname)



    panoid_infos = sv.panoids(lat, lon)

    if not panoid_infos:
        print "Oops, no panoids"
        return None

    # grab last_n years where data is available
    for i in range(last_n):
        info = panoid_infos.pop(0)

        pid = info['panoid']
        year = info.get('year', 2017)
        plat = info['lat']
        plng = info['lon']

        sv.api_download(pid, bearing, output_dir, year=year, lat=plat,
                lng=plng, key=key)

    
def cut(line, dist):
    """
    Cuts a line in two at a distance from the starting point.

    CREDIT: Found online (somewhere...)
    """

    if dist <= 0.0 or dist >= line.length:
        return [LineString(line)]
    
    coords = list(line.coords)

    for i, p in enumerate(coords):
        pd = line.project(Point(p))

        if pd == dist:
            return [
                LineString(coords[:i+1]),
                LineString(coords[i:])
            ]

        if pd > dist:
            # get Cut Point
            cp = line.interpolate(dist)
            cp_coords = [(cp.x, cp.y)]

            return [
                LineString(coords[:i] + cp_coords),
                LineString(cp_coords + coords[i:])
            ]


def sample_road(geometry, to_crs, from_crs=4326, delta=30.0):
    """
    Sample road geometry (must be a LineString) at intervals specified
    by `delta` (which is in meters).

    Returns:
        List of sampling points, where each item is a tuple that specifies
        the WGS84 longitude, latitude, and bearing:

        [((-117.34, 32.153), 90.0), ((-117.363, 32.13), 98.0), ... ]
    """

    segment = reproject(geometry, from_crs, to_crs)
    samples = []

    while segment.length > delta:
        (x1, y1), (x2, y2) = segment.coords[:2]

        p1_coords = reproject(Point((x1, y1)), to_crs, from_crs).coords[0]
        p2_coords = reproject(Point((x2, y2)), to_crs, from_crs).coords[0]

        #bearing = calculate_bearing(x1, y1, x2, y2)
        bearing = calculate_bearing(p1_coords[0], p1_coords[1], 
                                    p2_coords[0], p2_coords[1])

        samples.append((p1_coords, (bearing + 90.0) % 360))
        samples.append((p1_coords, (bearing - 90.0) % 360))

        segment = cut(segment, delta)[-1]

    return samples


if __name__ == "__main__":
    import os
    
    OUTPUT_DIR = "./images/"

    # create output directory if doesn't exist
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        
    # test on a road segment (in Chicago)
    vertices = [
        (-117.127304, 32.695884),
        (-117.125298, 32.696001)
    ]

    road = LineString(vertices)
    
    # convert to UTM and sample road segment every 30 meters
    samples = sample_road(road, delta=30.0, to_crs=32611)

    for (lon, lat), bearing in samples:
        print "Input lon =", lon
        print "Input lat =", lat
        print "Input bearing =", bearing
        save_streetview_image(lon, lat, bearing, OUTPUT_DIR, last_n=2)

