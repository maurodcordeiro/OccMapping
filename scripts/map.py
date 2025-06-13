# !/usr/bin/env python3


import rospy as rp
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
import tf
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

def logtop(l):
    return 1 - (1/(1+np.exp(l)))

def ptolog(p):
    return np.log(p/(1-p))

# This will run every time the laser publishes a new message
def scan_callback(msg: LaserScan):
    global x_size, y_size, res, occmap
    global l_occ, l_free, l_prior

    # Get the map->base_scan transform
    try:
        tf_transform = tf_buffer.lookup_transform(
            "map",
            "base_scan",
            rp.Time(0),
            rp.Duration(1.0)
        )
        rp.loginfo("Success with lookup?")
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        rp.logwarn("Failed Lookup: %s", e)
        return

    # Get corrected robot position cell (x0 and y0)
    pos = tf_transform.transform.translation
    grid_x0 = int((pos.x - msg_occ.info.origin.position.x) / res)
    grid_y0 = int((pos.y - msg_occ.info.origin.position.y) / res)

    # Acumulate occupied and free cells
    occ_x, occ_y = [], []
    free_x, free_y = [], []

    for i, r in enumerate(msg.ranges):

        # Skip out of range values
        if r < msg.range_min or r > msg.range_max:
            continue

        # Converting polar to cartesian coordinates, in the sensor frame
        alpha = msg.angle_min + i * msg.angle_increment
        x = r * np.cos(alpha)
        y = r * np.sin(alpha)

        # Creates a point in the sensor frame with information
        point = PointStamped()
        point.header.stamp = rp.Time()
        point.header.frame_id = msg.header.frame_id
        point.point.x = x
        point.point.y = y
        point.point.z = 0.0

        try:
            point2 = tf2_geometry_msgs.do_transform_point(point, tf_transform)
            x1, y1 = point2.point.x, point2.point.y
        except:
            rp.logwarn("Can't apply transform")
            continue

        # Convert to grid positions
        grid_x1 = int((x1 - msg_occ.info.origin.position.x) / res)
        grid_y1 = int((y1 - msg_occ.info.origin.position.y) / res)

        # Calculate points using the Bresenham's line algorithm until the endpoint
        points = bresenham(grid_x0, grid_y0, grid_x1, grid_y1)

        # Save endpoint if it is within the range
        if 0 <= grid_x1 < x_size and 0 <= grid_y1 < y_size:
            occ_x.append(grid_x1)
            occ_y.append(grid_y1)

        # Save free cells that are not the endpoint
        for x_cell, y_cell in points[:-1]:
            if 0 <= x_cell < x_size and 0 <= y_cell < y_size:
                free_x.append(x_cell)
                free_y.append(y_cell)

    # Convert python lists to NumPy arrays
    occ_x = np.array(occ_x, dtype=np.int32)
    occ_y = np.array(occ_y, dtype=np.int32)
    free_x = np.array(free_x, dtype=np.int32)
    free_y = np.array(free_y, dtype=np.int32)

    # Updates free cells
    if free_x.size > 0:
        mask_free = occmap_v[free_x, free_y] == 1
        occmap[free_x[mask_free], free_y[mask_free]] += l_free

        mask_new_free = ~mask_free
        occmap[free_x[mask_new_free], free_y[mask_new_free]] = l_prior + l_free
        occmap_v[free_x[mask_new_free], free_y[mask_new_free]] = 1

    # Updates occupied cells
    if occ_x.size > 0:
        mask_occ = occmap_v[occ_x, occ_y] == 1
        occmap[occ_x[mask_occ], occ_y[mask_occ]] += l_occ

        mask_new_occ = ~mask_occ
        occmap[occ_x[mask_new_occ], occ_y[mask_new_occ]] = l_prior + l_occ
        occmap_v[occ_x[mask_new_occ], occ_y[mask_new_occ]] = 1

# Broadcasts odom transform for every odom data message received
def odom_callback(msg: Odometry):
    m = TransformStamped()
    m.header.stamp = msg.header.stamp
    m.header.frame_id = "odom"
    m.child_frame_id = "base_link"
    m.transform.translation.x = msg.pose.pose.position.x
    m.transform.translation.y = msg.pose.pose.position.y
    m.transform.translation.z = msg.pose.pose.position.z
    m.transform.rotation = msg.pose.pose.orientation
    pub_odom_link.sendTransform(m)

def static_frame(frame_id: str, child_frame_id: str):
    m = TransformStamped()

    m.header.stamp = rp.Time.now()
    m.header.frame_id = frame_id
    m.child_frame_id = child_frame_id
    m.transform.translation.x = 0
    m.transform.translation.y = 0
    m.transform.translation.z = 0
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    m.transform.rotation.x = quaternion[0]
    m.transform.rotation.y = quaternion[1]
    m.transform.rotation.z = quaternion[2]
    m.transform.rotation.w = quaternion[3]

    return m

def bresenham(x0, y0, x1, y1):
    points = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    x, y = x0, y0

    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points

    

# Main function
if __name__ == '__main__':
    rp.init_node("OGM")
    rp.loginfo("OGM map node has started.")

    # Map Initialization
    global x_size,y_size,res,occmap
    x_size = 384
    y_size = 384
    res = 0.05 
    global l_occ,l_free,l_prior
    l_occ = ptolog(0.75)
    l_free = ptolog(0.45)
    l_prior = ptolog(0.50)
    occmap = -1*np.ones((x_size,y_size))
    occmap_v = np.zeros((x_size,y_size))

    global tf_buffer, tf_listener
    tf_buffer = tf2_ros.Buffer(rp.Duration(40))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Subscribe
    sub_laser = rp.Subscriber("/scan", LaserScan, callback=scan_callback)
    sub_odom = rp.Subscriber("/odom", Odometry, callback=odom_callback)

    # Publish
    global pub_odom, spub_link_scan
    spub_link_scan = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
    pub_odom_link = tf2_ros.TransformBroadcaster()
    pub_map = rp.Publisher("/map2", OccupancyGrid, queue_size=10)

    # Sends base_scan static transform (odom->base_link handled in odom_callback)
    m = static_frame("base_link", "base_scan")
    spub_link_scan.sendTransform(m)


    # Initialize Map msg
    q = tf.transformations.quaternion_from_euler(0,np.pi,-np.pi/2)
    msg_occ = OccupancyGrid()
    msg_occ.header.frame_id = "map"
    msg_occ.info.width = x_size
    msg_occ.info.height = y_size
    msg_occ.info.resolution = res
    msg_occ.info.origin.position.x = -((x_size*res)/2)
    msg_occ.info.origin.position.y = -((y_size*res)/2)
    msg_occ.info.origin.orientation.x = q[0]
    msg_occ.info.origin.orientation.y = q[1]
    msg_occ.info.origin.orientation.z = q[2]
    msg_occ.info.origin.orientation.w = q[3]

    # Main cycle
    while not rp.is_shutdown():

        occmap2 = occmap.flatten()

        # Vectorized conditional update
        mask_valid = occmap2 != -1
        occmap2[mask_valid] = np.where(occmap2[mask_valid] >= 0.5, 100, 0)

        # Ensure correct dtype
        occmap2 = occmap2.astype(np.int8)

        # Populate and publish
        msg_occ.data = occmap2
        msg_occ.header.stamp = rp.Time.now()
        pub_map.publish(msg_occ)

