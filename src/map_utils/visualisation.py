from std_msgs.msg import ColorRGBA
from rospy import Publisher, get_rostime
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point, Vector3, Pose, Quaternion

# lift all points up in the air since
# we are working in 2D space and we
# don't want our stuff to clip
render_offsets = (0, 0, 0.08)

# object scale
sphere_scale = Vector3(0.08, 0.08, 0.08)
line_scale = Vector3(0.05, 0.0, 0.0)

# object colours
sphere_colour = ColorRGBA(0.0, 0.1, 0.8, 1.0)
line_colour = ColorRGBA(0.8, 0.2, 0.2, 1.0)
prospective_line_colour = ColorRGBA(0.8, 0.2, 0.2, 0.5)

# identity pose
identity_pose = Pose(
    Point(0, 0, 0),
    Quaternion(0, 0, 0, 1)
)

class Visualiser:
    '''
    Visualisation helper
    '''

    pub: Publisher

    def __init__(self):
        self.pub = Publisher("/map_utils/marker", Marker, queue_size=10)

    def stop(self):
        self.pub.unregister()

    def render_polygon(self, points: [Point], id_offset = 0):
        '''
        Render a polygon.
        '''

        # offset all points
        offset_points = []
        for point in points:
            offset_points.append(Point(
                point.x + render_offsets[0],
                point.y + render_offsets[1],
                point.z + render_offsets[2],
            ))

        # configure common options
        msg = Marker()
        msg.header.frame_id = "map"
        msg.ns = "map_utils"
        msg.action = Marker.MODIFY
        msg.pose = identity_pose
        
        # draw spheres
        msg.header.stamp = get_rostime()
        msg.id = id_offset
        msg.type = Marker.SPHERE_LIST
        msg.scale = sphere_scale
        msg.color = sphere_colour
        msg.points = offset_points

        self.pub.publish(msg)

        # draw lines
        msg.header.stamp = get_rostime()
        msg.type = Marker.LINE_STRIP
        msg.id = id_offset + 1

        if len(points) > 1:
            msg.scale = line_scale
            msg.color = line_colour
            self.pub.publish(msg)

            # draw last connecting line
            msg.header.stamp = get_rostime()
            msg.id = id_offset + 2

            if len(points) > 2:
                msg.color = prospective_line_colour
                msg.points = [
                    offset_points[len(offset_points) - 1],
                    offset_points[0]
                ]
            else:
                msg.action = Marker.DELETE
            
            self.pub.publish(msg)
        else:
            # delete if no lines
            msg.action = Marker.DELETE
            msg.points = []
            self.pub.publish(msg)

            # also delete last connecting line
            msg.id = id_offset + 2
            self.pub.publish(msg)