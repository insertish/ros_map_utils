import rospy
from geometry_msgs.msg import Point, PointStamped

class PointListener:
    '''
    Listens for clicked points from ROS.
    '''

    sub: rospy.Subscriber

    def __init__(self):
        self.sub = rospy.Subscriber('/clicked_point', PointStamped, self._callback)

    def stop(self):
        '''
        Unregister the subscriber.
        '''
        self.sub.unregister()

    def _callback(self, msg: PointStamped):
        '''
        Callback with the PointStamped data.
        '''
        self.callback(msg.point)

    def callback(self, point: Point):
        '''
        Abstract callback with the Point data.
        '''
        pass
