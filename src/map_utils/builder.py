from .listener import PointListener
from .visualisation import Visualiser

from geometry_msgs.msg import Point

class PolygonBuilder(PointListener):
    '''
    Build a Polygon feature from points.
    '''

    name: str
    points: [Point] = []
    visualiser: Visualiser

    def __init__(self, name):
        super().__init__()
        self.name = name
        self.visualiser = Visualiser()

    def callback(self, point: Point):
        self.points.append(point)
        self.visualiser.render_polygon(self.points)

    def stop(self):
        super().stop()
        self.visualiser.stop()

        coordinates = []
        for point in self.points:
            coordinates.append((point.x, point.y, point.z))

        # TODO: OOP
        return {
            "type": "Feature",
            "geometry": {
                "type": "Polygon",
                "coordinates": coordinates
            },
            "properties": {
                "name": self.name
            }
        }
        
