from json import dumps, loads
from pathlib import Path

class GeoFile:
    path: Path

    def __init__(self, filename: str):
        self.path = Path(filename)

        if self.path.is_file():
            self.read()
        else:
            self.create()
    
    def create(self):
        self.data = {
            "type": "FeatureCollection",
            "features": []
        }

        self.write()

    def write(self):
        with open(self.path, 'w') as f:
            f.write(dumps(self.data))
    
    def read(self):
        # TODO: validate data
        with open(self.path, 'r') as f:
            self.data = loads(f.read())
    
    '''
    Temporary Geo API
    Could do this OOP instead.
    '''

    def add_feature(self, feature):
        self.data["features"].append(feature)
        self.write()
