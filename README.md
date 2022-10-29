# map_utils

ROS package for manipulating GeoJSON data in worldspace.

## Usage

Clone this project into your ROS workspace:

```bash
cd ros_ws
git clone https://github.com/insertish/map_utils src/map_utils
```

Run the utility by building then using rosrun:

```bash
catkin build # or catkin_make
rosrun map_utils editor <filename.geojson>
```

## Known Issues / Limitations

- Only supports creating polygons.
- Currently this is hardcoded to use `map` as a frame of reference.
- Placing the third point on a polygon doesn't draw a line for some unknown reason, might be an issue with Foxglove.

## Demo

https://user-images.githubusercontent.com/38285861/198846753-27af9ee0-9ceb-4253-a436-e9dc01a797e2.mp4
