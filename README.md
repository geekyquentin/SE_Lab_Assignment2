# SE_Lab_Assignment2

[Openstreetmap](www.openstreetmap.org) is a free and open resource which provides high  quality map data for all areas in the world. OSM is an xml-based format, which has various elements and the attributes for those elements.

This software reads data about various artifacts on the map, and provide a text searchable interface to the map.

An [OSM file of the Kharagpur area](https://github.com/geekyquentin/SE_Lab_Assignment2/blob/master/map.osm) is given as reference. Two particular elements of the OSM format, are of interest:

1. “node”: This represents a particular place on the map, e.g. a shop, building, etc. It has an  id, latitude, and longitude. Additionally, some node elements have a “name” attribute which describes the place.

2. “way”: This represents a path or a line through the map, and is denoted by a sequence of node elements. This element also has an id, and optionally a name.

### Use case #1:
Parse a given OSM file and extract the node and way elements from the file along with their attributes. It prints the total number nodes and ways discovered in the given file. It allow a user to search for a particular element by matching input string with substring of the name.

### Use case #2:
Finding the k-closest nodes to a given node using the crow fly distance. To calculate distance, longitude and latitude information is used. Here k is a user input.

### Use case #3:
Calculate the shortest path between two node elements, through the way elements. The distance on a way is the sum of distances between consecutive node elements in the way.

## Instructions:
Run the [`20CS30036_Assign2`](https://github.com/geekyquentin/SE_Lab_Assignment2/blob/master/20CS30036_Assign2.cpp) file.
