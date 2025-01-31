## :earth_africa: Map generator

This python script allows you to generate map and place on it a path according to GPS points with a color of intensity depending 
on the amplitude of vibrations (red - big vibrations, blue- small vibrations).
To use it, you need to have a .csv file with the following data :
- longitue and latitude read from GPS module
 - axis Y and axis Z read from accelerometer module.

## Installation

This program was written using Python ver. 3.8 and the Folium and branca libraries.
Before starting it, make sure that you have the folium and branca libraries installed.

**Installation of Folium library**
```bash
pip install folium
```
**Installation of Branca library**
```bash
pip install branca
```

## Usage
To run this script, type the following on the command line:

```bash
python map_generator.py -i datafile.csv -o generated_map.html
```
After execution, an html file will be generated containing a map with a marked path.

An example of a generated map:

<img width="645" alt="mapa_2" src="https://user-images.githubusercontent.com/33400631/94155160-ecc6b000-fe7e-11ea-9283-432b5b6618d3.PNG">

