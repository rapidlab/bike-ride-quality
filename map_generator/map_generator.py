import folium
import pandas as pd
import folium.plugins
import math
import branca
import branca.colormap as cm
import sys, getopt

if __name__ == "__main__":
    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print ("map_generator.py -i <inputfile> -o <outputfile>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ("map_generator.py -i <inputfile> -o <outputfile>")
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    print('Input file is ', inputfile)
    print('Output file is ', outputfile)

    #reading data from a .csv file
    column_names=['Latitude', 'North/South','Longitude', 'East/West', 'Axis X', 'Axis Y', 'Axis Z']
    df = pd.read_csv(inputfile, sep=';', names=column_names, header=None)

    #type conversion to float
    df['Latitude'].astype(float)
    df['Longitude'].astype(float)
    df['Latitude'] = df['Latitude']/100
    df['Longitude'] = df['Longitude']/100

    for index, row in  df.iterrows():
        latitude1 = math.modf(row['Latitude'])
        longitude1 = math.modf(row['Longitude'])

        #conversion of geographic coordinates to decimal format
        latitude1= latitude1[0]*100/60 + latitude1[1]   
        longitude1 = longitude1[0]*100/60 + longitude1[1]

        df.at[index,'Latitude'] = latitude1
        df.at[index, 'Longitude'] = longitude1
        #conversion of raw data to mG
        osY = row['Axis Y'] *0.488 
        osZ = row['Axis Z'] *0.488
        resultant = math.sqrt(osY*osY + osZ*osZ)
        df.at[index, 'Axis Z'] = resultant

    for index, row in  df.iterrows():
        if index == 0:
            currentLatitude = row['Latitude']
            currentLongitude = row['Longitude']
            count = 1
        else:
            if currentLatitude == row['Latitude'] and currentLongitude == row['Longitude']:
                count +=1
            else:
                differenceLatitude = row['Latitude'] - currentLatitude
                differenceLongitude = row['Longitude'] - currentLongitude

                for i in range(count-1, 0, -1):
                    df.at[index - i, 'Latitude'] += (count - i)*differenceLatitude/count
                    df.at[index - i, 'Longitude'] += (count - i)*differenceLongitude/count

                currentLatitude = row['Latitude']
                currentLongitude = row['Longitude']
                count = 1


    #sorting and saving on the list the maximum acceleration value for the given gps coordinate
    #sorting = df.sort_values('Axis Z', ascending=False).drop_duplicates(['Latitude','Longitude'])
    sorting = df.sort_values('Axis Z', ascending=True)

    #setting the map start coordinates
    x_start = (sorting['Latitude'].max() + sorting['Latitude'].min()) / 2
    y_start = (sorting['Longitude'].max() + sorting['Longitude'].min()) / 2
    start_coord = (x_start, y_start)

    #minimum acceleration value
    minValue = sorting['Axis Z'].min()
    #maximum acceleration value
    maxValue = sorting['Axis Z'].max()

    #setting the colors and scale of acceleration values for points displayed on the map (colorbar)
    colormap = cm.LinearColormap(colors=['blue','green', 'yellow', 'red'], index=[minValue,(minValue+maxValue)/3, 2*(minValue+maxValue)/3, maxValue],vmin=sorting['Axis Z'].min(),vmax=sorting['Axis Z'].max())

    #creating a map with initial zoom level set to 50
    map = folium.Map(location=start_coord, zoom_start=50)

    #lists with latitude, longitude and acceleration values
    lat = list(sorting['Latitude'])
    lon = list(sorting['Longitude'])
    pow = list(sorting['Axis Z'])

    #plotting data in the form of points on the map
    for loc, p in zip(zip(lat, lon), pow):
        folium.Circle(
            location=loc,
            radius=0.10,
            fill=True,
            color=colormap(p),
            fill_opacity=1.0
        ).add_to(map)

    #adding a color legend to a map
    map.add_child(colormap)

    #saving the map as an .html file
    map.save(outputfile)