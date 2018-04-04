import yaml
import matplotlib.pyplot as plt

stream = open('rmcore/map/map.yaml', 'r')
map = yaml.load(stream)
print(map)
lastP = None

colors = ['red', 'green', 'blue', 'black']

for segment in map['segments']:
    xdata = []
    ydata = []
    if(lastP is not None):
        xdata.append(lastP['x'])  # pylint: disable=E1136
        ydata.append(lastP['y'])  # pylint: disable=E1136

    for p in segment['viaPoints']:
        xdata.append(p['x'])
        ydata.append(p['y'])
        lastP = p
    plt.plot(xdata, ydata, color=colors[segment['id'] % len(colors)])


plt.axis([-5, 5, -6, 6])
plt.show()
