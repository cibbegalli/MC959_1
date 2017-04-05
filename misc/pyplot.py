import matplotlib.pyplot as plt
import csv
x = []
y = []

with open("../files/gridmapbylaser.txt") as f:
    coords = csv.reader(f, delimiter='\t', quotechar='|')
    for line in coords:
        if line:
            x.append(float(line[0]))
            y.append(float(line[1]))
plt.plot(x, y, 'r.')

x = []
y = []

with open("../files/graph.csv") as f:
    coords = csv.reader(f, delimiter='\t', quotechar='|')
    for line in coords:
        if line:
            x.append(float(line[1]))
            y.append(float(line[2]))
plt.plot(x, y, 'b.')

plt.axis()
plt.show()
