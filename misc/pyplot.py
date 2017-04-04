import matplotlib.pyplot as plt
import csv
x = []
y = []

with open("gridmapbylaser.txt") as f:
#with open("teste.txt") as f:
    coords = csv.reader(f, delimiter='\t', quotechar='|')
    for line in coords:
        x.append(float(line[0]))
        y.append(float(line[1]))

plt.plot(x, y, 'r.')
plt.axis()
plt.show()
