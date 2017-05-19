import csv
i = 0
with open("../files/gt.txt") as f:
    coords = csv.reader(f, delimiter='\t', quotechar='|')
    for line in coords:
        if line:
                while i < len(line) - len(line)%4:
                    print(line[i], line[i+1],line[i+2], line[i+3])
                    i += 4
