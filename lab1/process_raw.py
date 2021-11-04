import cv2
import numpy as np
from os import listdir
import random
from collections import Counter

def process_raw(plist) :
    # select numsel (now 50) points, delete some points or add some
    numsel = 50
    numpoints = len(plist)
    if numpoints > numsel : 
        dellist = random.sample(range(numpoints), (numpoints - numsel))
        for todel in sorted(dellist, reverse=True) :
            del plist[todel]
    elif numpoints < numsel :
        addlist = random.choices(range(1, numpoints), k=(numsel - numpoints))
        for toadd in sorted(addlist, reverse=True) : 
                addpoint = [(plist[toadd][0] + plist[toadd - 1][0])/2, (plist[toadd][1] + plist[toadd - 1][1])/2]                    
                plist.insert(toadd, addpoint)
    # print(len(plist))
        
    # Align and rescale
    # find mininum enclosing rectangle
    min_x, min_y = 680, 680
    max_x, max_y = 0, 0
    for point in plist :
        if point[0] < min_x : 
            min_x = point[0]
        if point[0] > max_x :
            max_x = point[0]
        if point[1] < min_y : 
            min_y = point[1]
        if point[1] > max_y :
            max_y = point[1]

    # center = (340, 240), size = 340*480
    # therefore, x range (170, 510), y range (0, 480)
    center_x = (min_x + max_x)/2
    center_y = (min_y + max_y)/2
    width = max_x - min_x
    height = max_y - min_y
    for point in plist : 
        point[0] = 340 + (point[0] - center_x) * (340 / width)
        point[1] = 240 + (point[1] - center_y) * (480 / height)

    # flatten and return
    flat_plist = [point for sublist in plist for point in sublist]
    return flat_plist

# Process raw training files
trainlist = []
for i in range(10) : # should be [0, 10)
    pdir = f"./points/{i}"
    for fname in listdir(pdir) :
        # print(fname)
        fpath = f"./points/{i}/{fname}"
        with open(fpath) as f :
            plist = [[int(x) for x in line.split()] for line in f] 
            trainlist.append(process_raw(plist))
# write results to a file for future use
with open("trainfile" , 'w') as f :
    for flatlist in trainlist :
        for coord in flatlist:
            f.write(f"{coord} ")
        f.write("\n")