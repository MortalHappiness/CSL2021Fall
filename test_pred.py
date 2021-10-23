import numpy as np
from collections import Counter
from process_raw import process_raw

# some initialization
k = 5
label = np.empty(255, dtype=int)
for i in range(1, 6) :
    label[(i - 1) * 51:i * 51] = int(i)

# Read processed results
with open("trainfile", 'r') as f :
    train = np.array([[x for x in line.split()] for line in f]).astype(float)

# Read input to classify
llist = [] # test : 1=5, 2=78, 3=145, 4=199, 5=238
with open("pts_238", 'r') as f :
    plist = [[int(x) for x in line.split()] for line in f]
    x = np.array(process_raw(plist)).astype(float)

# k nearest neighbors
# euclidean distance
distances = np.linalg.norm(train - x, axis=1)
# nearest neighbot indices
nearest_neighbors = distances.argsort()[:k]
# majority vote
pred, _ = Counter(label[nearest_neighbors]).most_common()[0]
print(pred)