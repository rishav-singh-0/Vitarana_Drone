import csv
import numpy
import os

s=[]
with open(os.path.join(os.path.dirname(os.path.realpath(__file__)),'manifest.csv'),'r') as x:
    content = csv.reader(x)
    c2 = numpy.array(list(csv.reader(x)))
    data2 = c2[:,1:]
    print(c2[:,0])
    # print(data2)
    for i in data2:
        for j in i:
            print(float(j))
    # x=0
#     for i in content:
#         # print(i)
#         s.append(i)
#         s[x].pop(0)
#         x+=1
# q=len(s)
    
# for i in range(len(s)):
#     print(i)
#     for j in range(len(s)):
#         # print(j)
#         s[i][j]=float(s[i][j])
# print(s[0])
# print(s[1])
# print(s[2])


