import csv
s=[]
with open('manifest.csv','r') as x:
    content = csv.reader(x)
    # print(content)
    x=0
    for i in content:
        # print(i)
        s.append(i)
        s[x].pop(0)
        x+=1
q=len(s)
    
for i in range(len(s)):
    print(i)
    for j in range(len(s)):
        # print(j)
        s[i][j]=float(s[i][j])
print(s[0])
print(s[1])
print(s[2])


