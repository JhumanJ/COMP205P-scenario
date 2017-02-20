
#Open File
f = open('robots.mat.txt','r')
for i in range (1,30):
    text = f.readline()
    print('\nGraph '+str(i)+': \n')
    print(text)
