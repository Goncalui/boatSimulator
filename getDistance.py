import matplotlib.pyplot as plt

N = 2.4
MP = -44

RSSI = [MP-1.25*i for i in range(0,20)]
print(RSSI)

b = []
for i in RSSI:
  b.append(10**((MP-i)/(10*N)))
  
print(b)

#def measureDistance(RSSI)

plt.plot (b,RSSI)
plt.show()
