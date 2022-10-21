def getDistance(RSSI):
  N = 2.4
  MP = -44
  return 10**((MP-RSSI)/(10*N))
  