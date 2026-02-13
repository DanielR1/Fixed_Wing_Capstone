import numpy as np

#Computes Aero Forces and Moments from current state and control deflections using database lookup
def getAeroForcesMoments(state, ctrl_in):
    
