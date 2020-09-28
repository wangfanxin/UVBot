import numpy as np
import matplotlib.pyplot as plt
import time
import random

def pl_filter(pl1, pl2, L=100):
    start = time.process_time()
    L1 = pl1.shape[0]
    L2 = pl2.shape[0]
    if min(L1, L2) < L:
        L = min(L1, L2)
    print(L1, L2)
    pl1 = remove_under_prob(pl1, L1 - L)
    pl2 = remove_under_prob(pl2, L2 - L)
    print(time.process_time() - start)
    return pl1, pl2


def remove_under_prob(pl, R):
    if R > 0:
        L = pl.shape[0]
        prob = 1./ pl[:,2]
        prob /= sum(prob)
        r = np.zeros((R,),dtype=int)
        i = 0
        itr = 0
        while i < R:
            if prob[itr] > random.random():
                r[i] = itr
                i += 1
                prob[itr] = 0
            itr += 1
            if itr == L:
                itr -= L

        pl = np.delete(pl, r, axis=0)
    return pl



scan0 = np.loadtxt('scan0.csv', delimiter=',')
scan1 = np.loadtxt('scan1.csv', delimiter=',')

A0 = np.array([[a[2]*np.cos(a[1]/180.*np.pi), a[2]*np.sin(a[1]/180.*np.pi)] for a in scan0])
A1 = np.array([[a[2]*np.cos(a[1]/180.*np.pi), a[2]*np.sin(a[1]/180.*np.pi)] for a in scan1])

scan0_, scan1_ = pl_filter(scan0, scan1, L=50)

A0_ = np.array([[a[2]*np.cos(a[1]/180.*np.pi), a[2]*np.sin(a[1]/180.*np.pi)] for a in scan0_])
A1_ = np.array([[a[2]*np.cos(a[1]/180.*np.pi), a[2]*np.sin(a[1]/180.*np.pi)] for a in scan1_])



plt.subplot(121)
for a in A0:
    plt.plot(a[0],a[1],'r.')
for a in A1:
    plt.plot(a[0],a[1],'b.')
plt.axis('scaled')

plt.subplot(122)
for a in A0_:
    plt.plot(a[0],a[1],'r.')
for a in A1_:
    plt.plot(a[0],a[1],'b.')
plt.axis('scaled')

plt.show()
