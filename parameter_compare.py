import numpy as np
import matplotlib.pyplot as plt
import os

file_list = os.listdir(os.getcwd() + '/motion_data')

N = len(file_list)
ax = plt.subplot(111)
for i in range(N):
    temp = np.load(os.getcwd() + '/motion_data/' + file_list[i])
    plt.plot(temp[:, 0], temp[:, -3], label=file_list[i])

leg = plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)
leg.get_frame().set_alpha(0.5)

plt.show()