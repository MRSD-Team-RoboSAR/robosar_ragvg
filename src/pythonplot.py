import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import cv2
img = cv2.imread('/home/naren/catkin_ws/src/robosar_ragvg/maps/scott_final.png')
a = np.array([[500,281],[490,246],[506,214],[517,167],[226,147],[227,111],[498,104],[191,77],[156,78],[121,83],[262,78],[297,79],[332,76],[367,78],[402,78],[437,77],[478,82],[37,75],[72,76],[624,75],[589,72],[554,73]])
a = np.array([[441,257],[541,244],[576,243],[611,243],[406,239],[371,238],[339,238],[304,238],[269,237],[234,232],[482,230],[495,192],[227,190],[519,161],[516,126],[491,67],[500,35],[496,2]])
fig, ax = plt.subplots(1)

ax.imshow(img)
ax.axis('off')
for i in range(a.shape[0]):
    circ = Circle((a[i,0],a[i,1]),5)
    ax.add_patch(circ)
plt.show()