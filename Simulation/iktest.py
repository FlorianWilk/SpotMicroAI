import ikpy
import numpy as np
from ikpy import plot_utils

my_chain = ikpy.chain.Chain.from_urdf_file("../urdf/spotmicroai_gen.urdf.xml",base_elements=["front_left_shoulder_link"])
print(my_chain)
target_vector = [ 0.1, -0.2, 0.1]
target_frame = np.eye(4)
target_frame[:3, 3] = target_vector

print("The angles of each joints are : ", my_chain.inverse_kinematics(target_frame))

real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_frame))
print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))


# If there is a matplotlib error, uncomment the next line, and comment the line below it.
# %matplotlib inline
import matplotlib.pyplot as plt
ax = plot_utils.init_3d_figure()
my_chain.plot(my_chain.inverse_kinematics(target_frame), ax, target=target_vector)
plt.xlim(-0.1, 0.1)
plt.ylim(-0.1, 0.1)