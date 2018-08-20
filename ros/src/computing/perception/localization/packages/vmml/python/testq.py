from tf import transformations as tfm

roll, pitch, yaw = 0.046039, 0.002832, 4.923170
q = tfm.quaternion_from_euler(roll, pitch, yaw)
# q = [0.019, 0.013, 0.625, 0.781]
# rpy = tfm.euler_from_quaternion(q)
print(q)