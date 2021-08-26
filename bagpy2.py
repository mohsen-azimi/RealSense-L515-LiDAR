from bagpy import bagreader
# import bagpy
# import pandas as pd
# import seaborn as sea
# import matplotlib.pyplot as plt


b = bagreader('bag\\bag.bag')

# print(b.__dict__)
print(b.reader)
b.message_types('/device_0/sensor_1/option/White_Balance/value')

