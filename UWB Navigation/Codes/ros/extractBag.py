import rosbag 
import pickle
import numpy as np

bag = rosbag.Bag("../dataCollection/12_sept/loop1.bag")
data = [[0.,0.,0.,0.,0.,0.,0.]]
latestMocap = [0.,0.,0.]
latestUwb = [0.,0.,0.,0.]
for topic, msg, t in bag.read_messages(topics=["/uwb", "/vrpn_client_node/bot/pose"]):

    if(topic == "/vrpn_client_node/bot/pose"):
        latestMocap =  [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    if(topic == "/uwb"):
        latestUwb = msg.curr_pos

    #print(list(latestUwb)+latestMocap)
    data.append(list(latestUwb)+latestMocap)

with open("uwb.pkl",'wb') as f:
    pickle.dump(data, f)
