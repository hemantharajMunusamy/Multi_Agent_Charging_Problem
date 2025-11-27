import enum
from queue import Queue
import threading

from data_model.data import *
import matplotlib.pyplot as plt


class BlackBoard:
    """
    BlackBoard - Centralized way of accessing data Key format is (Global_id, Key_ID)
    """
    def __init__(self):
        #Related to data
        self.data = {}
        self.topic_list = {}
        #1 self.data_graph = nx.Graph(n = 0, directed = True)
        self.data_lock = threading.Lock()
 
    def register_node(self, global_id, name):
        """Register node
        Args:
            global_id (_type_): _description_
            name (_type_): _description_
        """
        #register node in graph
        #1 self.data_graph.add_node(global_id, name=name)
        pass

    def register(self, global_id, access, data_type):
        """
        Create Data repository [0 - read, 1 - write]
        Args:
            global_id (_type_): _description_
            access (_type_): _description_
            data_type (_type_): _description_
        """
        if access == 0:
            pass
            #topic_node = self.topic_list[(global_id, data_type)]
            #self.data_graph.add_edges([(topic_node.index, node_id)])
            #self.data[(global_id, data_type)] = None
        elif access == 1:
            node_name = f"{data_type.name}"
            #1 topic_node = self.data_graph.add_node(data_type.value*1000,name=node_name)
            #1 self.topic_list[data_type.value * 1000] = topic_node
            #1 self.data_graph.add_edge(global_id, data_type.value*1000)
            self.data[(global_id, data_type)] = None
    
    def read(self, key):
        """
        Read Data Value
        
        :param self: Description
        :param key: Description
        """
        return self.data[key]

    def read_all(self, data_type):
        """
        Read all the Data with ending datatype

        :param self: Description
        :param data_type: Description
        """
        datas = []
        for key in self.data.keys():
            #key match with any data_type
            if key[1] in data_type:
                datas.append(self.read(key))
        return datas

    def merge_all(self, data_type_list):
        """
        Read combinely all the Data with list of matched datatype        
        :param self: Description
        :param data_type_list: Description
        """
        self.data_lock.acquire()
        datas = []

        idx = 0
        if data_type_list[0] == Data.GID:
            idx += 1
        for key in self.data.keys():
            #key match with any data_type
            if key[1] == data_type_list[idx]:
                data = []
                for data_type in data_type_list:
                    data.append(self.read((key[0],data_type)))
                datas.append(data)
        self.data_lock.release()
        return datas

    def write(self, key, data):
        """
        Write Data into BlackBoard
        Args:
            key (_type_): _description_
            data (_type_): _description_
        """
        self.data_lock.acquire()
        self.data[key] = data
        self.data_lock.release()

    def plot_graph(self):
        #self.data_graph.vs["label"] = self.data_graph.vs["name"]
        #1 nx.draw(self.data_graph, with_labels=True, node_size=10)
        plt.show()
