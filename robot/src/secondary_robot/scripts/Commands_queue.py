import numpy as np
class Commands_queue:

    def __init__(self, size = 10, d_type = str):
        self.size = size
        self.d_type = d_type
        self.data = np.array([])


    def elements_count(self):
        return len(self.data)


    def get(self):
        if self.elements_count() > 0:
            data = self.data[-1]
            self.data = np.delete(self.data, -1)
            return data
        else:
            return False
        
    def push(self, data):
        if isinstance(data, self.d_type):
            if self.elements_count() < self.size:
                self.data = np.append(data, self.data)
                return self.data
            else:
                print("queue saturated!")
                return False
        else:
            print("type mismatch")
            return False
        
    def __str__(self):
        return "\n".join([d.action+" " + d.subject+" " + str(d.value)+" " + d.unit for d in self.data])