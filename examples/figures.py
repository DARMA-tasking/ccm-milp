
class Illustration1:
    def __init__(self):
        self.rank_mems = [20]*2
        self.rank_working_bytes = [0, 0]
        self.task_loads = [10, 20, 10]
        self.task_working_bytes = [0, 0, 0]
        self.task_footprint_bytes = [0, 0, 0]
        self.task_indices = [[0], [1], [2]]
        self.task_rank = [0, 0, 0]
        self.task_id = [0, 1, 2]
        self.memory_blocks = [10,10]
        self.memory_block_home = [0,1]
        self.task_memory_block_mapping = [[0,1],[2]]
        self.task_communications = []

class Illustration2:
    def __init__(self):
        self.rank_mems = [20]*2
        self.rank_working_bytes = [0, 0]
        self.task_loads = [10, 20, 10]
        self.task_working_bytes = [0, 0, 0]
        self.task_footprint_bytes = [0, 0, 0]
        self.task_indices = [[0], [1], [2]]
        self.task_rank = [0, 0, 0]
        self.task_id = [0, 1, 2]
        self.memory_blocks = [10,10]
        self.memory_block_home = [0,1]
        self.task_memory_block_mapping = [[0,1],[2]]
        self.task_communications = [
            [0,2,1024*1024*1024], #c_1
            [1,2,1024*1024*1024], #c_2
            [2,1,1024*1024*1024], #c_3
            [0,1,1024*1024*1024]  #c_4
        ]
