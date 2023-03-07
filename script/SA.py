import math
import random
import numpy as np


# 计算两点间距离
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


# 计算距离矩阵
def distance_matrix(points):
    num = len(points)
    distance_matrix = np.zeros([num, num])
    for i in range(num):
        for j in range(num):
            distance_matrix[i][j] = distance(points[i], points[j])
    return distance_matrix

# SA分配算法
class SA():
    def __init__(self, strategy='MIN_DISTANCE'):
        self.iters = 50     # 每轮温度迭代次数
        self.decay = 0.99   # 退火系数
        self.temp     = 100 # 起始温度
        self.temp_end = 10  # 终止温度
        
        # 最优策略："MIN_TIME" or "MIN_DISTANCE"
        self.strategy = strategy  

    #初始化解
    def init_solution(self, point_index, robotnum, robot_status):
        initial_solution = point_index[:]
        random.shuffle(initial_solution)
        for i in range(0, robotnum-1):
            # 插入标记，将路径分配给不同机器人
            initial_solution.insert(random.randint(0, len(initial_solution)), -1)
        return initial_solution

    # 判断解的可行性
    def verify_solution(self, robot_num, robot_status, task1num, task2num, solution):
        routes = self.split_route(robot_num, solution)
        for i in range(robot_num):
            if 0 == robot_status[i][0]:
                for pointindex in routes[i]:
                    if pointindex < task1num:
                        return False
            if 0 == robot_status[i][1]:
                for pointindex in routes[i]:
                    if pointindex >= task1num and pointindex < task1num + task2num:
                        return False
                pass
        return True

    # 切分路径
    def split_route(self, robotnum, solution):
        split_index = [i for i in range(len(solution)) if solution[i] == -1]
        split_index.insert(0, -1)
        split_index.append(len(solution))
        
        routes = []
        for i in range(0,robotnum):
            route = solution[split_index[i]+1 : split_index[i+1]]
            routes.append(route)
        return routes

    def calculate_cost(self, robot_num, robot_location, task_location, solution):
        costs = 0
        cost_list = []
        routes = self.split_route(robot_num, solution)

        # 总距离最短策略
        if "MIN_DISTANCE" == self.strategy:
            for i in range(0, robot_num):
                cost = 0
                route = routes[i]
                if len(route) > 0:
                    cost += distance(robot_location[i], task_location[route[0]])
                    for j in range(0,len(route)-1):
                        cost += distance(task_location[route[j]], task_location[route[j+1]])
                cost_list.append(cost)
            costs = sum(cost_list)
        # 总时间最短策略
        elif "MIN_TIME" == self.strategy:
            for i in range(0, robot_num):
                cost = 0
                route = routes[i]
                if len(route) > 1:
                    cost += distance(robot_location[i], task_location[route[0]])
                    for j in range(0, len(route) - 1):
                        cost += distance(task_location[route[j]], task_location[route[j+1]])
                cost_list.append(cost)
            costs = max(cost_list) + 0.1 * sum(cost_list)
        
        return costs

    #生成新解
    def new_solution(self, old):
        p = random.random()
        # 交换法
        if p < 0.33:
            while True:
                pos1, pos2 = np.random.randint(0, len(old), size=2).tolist()
                if pos1 != pos2:
                    break
            new = old[:]
            swap = old[pos1], old[pos2]
            new[pos2], new[pos1] = swap
        # 移位法
        elif p < 0.66:
            while True:
                pos = np.random.randint(0, len(old), size=3).tolist()
                if pos[0] != pos[1] and pos[1] != pos[2] and pos[0] != pos[2]:
                    break
            pos.sort()
            a=old[:pos[0]]
            b=old[pos[0] : pos[1]]
            c=old[pos[1] : pos[2]]
            d=old[pos[2]:]
            d.pop(0)
            new = a + c
            new.append(old[pos[2]])
            new = new + b + d
        # 倒置法
        else:
            while True:
                pos1,pos2 = np.random.randint(0, len(old) - 1, size=2).tolist()
                if pos2 - pos1 >=1:
                    break
            a=old[:pos1]
            b=old[pos1 : pos2+1][::-1]
            c=old[pos2+1:]
            new = a + b + c
        return new

    def solve(self, point_index, robot_location, task1_location, task2_location, robot_status):
        robot_num = len(robot_location)
        D = distance_matrix(robot_location + task1_location + task2_location)
        init_solution = self.init_solution(point_index, robot_num, robot_status)
        
        while not self.verify_solution(robot_num, robot_status, len(task1_location), len(task2_location), init_solution):
            init_solution = self.init_solution(point_index, robot_num, robot_status)
        init_cost = self.calculate_cost(robot_num, robot_location, task1_location + task2_location, init_solution)
        
        print('initial_solution = ', init_solution)
        print('initial_cost = ',init_cost)
        
        best_cost = init_cost
        best_solution = init_solution[:]
        current_solution = init_solution[:]
        current_cost = init_cost

        flag = 0
        ii = 0
        while self.temp > self.temp_end:
            for i in range(self.iters):
                new_solution = self.new_solution(current_solution)
                while self.verify_solution(robot_num, robot_status, len(task1_location), len(task2_location),
                                           new_solution) == False:
                    new_solution = self.new_solution(current_solution)
                new_cost = self.calculate_cost(robot_num, robot_location, task1_location + task2_location, new_solution)
                if new_cost < current_cost
                   or np.exp(-1 * np.abs(new_cost - current_cost) / self.temp) >= np.random.rand():
                    current_cost = new_cost
                    current_solution = new_solution[:]
                if current_cost < best_cost:
                    flag = 0
                    best_cost = current_cost
                    best_solution = current_solution[:]

            ii += 1
            flag += 1

            if flag == 5:
                flag = 0
                current_cost = best_cost
                current_solution = best_solution
            self.temp = self.temp * self.decay
        return best_solution, best_cost
