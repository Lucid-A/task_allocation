import random
import numpy as np
import matplotlib.pyplot as plt
import SA
import Astar_Cost as C

# 仿真时随机生成任务点
astar = SA.prepare_astar()
available_location = np.argwhere(astar.grid==0)
np.random.seed(19)
taskA_location = available_location[np.random.choice(available_location.shape[0],3,replace=False),:].tolist()
taskB_location = available_location[np.random.choice(available_location.shape[0],3,replace=False),:].tolist()
taskA_location = C._xy_transation(taskA_location)
taskB_location = C._xy_transation(taskB_location)
print(taskA_location)
print(taskB_location)

# args to be read from config file
robot_num = 2
robot_location = [[0,0], [0,0]] # The robot pose in map frame when calling task allocation
robot_status   = [[1,1], [1,1]] # robot_status[i][j] is 1 means robot[i+1] can carry out task[j+1]
D_A = SA.distance_astar_matrix(taskA_location + taskB_location)

def show(routes, robot_location):
    task_lacation = taskA_location + taskB_location
    x1, y1, x2, y2 = [], [], [], []
    for i in range(len(taskA_location)):
        x1.append(taskA_location[i][0])
        y1.append(taskA_location[i][1])
    for i in range(len(taskB_location)):
        x2.append(taskB_location[i][0])
        y2.append(taskB_location[i][1])
    plt.scatter(x1,y1, c='red',  s=50, marker='v', label='taskA')
    plt.scatter(x2,y2, c='blue', s=40, marker='o', label='taskB')
    plt.legend()
    plt.axis([min(x1+x2)-3, max(x1+x2)+3, min(y1+y2)-3, max(y1+y2)+3])

    for i in range(len(routes)):
        plt.scatter(robot_location[i][0],robot_location[i][1], s=30, marker='*', label='robot'+str(i+1))
        route_x = [task_lacation[point][0] for point in routes[i]]
        route_y = [task_lacation[point][1] for point in routes[i]]
        route_x.insert(0, robot_location[i][0])
        route_y.insert(0, robot_location[i][1])
        plt.plot(route_x, route_y)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    sa=SA.SA('MIN_TIME')
    best_solution,best_cost = sa.solve([0,1,2,3,4,5], robot_location, taskA_location, taskB_location, robot_status)
    routes = sa.split_route(robot_num, best_solution)
    
    # for Cmd
    print(f"routes = {routes}")
    print(f"best_solution = {best_solution}")
    print(f"best_cost = {best_cost}")
    ## for Graphic
    # show(routes, robot_location)
