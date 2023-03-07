import SA
import random
import matplotlib.pyplot as plt


task1_location=[[3,3],[1,5],[7,9]]
task2_location=[[0,1],[0,5],[7,8]]
robot_num=2
robot_location=[[7,6],[0,0]]# 机器人初始坐标
robot_status=[[1,1],[1,1]]# 某处为1表示列表示第i个机器人可以去执行第j种任务

def show(routes,robot_location):
    task_lacation = task1_location+task2_location
    x1,y1,x2,y2=[],[],[],[]
    for i in range(len(task1_location)):
        x1.append(task1_location[i][0])
        y1.append(task1_location[i][1])
    for i in range(len(task2_location)):
        x2.append(task2_location[i][0])
        y2.append(task2_location[i][1])
    plt.scatter(x1,y1,c='red',s=50,marker='v', label='taskA')
    plt.scatter(x2,y2,c='blue',s=40,marker='o', label='taskB')
    plt.legend()
    plt.axis([min(x1+x2)-3, max(x1+x2)+3, min(y1+y2)-3,max(y1+y2)+3])

    for i in range(len(routes)):
        plt.scatter(robot_location[i][0],robot_location[i][1],s=30,marker='*',label='robot'+str(i+1))
        route_x =[task_lacation[point][0] for point in routes[i]]
        route_y = [task_lacation[point][1] for point in routes[i]]
        route_x.insert(0,robot_location[i][0])
        route_y.insert(0, robot_location[i][1])
        plt.plot(route_x,route_y)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    points=robot_location+task1_location+task2_location
    sa=SA.SA('MIN_TIME')
    best_solution,best_cost=sa.solve([0,1,2,3,4,5],robot_location,task1_location,task2_location,robot_status)
    routes=sa.split_route(robot_num,best_solution)
    print('routes=',routes)
    print('best_solution=',best_solution)
    print('best_cost=',best_cost)
    # show(routes,robot_location)
