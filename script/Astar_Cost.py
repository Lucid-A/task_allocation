import os
import time
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# np.set_printoptions(threshold=np.inf)
class Progress_Pgm(object):
    def __init__(self, path):
        self.path = path
        # img = Image.open(path)    # 读取文件
        # img.show()    # 展示图片

    def is_pgm_file(self, path):
        if not os.path.isfile(path):
            return False
        if path is not str and not path.endswith('.pgm'):
            return False
        return True

    def convert_pgm(self):
        """
        将pgm文件转换成np_arrray
        :param in_path: 输入pgm文件路径
        """
        if not self.is_pgm_file(self.path):
            raise Exception(f"\"{self.path}\"不是一个PGM文件")
        with open(self.path, 'rb') as f:
            # 读取两个字节 - 幻数，并解码成字符串
            magic_number = f.readline().strip().decode('utf-8')
            # 读取高和宽
            width, height = f.readline().strip().decode('utf-8').split(' ')
            self.width = int(width)
            self.height = int(height)
            # 读取最大值
            maxval = f.readline().strip()
            # 每次读取灰度值的字节数
            if int(maxval) < 256:
                one_reading = 1
            else:
                one_reading = 2
            # 创建空白图像，大小为(行，列)=(height, width)
            self.map = np.zeros((self.height, self.width))
            self.map[:, :] = [[ord(f.read(one_reading)) for j in range(self.width)] for i in range(self.height)]

    def map_process(self, obsize=7):
        """
       膨胀地图障碍物的边界
       :param obsize: 车体宽度
       """
        # 先二值化处理
        self.map[self.map <  127] = 0
        self.map[self.map >= 127] = 1

        indexList = np.where(self.map == 0)  # 将地图矩阵中1的位置找到
        # 遍历地图矩阵
        for x in range(self.map.shape[0]):
            for y in range(self.map.shape[1]):
                if x < 10 or x > self.height-10 or
                   y < 10 or y > self.width-10: # 边缘置0
                    self.map[x][y] = 0
                if self.map[x][y] == 1:
                    for ox, oy in zip(indexList[0], indexList[1]):
                        # 如果和有0的位置的距离小于等于膨胀系数，那就设为0
                        distance = ((x - ox) ** 2 + (y - oy) ** 2)
                        if distance <= obsize*obsize:
                            self.map[x][y] = 0
        np.save('final_map', self.map)

    def print_map(self):
        np.set_printoptions(threshold=np.inf)
        print(self.map)
    
    def show_final_map(self):
        img = Image.fromarray(self.map*255)
        img.show()

# 实际地图与仿真地图转换 实际坐标->图像坐标
def xy_transation(realxy_list):
    # 实际地图的(0,0)在图像的(94,112)处，实际地图x,y方向:↑←；图像x,y方向:→↓。1个像素=0.05
    mapxy_list = []
    for real_xy in realxy_list:
        mapxy_list.append([94-real_xy[0]*20, 112-real_xy[1]*20])
    return mapxy_list
# 实际地图与仿真地图转换：图像坐标->实际坐标
def _xy_transation(mapxy_list):
    # 实际地图的(0,0)在图像的(94,112)处，实际地图x,y方向:↑←；图像x,y方向:→↓。1个像素=0.05
    realxy_list = []
    for map_xy in mapxy_list:
        realxy_list.append([(94 - map_xy[0])/20, (112 - map_xy[1])/20])
    return realxy_list

class Astar(object):
    def __init__(self, grid):
        self.grid = grid
        # print('Imput Grid:\n', self.grid)
        self.found = False  # 到达标记
        self.resign = False  # 启动标记
        self.cost = 1  # 每移动一步花费的cost
        self.begin_point = np.array([0, 0], dtype=np.int32)
        self.target_point = np.array([0, 0], dtype=np.int32)
        self.W = len(grid[0])  # grid宽度
        self.H = len(grid)  # grid高度
        self.shape = (self.H, self.W)
        #self.delta = [[-1, 0],  # 上
        #             [0, -1],  # 左
        #             [1, 0],  # 下
        #             [0, 1],   # 右
        #             [-1,-1],  # 左上
        #             [1,-1],   # 左下
        #             [1,1],    # 右下
        #             [-1,1]]  # 右上
        self.delta = [
            [-1, -1], [-1, 0], [-1, 1],
            [ 0, -1],          [ 0, 1],
            [ 1, -1], [ 1, 0], [ 1, 1]]
        self.heuristic     = np.zeros(self.shape, dtype=np.int32)  # 生成与grid同大小的0矩阵
        self.close_matrix  = np.zeros(self.shape, dtype=np.int32)  # 生成与grid同大小的0矩阵,用来保存周围坐标
        self.action_matrix = np.zeros(self.shape, dtype=np.int32)  # 生成与grid同大小的0矩阵，用来保存已经计算过的点

    def set_start(self, startx, starty):  # 输入起始点X，Y坐标
        self.begin_point[0] = startx
        self.begin_point[1] = starty

    def set_target(self, targetx, targety):  # 设置终点函数
        self.target_point[0] = targetx
        self.target_point[1] = targety

    def x_y_inrange(self, xin, yin):  # 检查x、yin是否超出地图边界
        if xin >= 0 and xin < self.H and yin >= 0 and yin < self.W:
            return True
        else:
            return False

    def edge_punish(self, x, y):
        # 贴边惩罚
        delta = self.delta
        punish = 0
        for i in range(len(delta)):
            punish += self.grid[x + delta[i][0], y + delta[i][1]]
        return punish

    def calculate(self):
        # 起始点与终点不能在障碍物的地方
        # print(self.begin_point[0],self.begin_point[1],self.target_point[0],self.target_point[1])
        # print(self.grid[self.begin_point[0]][self.begin_point[1]],self.grid[self.target_point[0]][self.target_point[1]])
        assert (self.grid[self.begin_point[0]][self.begin_point[1]] != 1) and (
                    self.grid[self.target_point[0]][self.target_point[1]] != 1)
        # 检查起点和终点是不是在障碍物区内
        self.start = time.time()
        target_point = self.target_point
        delta = self.delta
        grid_copy = self.grid.copy()  # 生成grid的副本用于添加路径信息以及可视化内容
        # 计算H-cost
        for i in range(self.H):  # 逐个计算与终点的距离值
            for j in range(self.W):
                self.heuristic[i][j] = abs(i - target_point[0]) + abs(j - target_point[1])
                if self.grid[i][j] == 1:  # 如果遇到障碍，默认给一个巨大的值，让他不可取
                    self.heuristic[i][j] = 9999
        #print('H-coat Grid:\n', self.heuristic)
        x = self.begin_point[0]
        y = self.begin_point[1]
        g = 0
        f = g + self.heuristic[self.begin_point[0]][self.begin_point[1]]
        cell = np.array([[f, g, x, y]])
        # print(cell)

        found = False  # 到达标记
        resign = False  # 启动标记
        steps = 1
        while not found and not resign:
            if len(cell) == 0:
                resign = True
                print('寻路失败~\n请检查路径是否封闭')
                exit()
            else:
                # 将f最小的点排到最前
                cell = cell[np.argsort(cell[:, 0])]
                next_point = cell[0]
                cell = cell[1:]
                # print(cell)
                # print(next_point)
                f = next_point[0]
                g = next_point[1]
                x = int(next_point[2])
                y = int(next_point[3])

                if x == target_point[0] and y == target_point[1]:
                    found = True
                    self.realcost = cell[0][0]
                    # print(cell)
                else:
                    for i in range(len(delta)):  # 计算周围四个坐标参数
                        x_new = x + delta[i][0]  # x坐标跟新
                        y_new = y + delta[i][1]  # y坐标跟新
                        # 判断新坐标合法性
                        if self.x_y_inrange(x_new, y_new):
                            if self.close_matrix[x_new][y_new] == 0 and self.grid[x_new][y_new] == 0:
                                if(i < 4):
                                    g_new = g + self.cost
                                else: # 对角线走法
                                    g_new = g + self.cost*1.414
                                f_new = g_new + 0.4 * self.heuristic[x_new][y_new]
                                c_new = np.array([[f_new, g_new, x_new, y_new]])
                                # print('STEPS=',steps,'xnew=',x_new,'ynew=',y_new)
                                cell = np.r_[cell, c_new]  # numpy数组中加入新的点位信息
                                # print('new cell\n',cell)
                                self.close_matrix[x_new][y_new] = 1
                                self.action_matrix[x_new][y_new] = i
                                steps += 1

        path = np.array([[target_point[0], target_point[1]]], dtype=np.int32)
        x = target_point[0]
        y = target_point[1]

        self.steps = steps
        # 开始根据action回溯路径
        while x != self.begin_point[0] or y != self.begin_point[1]:
            x_pre = x - delta[self.action_matrix[x][y]][0]
            y_pre = y - delta[self.action_matrix[x][y]][1]
            x = x_pre
            y = y_pre
            grid_copy[x][y] = 10  # 路径颜色
            path = np.r_[[[x, y]], path]
        self.end = time.time()
        self.path = path
        grid_copy[grid_copy == 1] = 5  # 障碍物颜色
        self.path_grid = grid_copy
        self.close_matrix  = np.zeros(self.shape, dtype=np.int32)  # 生成与grid同大小的0矩阵,用来保存周围坐标
        self.action_matrix = np.zeros(self.shape, dtype=np.int32)  # 生成与grid同大小的0矩阵，用来保存已经计算过的点

    def show_mat(self, title):
        plt.matshow(self.path_grid)
        plt.title(title)
        plt.show()
#
# if __name__ == "__main__":
    # path = "./xunjian_new.pgm"
    # # if 1:
    # if not os.path.exists("final_map.npy"):
    #     MY_MAP = Progress_Pgm(path)
    #     MY_MAP.convert_pgm()
    #     MY_MAP.map_process(6)
    #     # MY_MAP.print_map()
    #     MY_MAP.show_final_map()
    #     my_map = MY_MAP.map
    # else:
    #     my_map = np.load('final_map.npy')
    #     my_map[94,112]= 0
    #     img = Image.fromarray(my_map*255)
    #     img.show()
    # print("ok")
    # #
    # #生成算法实例对象
    # k=Astar(1-my_map)
    # #设置路径起点
    # k.set_start(20,90)
    # #设置目标终点
    # k.set_target(90,20)#设置终点为右下角
    # #计算路径
    # k.calculate()
    # #矩阵绘制显示
    # k.show_mat('Path Output,STEPS='+str(k.steps)+',COST='+str(round(k.realcost,3))+',Time cost='+str(round((k.end-k.start),3))+'s')
