from Lattice_planner_tools import *


if __name__ == '__main__':
    # 从文件加载路径数据
    path_data = np.loadtxt("roadMap_lzjSouth1.txt")
    rx = path_data[:, 1]
    ry = path_data[:, 2]

    theta_thr = M_PI / 6  # 角度阈值，偏离匹配路径的角度范围
    ttcs = [3, 4, 5]  # 静态的升序碰撞时间，单位：秒

    # 定义障碍物信息
    obstacles = []
    obstacles.append(Obstacle([rx[150], ry[150], 0, 0.5, 0.5, M_PI / 6]))
    obstacles.append(Obstacle([rx[300] + 1, ry[300], 0, 1, 1, M_PI / 2]))
    obstacles.append(Obstacle([rx[500] + 1, ry[500], 0, 1, 1, M_PI / 3]))

    # 计算参考路径
    cts_points = np.array([rx, ry])
    path_points = CalcRefLine(cts_points)

    # 定义轨迹点并匹配路径
    tp_list = [rx[0], ry[0], 0, 0, 3., 0]  # 实际上应来自传感器，这里是示例
    traj_point = TrajPoint(tp_list)
    traj_point.MatchPath(path_points)

    # 匹配障碍物路径
    for obstacle in obstacles:
        obstacle.MatchPath(path_points)

    # 样本基础设置
    samp_basis = SampleBasis(traj_point, theta_thr, ttcs)
    local_planner = LocalPlanner(traj_point, path_points, obstacles, samp_basis)
    traj_points_opt = local_planner.LocalPlanning(traj_point, path_points, obstacles, samp_basis)

    v_list = []
    a_list = []

    i = 0
    # 距离目标路径的终点还有距离时，继续规划
    while (Dist(traj_point.x, traj_point.y, rx[-1], ry[-1]) > 2):
        i += 1
        traj_point = traj_points_opt[1]
        traj_point.MatchPath(path_points)
        samp_basis = SampleBasis(traj_point, theta_thr, ttcs)
        local_planner = LocalPlanner(traj_point, path_points, obstacles, samp_basis)
        traj_points_opt = local_planner.LocalPlanning(traj_point, path_points, obstacles, samp_basis)

        # 如果没有找到可行解，扩大采样范围
        if not traj_points_opt:
            print("扩大范围")
            theta_thr_ = M_PI / 3
            ttcs_ = [2, 3, 4, 5, 6, 7, 8]
            samp_basis = SampleBasis(traj_point, theta_thr_, ttcs_)
            local_planner = LocalPlanner(traj_point, path_points, obstacles, samp_basis)
            traj_points_opt = local_planner.LocalPlanning(traj_point, path_points, obstacles, samp_basis)

        # 如果仍然没有找到可行解，输出空解
        if not traj_points_opt:
            traj_points = [[0, 0, 0, 0, 0, 0]]
            print("无解")
            break
        else:
            traj_points = []
            for tp_opt in traj_points_opt:
                traj_points.append([tp_opt.x, tp_opt.y, tp_opt.v, tp_opt.a, tp_opt.theta, tp_opt.kappa])

        # 更新并显示轨迹
        tx = [x[0] for x in traj_points]
        ty = [y[1] for y in traj_points]
        plt.ion()
        plt.figure(1)
        plt.plot(rx, ry, 'b')  # 绘制原始路径
        plt.plot(traj_point.x, traj_point.y, 'or')  # 绘制当前轨迹点
        plt.plot(tx, ty, 'r')  # 绘制优化后的轨迹

        # 绘制障碍物
        for obstacle in obstacles:
            plt.gca().add_patch(plt.Rectangle((obstacle.corner[0], obstacle.corner[1]), obstacle.length,
                                              obstacle.width, color='r', angle=obstacle.heading * 180 / M_PI))
            plt.axis('scaled')

        plt.show()
        plt.pause(0.1)
        plt.clf()
