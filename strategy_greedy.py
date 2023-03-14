import numpy as np
import math

# 各种物品的购入价格
product_buy_prices = [3000, 4400, 5800, 15400, 17200, 19200, 76000]
# 各种物品的售出价格
product_sell_prices = [6000, 7600, 9200, 22500, 25000, 27500, 105000]
# 购入并售出某种物品能赚取的差价
product_profits = [product_sell_prices[i] - product_buy_prices[i] for i in range(len(product_buy_prices))]

# 各种工作台消耗的原材料编号
product_work_bench_consume = [[], [], [], [1, 2], [1, 3], [2, 3], [4, 5, 6], [7], [1, 2, 3, 4, 5, 6, 7]]
# 各种工作台生产的物品编号
product_work_bench_produce = [1, 2, 3, 4, 5, 6, 7]

# 各种工作台的工作周期
production_cycle = [50, 50, 50, 500, 500, 500, 1000, 1, 1]


# work_bench和robot_list是主函数中获取的工作台和机器人的信息
# targets_of_robots为4个机器人的配送策略，形式为[目标取货工作台序号，购买后目标运送货物工作台号,要购买和运送的货物类型编号]
# 初始化时targets_of_robots = [[], [], [], []]
# 每一帧决策前调用此函数，若机器人完成了取货和送货的任务，须将targets_of_robots对应位置置为[]再调用
# 返回函数为更新后的targets_of_robots
def strategy_greedy(work_bench, robot_list, targets_of_robots):
    # 还没有分配目标的机器人编号
    robots_without_target = [i for i in range(4) if targets_of_robots[i] == []]
    if not robots_without_target:
        return targets_of_robots

    # 候选购买材料目的地
    candidate_buy_destinations = []

    # 候选销售产品目的地
    candidate_sell_destinations = []

    # 找出候选购买材料目的地和候选销售产品目的地
    for item in work_bench:
        # 工作台id
        work_bench_serial_number = work_bench.index(item)
        if item['product_state'] == 1:
            # [工作台序号，生产的物品类型，剩余生产时间]
            candidate_buy_destinations.append([work_bench_serial_number, item['id'], 0])
        else:
            if item['produce_remain_time'] >= 0:
                # [工作台序号，生产的物品类型，剩余生产时间]
                candidate_buy_destinations.append([work_bench_serial_number, item['id'], item['produce_remain_time']])

        # 00000010=>2 对应材料1
        # 00000100=>4 对应材料2
        # 00001000=>8 对应材料3
        # 00010000=>16 对应材料4
        # 00100000=>32 对应材料5
        # 01000000=>64 对应材料6
        # 10000000=>128 对应材料7

        # 类型4工作台，收购1和2
        if item['id'] == 4:
            # 如果材料1空缺
            if not item['raw_state'] & 2:
                candidate_sell_destinations.append([work_bench_serial_number, 1])
            # 如果材料2空缺
            if not item['raw_state'] & 4:
                candidate_sell_destinations.append([work_bench_serial_number, 2])
        # 类型5工作台，收购1和3
        if item['id'] == 5:
            # 如果材料1空缺
            if not item['raw_state'] & 2:
                candidate_sell_destinations.append([work_bench_serial_number, 1])
            # 如果材料3空缺
            if not item['raw_state'] & 8:
                candidate_sell_destinations.append([work_bench_serial_number, 3])
        # 类型6工作台，收购2和3
        if item['id'] == 6:
            # 如果材料2空缺
            if not item['raw_state'] & 4:
                candidate_sell_destinations.append([work_bench_serial_number, 2])
            # 如果材料3空缺
            if not item['raw_state'] & 8:
                candidate_sell_destinations.append([work_bench_serial_number, 3])
        # 类型7工作台，收购4，5，6
        if item['id'] == 7:
            # 如果材料4空缺
            if not item['raw_state'] & 16:
                candidate_sell_destinations.append([work_bench_serial_number, 4])
            # 如果材料5空缺
            if not item['raw_state'] & 32:
                candidate_sell_destinations.append([work_bench_serial_number, 5])
            # 如果材料6空缺
            if not item['raw_state'] & 64:
                candidate_sell_destinations.append([work_bench_serial_number, 6])
        # 类型8工作台，收购7
        if item['id'] == 8:
            candidate_sell_destinations.append([work_bench_serial_number, 7])
        # 类型9工作台，收购1-7
        if item['id'] == 9:
            for i in range(7):
                candidate_sell_destinations.append([work_bench_serial_number, i + 1])

    # 构造所有的送货策略组合，格式为[取货点工作台序号，送货点工作台序号，配送货物类型编号，收益，耗费时间（取货点到送货点以最大速度配送预计需要的时间，以帧为单位）]
    distribution_strategies = []
    for departure in candidate_buy_destinations:
        for destination in candidate_sell_destinations:
            if departure[1] == destination[1]:
                # 起点坐标
                departure_xy = np.array([work_bench[departure[0]]['x'], work_bench[departure[0]]['y']])
                destination_xy = np.array([work_bench[destination[0]]['x'], work_bench[destination[0]]['y']])
                distance = np.linalg.norm(departure_xy - destination_xy)
                time = math.ceil(distance / 6 * 1000 / 20)
                distribution_strategies.append(
                    [departure[0], destination[0], departure[1], product_profits[departure[1]], time])

    # 为每个机器人找出前50个平均收益最大的方案
    num_of_robots_without_targets = len(robots_without_target)
    top_50_targets_for_robots = []
    for robot_id in robots_without_target:
        robot = robot_list[robot_id]
        robot_xy = np.array([robot['x'], robot['y']])
        # top_50_targets_for_this_robot内容格式为[取货点工作台序号，送货点工作台序号，配送货物类型编号，平均每帧收益]
        top_50_targets_for_this_robot = []
        for strategy in distribution_strategies:
            departure = strategy[0]  # 取货点工作台序号
            destination = strategy[1]  # 送货点工作台序号
            product_type = strategy[2]
            profit = strategy[3]
            time_from_departure_to_destination = strategy[4]
            departure_xy = np.array([work_bench[departure]['x'], work_bench[departure]['y']])
            distance_from_robot_to_departure = np.linalg.norm(robot_xy - departure_xy)
            time_from_robot_to_departure = math.ceil(distance_from_robot_to_departure / 6 * 1000 / 20)
            profit_per_frame = profit / (time_from_departure_to_destination + time_from_robot_to_departure)

            insert_index = len(top_50_targets_for_this_robot)
            for target in top_50_targets_for_this_robot:
                if target[3] <= profit_per_frame:
                    insert_index = top_50_targets_for_this_robot.index(target)
                    break
            if insert_index <= 49:
                top_50_targets_for_this_robot.insert(insert_index,
                                                     [departure, destination, product_type, profit_per_frame])

        top_50_targets_for_robots.append(top_50_targets_for_this_robot)

    for i in range(num_of_robots_without_targets):  # 为每一个机器人寻找方案
        for target in top_50_targets_for_robots[i]:  # 遍历该机器人的待选方案
            flag = False  # 该方案与其他机器人已选方案是否冲突
            for selected_target in targets_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
                if not selected_target:
                    continue
                if selected_target[0] == target[0] and selected_target[2] == target[2] or selected_target[1] == target[
                    1] and selected_target[2] == target[2]:
                    flag = True
                    break
            if not flag:
                targets_of_robots[robots_without_target[i]] = target[0:3]
                break

    return targets_of_robots
