import sys

import numpy as np

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


# work_bench_list和robot_list是主函数中获取的工作台和机器人的信息
# strategies_of_robots为4个机器人的配送策略，形式为{目标取货工作台序号(departure_work_bench_id)，购买后目标运送货物工作台号(destination_work_bench_id),
# 要购买和运送的货物类型编号(product_type)，机器人当前手中货物类型(carried_product_type,0表示未取货，1-7表示手中货物类型)}
# 初始化时strategies_of_robots = [{}, {}, {}, {}]
# 每一帧决策前调用此函数，若机器人完成了取货和送货的任务，须将strategies_of_robots对应位置策略置为{}
# 返回函数为更新后的strategies_of_robots
# nearest_sell_place记录的是图中所有的4，5，6，7类型机器人的产物距离最近的收购点，key为4，5，6，7类型工作台id，value为最近销售目标工作台id和距离
def strategy_greedy_for_map_1(work_bench_list, robot_list, strategies_of_robots, frame_id,
                              work_bench_statistics_by_type, work_bench_list_by_type):
    top_N = 30  # 取前top_N个最好的策略
    # 候选购买材料目的地,格式为{工作台id(work_bench_id)，生产的物品类型(product_type)，剩余生产时间(produce_remain_time)}
    candidate_buy_destinations = []

    # 候选销售产品目的地,格式为{工作台id（work_bench_id），需要的物品类型(material_requested)}
    candidate_sell_destinations = []

    # 找出候选购买材料目的地和候选销售产品目的地
    for work_bench in work_bench_list:
        # 工作台id
        if work_bench['product_state'] == 1:
            candidate_buy_destinations.append(
                {'work_bench_id': work_bench['id'], 'product_type': work_bench['type'], 'produce_remain_time': 0})

        # 00000010=>2 对应材料1
        # 00000100=>4 对应材料2
        # 00001000=>8 对应材料3
        # 00010000=>16 对应材料4
        # 00100000=>32 对应材料5
        # 01000000=>64 对应材料6
        # 10000000=>128 对应材料7

        # 类型4工作台，收购1和2
        if work_bench['type'] == 4:
            # 如果材料1空缺
            if not work_bench['material_state'] & 2:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
            # 如果材料2空缺
            if not work_bench['material_state'] & 4:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
        # 类型5工作台，收购1和3
        if work_bench['type'] == 5:
            # 如果材料1空缺
            if not work_bench['material_state'] & 2:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
            # 如果材料3空缺
            if not work_bench['material_state'] & 8:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
        # 类型6工作台，收购2和3
        if work_bench['type'] == 6:
            # 如果材料2空缺
            if not work_bench['material_state'] & 4:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
            # 如果材料3空缺
            if not work_bench['material_state'] & 8:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
        # 类型7工作台，收购4，5，6
        if work_bench['type'] == 7:
            # 如果材料4空缺
            if not work_bench['material_state'] & 16:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 4})
            # 如果材料5空缺
            if not work_bench['material_state'] & 32:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 5})
            # 如果材料6空缺
            if not work_bench['material_state'] & 64:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 6})
        # 类型8工作台，收购7
        if work_bench['type'] == 8:
            candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 7})
        # 类型9工作台，收购1-7
        if work_bench['type'] == 9:
            if work_bench_statistics_by_type['7']['num_of_this_type'] != 0:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 7})

    # 构造所有的送货策略组合，格式为字典{取货点工作台id（departure_work_bench_id），送货点工作台id(destination_work_bench_id)，
    # 配送货物类型编号(product_type,1-7)，收益(profit),距离（distance）
    # 潜在收益（potential_profit,送货点工作台的产物可能产生的收益的一部分作为潜在收益，4，5，6号工作台产物收益的一半作为潜在收益，7号工作台产物收益的1/3作为潜在收益）}
    # 获得潜在收益需要多跑的距离（distance_for_potential_profit）
    distribution_strategies = []
    for departure in candidate_buy_destinations:
        for destination in candidate_sell_destinations:
            if departure['product_type'] == destination['material_requested']:
                # 起点坐标
                departure_xy = np.array([work_bench_list[departure['work_bench_id']]['x'],
                                         work_bench_list[departure['work_bench_id']]['y']])
                destination_xy = np.array([work_bench_list[destination['work_bench_id']]['x'],
                                           work_bench_list[destination['work_bench_id']]['y']])
                distance = np.linalg.norm(departure_xy - destination_xy)

                distribution_strategies.append(
                    {'departure_work_bench_id': departure['work_bench_id'],
                     'destination_work_bench_id': destination['work_bench_id'],
                     'product_type': departure['product_type'],
                     'profit': product_profits[departure['product_type'] - 1],
                     'distance': distance,
                     })

    # 若机器人R当前任务是去A工作台取货，且有另一个机器人去A工作台送货，则机器人R应该立即放弃当前任务
    for robot in robot_list:
        if strategies_of_robots[robot['id']] == {}:
            continue
        for other_robot in robot_list:
            if robot['id'] == other_robot['id'] or strategies_of_robots[other_robot['id']] == {}:
                continue
            else:
                if strategies_of_robots[other_robot['id']]['carried_product_type'] != 0 and \
                        strategies_of_robots[other_robot['id']]['destination_work_bench_id'] == \
                        strategies_of_robots[robot['id']]['departure_work_bench_id'] and \
                        strategies_of_robots[robot['id']]['carried_product_type'] == 0:
                    strategies_of_robots[robot['id']] = {}
                    break

    # 为每个机器人找出前N个平均每米收益最大的方案
    top_n_strategies_for_robots = []
    for robot in robot_list:
        # top_n_strategies_for_this_robot内容格式为{取货点工作台序号(departure_work_bench_id)，送货点工作台序号(destination_work_bench_id)，
        # 配送货物类型编号(product_type)，平均每米收益(profit_per_meter)}
        top_n_strategies_for_this_robot = []
        if strategies_of_robots[robot['id']] != {}:
            top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)
            continue
        robot_xy = np.array([robot['x'], robot['y']])
        for strategy in distribution_strategies:
            departure_xy = np.array([work_bench_list[strategy['departure_work_bench_id']]['x'],
                                     work_bench_list[strategy['departure_work_bench_id']]['y']])
            distance_from_robot_to_departure = np.linalg.norm(robot_xy - departure_xy)
            distance_from_departure_to_destination = strategy['distance']
            # 如果当前任务在游戏结束前无法完成（机器人理论需要跑动的距离*延误系数>剩余时间以最快速度最多能跑的距离），则直接pass，
            if (distance_from_robot_to_departure + distance_from_departure_to_destination) * 1.5 > (
                    9000 - frame_id) * 0.12:
                continue
            base_profit_per_meter = (strategy['profit']) / (
                    distance_from_robot_to_departure + distance_from_departure_to_destination)
            profit_per_meter = base_profit_per_meter
            departure_work_bench = work_bench_list[strategy['departure_work_bench_id']]
            destination_work_bench = work_bench_list[strategy['destination_work_bench_id']]
            departure_work_bench_type = departure_work_bench['type']
            destination_work_bench_type = destination_work_bench['type']
            bonus_index = 0.5  # 奖励系数
            # 材料1可以用于生产4和5，如果已经送给生产4的材料更多，则这一个应当送给5
            if departure_work_bench_type == 1 and destination_work_bench_type == 5:
                if work_bench_statistics_by_type['4']['total_of_material_type_1'] > work_bench_statistics_by_type['5'][
                    'total_of_material_type_1']:
                    profit_per_meter += base_profit_per_meter * bonus_index
                    # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
                    if destination_work_bench['material_state'] > 0 and (
                            destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
                        profit_per_meter += base_profit_per_meter * bonus_index
                    # 或者有其他机器人正在往目标送货工作台送一个不同的材料
                    else:
                        for strategy_of_other_robot in strategies_of_robots:
                            if strategy_of_other_robot == {}:
                                continue
                            if strategy_of_other_robot['destination_work_bench_id'] == strategy[
                                'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
                                'product_type']:
                                profit_per_meter += base_profit_per_meter * bonus_index
                                break

            # 材料1可以用于生产4和5，如果已经送给生产5的材料更多，则这一个应当送给4
            if departure_work_bench_type == 1 and destination_work_bench_type == 4:
                if work_bench_statistics_by_type['4']['total_of_material_type_1'] < \
                        work_bench_statistics_by_type['5'][
                            'total_of_material_type_1']:
                    profit_per_meter += base_profit_per_meter * bonus_index
                    # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
                    if destination_work_bench['material_state'] > 0 and (
                            destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
                        profit_per_meter += base_profit_per_meter * bonus_index
                    # 或者有其他机器人正在往目标送货工作台送一个不同的材料
                    else:
                        for strategy_of_other_robot in strategies_of_robots:
                            if strategy_of_other_robot == {}:
                                continue
                            if strategy_of_other_robot['destination_work_bench_id'] == strategy[
                                'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
                                'product_type']:
                                profit_per_meter += base_profit_per_meter * bonus_index
                                break
            # 材料2可以用于生产4和6，如果已经送给生产4的材料更多，则这一个应当送给6
            if departure_work_bench_type == 2 and destination_work_bench_type == 4:
                if work_bench_statistics_by_type['4']['total_of_material_type_2'] < work_bench_statistics_by_type['6'][
                    'total_of_material_type_2']:
                    profit_per_meter += base_profit_per_meter * bonus_index
                    # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
                    if destination_work_bench['material_state'] > 0 and (
                            destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
                        profit_per_meter += base_profit_per_meter * bonus_index
                    # 或者有其他机器人正在往目标送货工作台送一个不同的材料
                    else:
                        for strategy_of_other_robot in strategies_of_robots:
                            if strategy_of_other_robot == {}:
                                continue
                            if strategy_of_other_robot['destination_work_bench_id'] == strategy[
                                'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
                                'product_type']:
                                profit_per_meter += base_profit_per_meter * bonus_index
                                break
            if departure_work_bench_type == 2 and destination_work_bench_type == 6:
                if work_bench_statistics_by_type['4']['total_of_material_type_2'] > work_bench_statistics_by_type['6'][
                    'total_of_material_type_2']:
                    profit_per_meter += base_profit_per_meter * bonus_index
                    # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
                    if destination_work_bench['material_state'] > 0 and (
                            destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
                        profit_per_meter += base_profit_per_meter * bonus_index
                    # 或者有其他机器人正在往目标送货工作台送一个不同的材料
                    else:
                        for strategy_of_other_robot in strategies_of_robots:
                            if strategy_of_other_robot == {}:
                                continue
                            if strategy_of_other_robot['destination_work_bench_id'] == strategy[
                                'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
                                'product_type']:
                                profit_per_meter += base_profit_per_meter * bonus_index
                                break
            # 材料3可以用于生产5和6，如果已经送给生产5的材料更多，则这一个应当送给6，反之给5
            if departure_work_bench_type == 3 and destination_work_bench_type == 5:
                if work_bench_statistics_by_type['5']['total_of_material_type_3'] < work_bench_statistics_by_type['6'][
                    'total_of_material_type_3']:
                    profit_per_meter += base_profit_per_meter * bonus_index
                    # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
                    if destination_work_bench['material_state'] > 0 and (
                            destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
                        profit_per_meter += base_profit_per_meter * bonus_index
                    # 或者有其他机器人正在往目标送货工作台送一个不同的材料
                    else:
                        for strategy_of_other_robot in strategies_of_robots:
                            if strategy_of_other_robot == {}:
                                continue
                            if strategy_of_other_robot['destination_work_bench_id'] == strategy[
                                'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
                                'product_type']:
                                profit_per_meter += base_profit_per_meter * bonus_index
                                break
            if departure_work_bench_type == 3 and destination_work_bench_type == 6:
                if work_bench_statistics_by_type['5']['total_of_material_type_3'] > work_bench_statistics_by_type['6'][
                    'total_of_material_type_3']:
                    profit_per_meter += base_profit_per_meter * bonus_index
                    # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
                    if destination_work_bench['material_state'] > 0 and (
                            destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
                        profit_per_meter += base_profit_per_meter * bonus_index
                    # 或者有其他机器人正在往目标送货工作台送一个不同的材料
                    else:
                        for strategy_of_other_robot in strategies_of_robots:
                            if strategy_of_other_robot == {}:
                                continue
                            if strategy_of_other_robot['destination_work_bench_id'] == strategy[
                                'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
                                'product_type']:
                                profit_per_meter += base_profit_per_meter * bonus_index
                                break
            if destination_work_bench_type == 7:
                profit_per_meter += 0.7 * base_profit_per_meter
                if work_bench_list[strategy['destination_work_bench_id']]['material_state'] in [16, 32, 64]:
                    profit_per_meter += 0.9 * base_profit_per_meter
                if work_bench_list[strategy['destination_work_bench_id']]['material_state'] in [48, 80, 96]:
                    profit_per_meter += 1.1 * base_profit_per_meter
            if 8 <= destination_work_bench_type <= 9:
                profit_per_meter += 1.2 * base_profit_per_meter
            # 插入排序，寻找插入点
            insert_index = len(top_n_strategies_for_this_robot)
            for top_50_strategy in top_n_strategies_for_this_robot:
                if top_50_strategy['profit_per_meter'] <= profit_per_meter:
                    insert_index = top_n_strategies_for_this_robot.index(top_50_strategy)
                    break
            if insert_index <= top_N - 1:
                top_n_strategies_for_this_robot.insert(insert_index,
                                                       {'departure_work_bench_id': strategy['departure_work_bench_id'],
                                                        'destination_work_bench_id': strategy[
                                                            'destination_work_bench_id'],
                                                        'product_type': strategy['product_type'],
                                                        'profit_per_meter': profit_per_meter})
        top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)
    for robot in robot_list:
        if robot['carried_product_type'] > 0:
            destination_work_bench = work_bench_list[strategies_of_robots[robot['id']]['destination_work_bench_id']]
            product_type = robot['carried_product_type']
            if (destination_work_bench['material_state'] >> product_type) & 1 == 1:
                destination_work_bench_type = destination_work_bench['type']
                # 找一个不冲突的工作台
                for work_bench in work_bench_list_by_type[destination_work_bench_type - 1]:
                    if work_bench['material_state'] >> product_type & 1 != 1:
                        flag = False  # 该方案与其他机器人已选方案是否冲突
                        for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
                            if selected_strategy == {}:
                                continue
                            if selected_strategy['product_type'] == product_type and selected_strategy[
                                'destination_work_bench_id'] == work_bench['id']:
                                flag = True
                                break
                        if not flag:
                            strategies_of_robots[robot['id']]['destination_work_bench_id'] = work_bench['id']
            # 看看可不可以配送到更好的地方，比如配送到一个已经有一部分原材料的地方
            destination_work_bench = work_bench_list[strategies_of_robots[robot['id']]['destination_work_bench_id']]
            if destination_work_bench['material_state'] == 0:
                destination_work_bench_type = destination_work_bench['type']
                product_type = robot['carried_product_type']
                # 找一个拥有互补材料的工作台
                for work_bench in work_bench_list_by_type[destination_work_bench_type - 1]:
                    if work_bench['material_state'] > 0 and (work_bench['material_state'] >> product_type) & 1 != 1:
                        flag = False  # 该方案与其他机器人已选方案是否冲突
                        for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
                            if selected_strategy == {}:
                                continue
                            if selected_strategy['product_type'] == product_type and selected_strategy[
                                'destination_work_bench_id'] == work_bench['id']:
                                flag = True
                                break
                        if not flag:
                            strategies_of_robots[robot['id']]['destination_work_bench_id'] = work_bench['id']
    for robot in robot_list:  # 为每一个机器人寻找方案
        # 如果机器人已经取到货，则看看可不可以配送到更好的地方，比如配送到一个已经有一部分原材料的地方
        if strategies_of_robots[robot['id']] != {}:
            continue
        for strategy in top_n_strategies_for_robots[robot['id']]:  # 遍历该机器人的待选方案
            flag = False  # 该方案与其他机器人已选方案是否冲突
            for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
                if selected_strategy == {}:
                    continue
                if selected_strategy['departure_work_bench_id'] == strategy['departure_work_bench_id'] and \
                        selected_strategy['product_type'] == strategy['product_type']:
                    flag = True
                    break
            if not flag:
                strategies_of_robots[robot['id']] = strategy
                strategies_of_robots[robot['id']]['carried_product_type'] = 0
    for robot in robot_list:
        if strategies_of_robots[robot['id']] == {}:
            for strategy in top_n_strategies_for_robots[robot['id']]:  # 遍历该机器人的待选方案
                flag = False  # 该方案与其他机器人已选方案是否冲突
                for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
                    if selected_strategy == {}:
                        continue
                    if selected_strategy['departure_work_bench_id'] == strategy['departure_work_bench_id'] and \
                            selected_strategy['product_type'] == strategy['product_type'] and not (
                            1 <= strategy['product_type'] <= 3):
                        flag = True
                        break
                if not flag:
                    strategies_of_robots[robot['id']] = strategy
                    strategies_of_robots[robot['id']]['carried_product_type'] = 0

    return strategies_of_robots


# def strategy_greedy_for_map_2(work_bench_list, robot_list, strategies_of_robots, frame_id,
#                               work_bench_statistics_by_type, work_bench_list_by_type):
#     top_N = 30  # 取前top_N个最好的策略
#     # 候选购买材料目的地,格式为{工作台id(work_bench_id)，生产的物品类型(product_type)，剩余生产时间(produce_remain_time)}
#     candidate_buy_destinations = []
#
#     # 候选销售产品目的地,格式为{工作台id（work_bench_id），需要的物品类型(material_requested)}
#     candidate_sell_destinations = []
#
#     # 找出候选购买材料目的地和候选销售产品目的地
#     for work_bench in work_bench_list:
#         # 工作台id
#         if work_bench['product_state'] == 1:
#             candidate_buy_destinations.append(
#                 {'work_bench_id': work_bench['id'], 'product_type': work_bench['type'], 'produce_remain_time': 0})
#
#         # 00000010=>2 对应材料1
#         # 00000100=>4 对应材料2
#         # 00001000=>8 对应材料3
#         # 00010000=>16 对应材料4
#         # 00100000=>32 对应材料5
#         # 01000000=>64 对应材料6
#         # 10000000=>128 对应材料7
#
#         # 类型4工作台，收购1和2
#         if work_bench['type'] == 4:
#             # 如果材料1空缺
#             if not work_bench['material_state'] & 2:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
#             # 如果材料2空缺
#             if not work_bench['material_state'] & 4:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
#         # 类型5工作台，收购1和3
#         if work_bench['type'] == 5:
#             # 如果材料1空缺
#             if not work_bench['material_state'] & 2:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
#             # 如果材料3空缺
#             if not work_bench['material_state'] & 8:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
#         # 类型6工作台，收购2和3
#         if work_bench['type'] == 6:
#             # 如果材料2空缺
#             if not work_bench['material_state'] & 4:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
#             # 如果材料3空缺
#             if not work_bench['material_state'] & 8:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
#         # 类型7工作台，收购4，5，6
#         if work_bench['type'] == 7:
#             # 如果材料4空缺
#             if not work_bench['material_state'] & 16:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 4})
#             # 如果材料5空缺
#             if not work_bench['material_state'] & 32:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 5})
#             # 如果材料6空缺
#             if not work_bench['material_state'] & 64:
#                 candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 6})
#         # 类型8工作台，收购7
#         if work_bench['type'] == 8:
#             candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 7})
#
#     # 构造所有的送货策略组合，格式为字典{取货点工作台id（departure_work_bench_id），送货点工作台id(destination_work_bench_id)，
#     # 配送货物类型编号(product_type,1-7)，收益(profit),距离（distance）
#     # 潜在收益（potential_profit,送货点工作台的产物可能产生的收益的一部分作为潜在收益，4，5，6号工作台产物收益的一半作为潜在收益，7号工作台产物收益的1/3作为潜在收益）}
#     # 获得潜在收益需要多跑的距离（distance_for_potential_profit）
#     distribution_strategies = []
#     for departure in candidate_buy_destinations:
#         for destination in candidate_sell_destinations:
#             if departure['product_type'] == destination['material_requested']:
#                 # 起点坐标
#                 departure_xy = np.array([work_bench_list[departure['work_bench_id']]['x'],
#                                          work_bench_list[departure['work_bench_id']]['y']])
#                 destination_xy = np.array([work_bench_list[destination['work_bench_id']]['x'],
#                                            work_bench_list[destination['work_bench_id']]['y']])
#                 distance = np.linalg.norm(departure_xy - destination_xy)
#
#                 distribution_strategies.append(
#                     {'departure_work_bench_id': departure['work_bench_id'],
#                      'destination_work_bench_id': destination['work_bench_id'],
#                      'product_type': departure['product_type'],
#                      'profit': product_profits[departure['product_type'] - 1],
#                      'distance': distance,
#                      })
#
#     # 若机器人R当前任务是去A工作台取货，且有另一个机器人去A工作台送货，则机器人R应该立即放弃当前任务
#     for robot in robot_list:
#         if strategies_of_robots[robot['id']] == {}:
#             continue
#         for other_robot in robot_list:
#             if robot['id'] == other_robot['id'] or strategies_of_robots[other_robot['id']] == {}:
#                 continue
#             else:
#                 if strategies_of_robots[other_robot['id']]['carried_product_type'] != 0 and \
#                         strategies_of_robots[other_robot['id']]['destination_work_bench_id'] == \
#                         strategies_of_robots[robot['id']]['departure_work_bench_id'] and \
#                         strategies_of_robots[robot['id']]['carried_product_type'] == 0:
#                     strategies_of_robots[robot['id']] = {}
#                     break
#
#     # 为每个机器人找出前N个平均每米收益最大的方案
#     top_n_strategies_for_robots = []
#     for robot in robot_list:
#         # top_n_strategies_for_this_robot内容格式为{取货点工作台序号(departure_work_bench_id)，送货点工作台序号(destination_work_bench_id)，
#         # 配送货物类型编号(product_type)，平均每米收益(profit_per_meter)}
#         top_n_strategies_for_this_robot = []
#         if strategies_of_robots[robot['id']] != {}:
#             top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)
#             continue
#         robot_xy = np.array([robot['x'], robot['y']])
#         for strategy in distribution_strategies:
#             departure_xy = np.array([work_bench_list[strategy['departure_work_bench_id']]['x'],
#                                      work_bench_list[strategy['departure_work_bench_id']]['y']])
#             distance_from_robot_to_departure = np.linalg.norm(robot_xy - departure_xy)
#             distance_from_departure_to_destination = strategy['distance']
#             # 如果当前任务在游戏结束前无法完成（机器人理论需要跑动的距离*延误系数>剩余时间以最快速度最多能跑的距离），则直接pass，
#             if (distance_from_robot_to_departure + distance_from_departure_to_destination) * 1.5 > (
#                     9000 - frame_id) * 0.12:
#                 continue
#             base_profit_per_meter = (strategy['profit']) / (
#                     distance_from_robot_to_departure + distance_from_departure_to_destination)
#             profit_per_meter = base_profit_per_meter
#             departure_work_bench = work_bench_list[strategy['departure_work_bench_id']]
#             destination_work_bench = work_bench_list[strategy['destination_work_bench_id']]
#             departure_work_bench_type = departure_work_bench['type']
#             destination_work_bench_type = destination_work_bench['type']
#             bonus_index = 0.5  # 奖励系数
#             # 材料1可以用于生产4和5，如果已经送给生产4的材料更多，则这一个应当送给5
#             if departure_work_bench_type == 1 and destination_work_bench_type == 5 and \
#                     work_bench_statistics_by_type['5']['num_of_products'] <= 1:
#                 if work_bench_statistics_by_type['4']['total_of_material_type_1'] > work_bench_statistics_by_type['5'][
#                     'total_of_material_type_1']:
#                     profit_per_meter += base_profit_per_meter * bonus_index
#                     # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
#                     if destination_work_bench['material_state'] > 0 and (
#                             destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
#                         profit_per_meter += base_profit_per_meter * bonus_index
#                     # 或者有其他机器人正在往目标送货工作台送一个不同的材料
#                     else:
#                         for strategy_of_other_robot in strategies_of_robots:
#                             if strategy_of_other_robot == {}:
#                                 continue
#                             if strategy_of_other_robot['destination_work_bench_id'] == strategy[
#                                 'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
#                                 'product_type']:
#                                 profit_per_meter += base_profit_per_meter * bonus_index
#                                 break
#
#             # 材料1可以用于生产4和5，如果已经送给生产5的材料更多，则这一个应当送给4
#             if departure_work_bench_type == 1 and destination_work_bench_type == 4 and \
#                     work_bench_statistics_by_type['4']['num_of_products'] <= 1:
#                 if work_bench_statistics_by_type['4']['total_of_material_type_1'] < \
#                         work_bench_statistics_by_type['5'][
#                             'total_of_material_type_1']:
#                     profit_per_meter += base_profit_per_meter * bonus_index
#                     # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
#                     if destination_work_bench['material_state'] > 0 and (
#                             destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
#                         profit_per_meter += base_profit_per_meter * bonus_index
#                     # 或者有其他机器人正在往目标送货工作台送一个不同的材料
#                     else:
#                         for strategy_of_other_robot in strategies_of_robots:
#                             if strategy_of_other_robot == {}:
#                                 continue
#                             if strategy_of_other_robot['destination_work_bench_id'] == strategy[
#                                 'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
#                                 'product_type']:
#                                 profit_per_meter += base_profit_per_meter * bonus_index
#                                 break
#             # 材料2可以用于生产4和6，如果已经送给生产4的材料更多，则这一个应当送给6
#             if departure_work_bench_type == 2 and destination_work_bench_type == 4 and \
#                     work_bench_statistics_by_type['4']['num_of_products'] <= 1:
#                 if work_bench_statistics_by_type['4']['total_of_material_type_2'] < work_bench_statistics_by_type['6'][
#                     'total_of_material_type_2']:
#                     profit_per_meter += base_profit_per_meter * bonus_index
#                     # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
#                     if destination_work_bench['material_state'] > 0 and (
#                             destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
#                         profit_per_meter += base_profit_per_meter * bonus_index
#                     # 或者有其他机器人正在往目标送货工作台送一个不同的材料
#                     else:
#                         for strategy_of_other_robot in strategies_of_robots:
#                             if strategy_of_other_robot == {}:
#                                 continue
#                             if strategy_of_other_robot['destination_work_bench_id'] == strategy[
#                                 'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
#                                 'product_type']:
#                                 profit_per_meter += base_profit_per_meter * bonus_index
#                                 break
#             if departure_work_bench_type == 2 and destination_work_bench_type == 6 and \
#                     work_bench_statistics_by_type['6']['num_of_products'] <= 1:
#                 if work_bench_statistics_by_type['4']['total_of_material_type_2'] > work_bench_statistics_by_type['6'][
#                     'total_of_material_type_2']:
#                     profit_per_meter += base_profit_per_meter * bonus_index
#                     # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
#                     if destination_work_bench['material_state'] > 0 and (
#                             destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
#                         profit_per_meter += base_profit_per_meter * bonus_index
#                     # 或者有其他机器人正在往目标送货工作台送一个不同的材料
#                     else:
#                         for strategy_of_other_robot in strategies_of_robots:
#                             if strategy_of_other_robot == {}:
#                                 continue
#                             if strategy_of_other_robot['destination_work_bench_id'] == strategy[
#                                 'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
#                                 'product_type']:
#                                 profit_per_meter += base_profit_per_meter * bonus_index
#                                 break
#             # 材料3可以用于生产5和6，如果已经送给生产5的材料更多，则这一个应当送给6，反之给5
#             if departure_work_bench_type == 3 and destination_work_bench_type == 5 and \
#                     work_bench_statistics_by_type['5']['num_of_products'] <= 1:
#                 if work_bench_statistics_by_type['5']['total_of_material_type_3'] < work_bench_statistics_by_type['6'][
#                     'total_of_material_type_3']:
#                     profit_per_meter += base_profit_per_meter * bonus_index
#                     # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
#                     if destination_work_bench['material_state'] > 0 and (
#                             destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
#                         profit_per_meter += base_profit_per_meter * bonus_index
#                     # 或者有其他机器人正在往目标送货工作台送一个不同的材料
#                     else:
#                         for strategy_of_other_robot in strategies_of_robots:
#                             if strategy_of_other_robot == {}:
#                                 continue
#                             if strategy_of_other_robot['destination_work_bench_id'] == strategy[
#                                 'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
#                                 'product_type']:
#                                 profit_per_meter += base_profit_per_meter * bonus_index
#                                 break
#             if departure_work_bench_type == 3 and destination_work_bench_type == 6 and \
#                     work_bench_statistics_by_type['6']['num_of_products'] <= 1:
#                 if work_bench_statistics_by_type['5']['total_of_material_type_3'] > work_bench_statistics_by_type['6'][
#                     'total_of_material_type_3']:
#                     profit_per_meter += base_profit_per_meter * bonus_index
#                     # 如果目标送货工作台有一个材料且与现在送的不同，则再给一个奖励
#                     if destination_work_bench['material_state'] > 0 and (
#                             destination_work_bench['material_state'] >> departure_work_bench_type) & 1 != 1:
#                         profit_per_meter += base_profit_per_meter * bonus_index
#                     # 或者有其他机器人正在往目标送货工作台送一个不同的材料
#                     else:
#                         for strategy_of_other_robot in strategies_of_robots:
#                             if strategy_of_other_robot == {}:
#                                 continue
#                             if strategy_of_other_robot['destination_work_bench_id'] == strategy[
#                                 'destination_work_bench_id'] and strategy_of_other_robot['product_type'] != strategy[
#                                 'product_type']:
#                                 profit_per_meter += base_profit_per_meter * bonus_index
#                                 break
#             if destination_work_bench_type == 7:
#                 profit_per_meter += 4 * base_profit_per_meter
#                 if work_bench_list[strategy['destination_work_bench_id']]['material_state'] in [16, 32, 64]:
#                     profit_per_meter += 1 * base_profit_per_meter
#                 if work_bench_list[strategy['destination_work_bench_id']]['material_state'] in [48, 80, 96]:
#                     profit_per_meter += 1 * base_profit_per_meter
#             if destination_work_bench_type == 8:
#                 profit_per_meter += 3 * base_profit_per_meter
#             # 插入排序，寻找插入点
#             insert_index = len(top_n_strategies_for_this_robot)
#             for top_50_strategy in top_n_strategies_for_this_robot:
#                 if top_50_strategy['profit_per_meter'] <= profit_per_meter:
#                     insert_index = top_n_strategies_for_this_robot.index(top_50_strategy)
#                     break
#             if insert_index <= top_N - 1:
#                 top_n_strategies_for_this_robot.insert(insert_index,
#                                                        {'departure_work_bench_id': strategy['departure_work_bench_id'],
#                                                         'destination_work_bench_id': strategy[
#                                                             'destination_work_bench_id'],
#                                                         'product_type': strategy['product_type'],
#                                                         'profit_per_meter': profit_per_meter})
#         top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)
#     # 如果机器人已取到货，看看可不可以配送到更好的地方，比如配送到一个已经有一部分原材料的地方
#     for robot in robot_list:
#         if robot['carried_product_type'] == 0:
#             continue
#         destination_work_bench = work_bench_list[strategies_of_robots[robot['id']]['destination_work_bench_id']]
#         if destination_work_bench['material_state'] == 0:
#             destination_work_bench_type = destination_work_bench['type']
#             product_type = robot['carried_product_type']
#             # 找一个拥有互补材料的工作台
#             for work_bench in work_bench_list_by_type[destination_work_bench_type - 1]:
#                 # 这个工作台有部分材料且与运送的材料不同
#                 if work_bench['material_state'] > 0 and (work_bench['material_state'] >> product_type) & 1 != 1:
#                     flag = False  # 该方案与其他机器人已选方案是否冲突
#                     for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
#                         if selected_strategy == {}:
#                             continue
#                         if selected_strategy['product_type'] == product_type and selected_strategy[
#                             'destination_work_bench_id'] == work_bench['id']:
#                             flag = True
#                             break
#                     if not flag:
#                         strategies_of_robots[robot['id']]['destination_work_bench_id'] = work_bench['id']
#     # 为机器人分配方案
#     for robot in robot_list:
#         if strategies_of_robots[robot['id']] == {}:
#             for strategy in top_n_strategies_for_robots[robot['id']]:  # 遍历该机器人的待选方案
#                 flag = False  # 该方案与其他机器人已选方案是否冲突
#                 for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
#                     if selected_strategy == {}:
#                         continue
#                     if (selected_strategy['departure_work_bench_id'] == strategy['departure_work_bench_id'] and
#                         selected_strategy['product_type'] == strategy['product_type'] and selected_strategy[
#                             'carried_product_type'] == 0) or (
#                             selected_strategy['product_type'] == strategy['product_type'] and
#                             selected_strategy['destination_work_bench_id'] == strategy['destination_work_bench_id']):
#                         flag = True
#                         break
#                 if not flag:
#                     strategies_of_robots[robot['id']] = strategy
#                     strategies_of_robots[robot['id']]['carried_product_type'] = 0
#
#     return strategies_of_robots
def strategy_greedy_for_map_2(work_bench_list, robot_list, strategies_of_robots, frame_id, nearest_sell_place,
                              stimulate_factor_for_product_6):
    top_N = 20

    # 候选购买材料目的地,格式为{工作台id(work_bench_id)，生产的物品类型(product_type)，剩余生产时间(produce_remain_time)}
    candidate_buy_destinations = []

    # 候选销售产品目的地,格式为{工作台id（work_bench_id），需要的物品类型(material_requested)}
    candidate_sell_destinations = []

    # 找出候选购买材料目的地和候选销售产品目的地
    for work_bench in work_bench_list:
        # 工作台id
        if work_bench['product_state'] == 1:
            candidate_buy_destinations.append(
                {'work_bench_id': work_bench['id'], 'product_type': work_bench['type'], 'produce_remain_time': 0})
        # 由于不考虑剩余生产时间分数更高，所以只在第一帧时考虑剩余时间，以免前50帧机器人静止不动
        elif work_bench['produce_remain_time'] > 0 and frame_id == 1:
            candidate_buy_destinations.append({'work_bench_id': work_bench['id'], 'product_type': work_bench['type'],
                                               'produce_remain_time': work_bench['produce_remain_time']})

        # 00000010=>2 对应材料1
        # 00000100=>4 对应材料2
        # 00001000=>8 对应材料3
        # 00010000=>16 对应材料4
        # 00100000=>32 对应材料5
        # 01000000=>64 对应材料6
        # 10000000=>128 对应材料7

        # 类型4工作台，收购1和2
        if work_bench['type'] == 4:
            # 如果材料1空缺
            if not work_bench['material_state'] & 2:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
            # 如果材料2空缺
            if not work_bench['material_state'] & 4:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
        # 类型5工作台，收购1和3
        if work_bench['type'] == 5:
            # 如果材料1空缺
            if not work_bench['material_state'] & 2:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
            # 如果材料3空缺
            if not work_bench['material_state'] & 8:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
        # 类型6工作台，收购2和3
        if work_bench['type'] == 6:
            # 如果材料2空缺
            if not work_bench['material_state'] & 4:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
            # 如果材料3空缺
            if not work_bench['material_state'] & 8:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
        # 类型7工作台，收购4，5，6
        if work_bench['type'] == 7:
            # 如果材料4空缺
            if not work_bench['material_state'] & 16:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 4})
            # 如果材料5空缺
            if not work_bench['material_state'] & 32:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 5})
            # 如果材料6空缺
            if not work_bench['material_state'] & 64:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 6})
        # 类型8工作台，收购7
        if work_bench['type'] == 8:
            candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 7})

    # 构造所有的送货策略组合，格式为字典{取货点工作台id（departure_work_bench_id），送货点工作台id(destination_work_bench_id)，
    # 配送货物类型编号(product_type,1-7)，收益(profit),距离（distance）
    # 潜在收益（potential_profit,送货点工作台的产物可能产生的收益的一部分作为潜在收益，4，5，6号工作台产物收益的一半作为潜在收益，7号工作台产物收益的1/3作为潜在收益）}
    # 获得潜在收益需要多跑的距离（distance_for_potential_profit）
    distribution_strategies = []
    for departure in candidate_buy_destinations:
        for destination in candidate_sell_destinations:
            if departure['product_type'] == destination['material_requested']:
                # 起点坐标
                departure_xy = np.array([work_bench_list[departure['work_bench_id']]['x'],
                                         work_bench_list[departure['work_bench_id']]['y']])
                destination_xy = np.array([work_bench_list[destination['work_bench_id']]['x'],
                                           work_bench_list[destination['work_bench_id']]['y']])
                distance = np.linalg.norm(departure_xy - destination_xy)
                profit = product_profits[departure['product_type'] - 1]
                potential_profit = 0
                distance_for_potential_profit = 0
                # 对于4，只要生产没有被阻塞就鼓励生产
                if work_bench_list[destination['work_bench_id']]['type'] == 4 and \
                        work_bench_list[destination['work_bench_id']]['produce_remain_time'] != 0:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']][
                                               'type'] - 1] / 2
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']
                # 对于5，产品格没有东西就鼓励生产
                if work_bench_list[destination['work_bench_id']]['type'] == 5 and \
                        work_bench_list[destination['work_bench_id']]['product_state'] != 1:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']]['type'] - 1] / 2
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']
                # 对于6，产品格没有东西就鼓励生产
                if work_bench_list[destination['work_bench_id']]['type'] == 6 and \
                        work_bench_list[destination['work_bench_id']]['product_state'] != 1:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']][
                                               'type'] - 1] / 2 * stimulate_factor_for_product_6
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']
                # 对于7 只要生产没阻塞就鼓励生产
                if work_bench_list[destination['work_bench_id']]['type'] == 7 and \
                        work_bench_list[destination['work_bench_id']]['produce_remain_time'] != 0:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']]['type'] - 1] / 3 * 1.2
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']
                if work_bench_list[destination['work_bench_id']]['type'] == 8:
                    potential_profit = 0.5 * profit
                distribution_strategies.append(
                    {'departure_work_bench_id': departure['work_bench_id'],
                     'destination_work_bench_id': destination['work_bench_id'],
                     'product_type': departure['product_type'],
                     'profit': product_profits[departure['product_type'] - 1],
                     'potential_profit': potential_profit,
                     'distance': distance,
                     'distance_for_potential_profit': distance_for_potential_profit})

    # 若机器人R当前任务是去A工作台取货，且有另一个机器人去A工作台送货，则机器人R应该立即放弃当前任务
    for robot in robot_list:
        if strategies_of_robots[robot['id']] == {}:
            continue
        for other_robot in robot_list:
            if robot['id'] == other_robot['id'] or strategies_of_robots[other_robot['id']] == {}:
                continue
            else:
                # print(str(strategies_of_robots[robot['id']]) + "--------------------------------", file=sys.stderr)
                if strategies_of_robots[other_robot['id']]['carried_product_type'] != 0 and \
                        strategies_of_robots[other_robot['id']]['destination_work_bench_id'] == \
                        strategies_of_robots[robot['id']]['departure_work_bench_id'] and \
                        strategies_of_robots[robot['id']]['carried_product_type'] == 0:
                    strategies_of_robots[robot['id']] = {}
                    break

    # 为每个机器人找出前N个平均每米收益最大的方案
    top_n_strategies_for_robots = []
    for robot in robot_list:
        # top_n_strategies_for_this_robot内容格式为{取货点工作台序号(departure_work_bench_id)，送货点工作台序号(destination_work_bench_id)，
        # 配送货物类型编号(product_type)，平均每米收益(profit_per_meter)}
        top_n_strategies_for_this_robot = []
        if strategies_of_robots[robot['id']] != {}:
            top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)
            continue
        robot_xy = np.array([robot['x'], robot['y']])
        for strategy in distribution_strategies:
            departure_xy = np.array([work_bench_list[strategy['departure_work_bench_id']]['x'],
                                     work_bench_list[strategy['departure_work_bench_id']]['y']])
            distance_from_robot_to_departure = np.linalg.norm(robot_xy - departure_xy)
            distance_from_departure_to_destination = strategy['distance']
            # 如果当前任务在游戏结束前无法完成（机器人理论需要跑动的距离*延误系数>剩余时间以最快速度最多能跑的距离），则直接pass，
            if (distance_from_robot_to_departure + distance_from_departure_to_destination) * 1.5 > (
                    9000 - frame_id) * 0.12:
                continue
            profit_per_meter = (strategy['profit'] + strategy['potential_profit']) / (
                    distance_from_robot_to_departure + distance_from_departure_to_destination + strategy[
                'distance_for_potential_profit'])

            # 插入排序，寻找插入点
            insert_index = len(top_n_strategies_for_this_robot)
            for top_50_strategy in top_n_strategies_for_this_robot:
                if top_50_strategy['profit_per_meter'] <= profit_per_meter:
                    insert_index = top_n_strategies_for_this_robot.index(top_50_strategy)
                    break
            if insert_index <= top_N - 1:
                top_n_strategies_for_this_robot.insert(insert_index,
                                                       {'departure_work_bench_id': strategy['departure_work_bench_id'],
                                                        'destination_work_bench_id': strategy[
                                                            'destination_work_bench_id'],
                                                        'product_type': strategy['product_type'],
                                                        'profit_per_meter': profit_per_meter})

        top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)

    for robot in robot_list:  # 为每一个机器人寻找方案
        if strategies_of_robots[robot['id']] != {}:
            continue
        for strategy in top_n_strategies_for_robots[robot['id']]:  # 遍历该机器人的待选方案
            rank = top_n_strategies_for_robots[robot['id']].index(strategy) + 1  # 这是平均每米收益第rank大的方案
            flag = False  # 该方案与其他机器人已选方案是否冲突
            for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
                if selected_strategy == {}:
                    continue
                if (selected_strategy['departure_work_bench_id'] == strategy['departure_work_bench_id'] and
                    selected_strategy['product_type'] == strategy['product_type'] and selected_strategy[
                        'carried_product_type'] == 0) or (
                        selected_strategy['product_type'] == strategy['product_type'] and
                        selected_strategy['destination_work_bench_id'] == strategy['destination_work_bench_id']):
                    flag = True
                    break
            if not flag:
                if strategies_of_robots[robot['id']] == {}:
                    strategies_of_robots[robot['id']] = strategy
                    strategies_of_robots[robot['id']]['carried_product_type'] = 0
                elif work_bench_list[strategy['destination_work_bench_id']]['material_state'] > 0:
                    strategies_of_robots[robot['id']] = strategy
                    strategies_of_robots[robot['id']]['carried_product_type'] = 0

                # 如果平均每米收益最大方案不会去已经有部分原材料的工作台送货，最多向后搜索到第3收益最大的方案
                if rank >= 6 or work_bench_list[strategy['destination_work_bench_id']]['material_state'] > 0:
                    break

    return strategies_of_robots


def strategy_greedy_for_map_3(work_bench_list, robot_list, strategies_of_robots, frame_id, nearest_sell_place):
    top_N = 15  # 取前top_N个最好的策略

    # 候选购买材料目的地,格式为{工作台id(work_bench_id)，生产的物品类型(product_type)，剩余生产时间(produce_remain_time)}
    candidate_buy_destinations = []

    # 候选销售产品目的地,格式为{工作台id（work_bench_id），需要的物品类型(material_requested), 预计多少时间后此原料格空出（未实现）}
    candidate_sell_destinations = []

    # 找出候选购买材料目的地和候选销售产品目的地
    for work_bench in work_bench_list:
        # 工作台id
        if work_bench['product_state'] == 1:
            candidate_buy_destinations.append(
                {'work_bench_id': work_bench['id'], 'product_type': work_bench['type'], 'produce_remain_time': 0})
        # 由于不考虑剩余生产时间分数更高，所以只在第一帧时考虑剩余时间，以免前50帧机器人静止不动
        elif work_bench['produce_remain_time'] > 0 and frame_id == 1:
            candidate_buy_destinations.append({'work_bench_id': work_bench['id'], 'product_type': work_bench['type'],
                                               'produce_remain_time': work_bench['produce_remain_time']})

        # 00000010=>2 对应材料1
        # 00000100=>4 对应材料2
        # 00001000=>8 对应材料3
        # 00010000=>16 对应材料4
        # 00100000=>32 对应材料5
        # 01000000=>64 对应材料6
        # 10000000=>128 对应材料7

        # 类型4工作台，收购1和2
        if work_bench['type'] == 4:
            # 如果材料1空缺
            if not work_bench['material_state'] & 2:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
            # 如果材料2空缺
            if not work_bench['material_state'] & 4:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
        # 类型5工作台，收购1和3
        if work_bench['type'] == 5:
            # 如果材料1空缺
            if not work_bench['material_state'] & 2:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
            # 如果材料3空缺
            if not work_bench['material_state'] & 8:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
        # 类型6工作台，收购2和3
        if work_bench['type'] == 6:
            # 如果材料2空缺
            if not work_bench['material_state'] & 4:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
            # 如果材料3空缺
            if not work_bench['material_state'] & 8:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
        # 类型7工作台，收购4，5，6
        if work_bench['type'] == 7:
            # 如果材料4空缺
            if not work_bench['material_state'] & 16:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 4})
            # 如果材料5空缺
            if not work_bench['material_state'] & 32:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 5})
            # 如果材料6空缺
            if not work_bench['material_state'] & 64:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 6})
        # 类型8工作台，收购7
        if work_bench['type'] == 8:
            candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 7})
        # 类型9工作台，收购1-7
        if work_bench['type'] == 9:
            for i in range(4, 7):
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': i})

    # 构造所有的送货策略组合，格式为字典{取货点工作台id（departure_work_bench_id），送货点工作台id(destination_work_bench_id)，
    # 配送货物类型编号(product_type,1-7)，收益(profit),距离（distance）
    # 潜在收益（potential_profit,送货点工作台的产物可能产生的收益的一部分作为潜在收益，4，5，6号工作台产物收益的一半作为潜在收益，7号工作台产物收益的1/3作为潜在收益）}
    # 获得潜在收益需要多跑的距离（distance_for_potential_profit）
    distribution_strategies = []
    for departure in candidate_buy_destinations:
        for destination in candidate_sell_destinations:
            if departure['product_type'] == destination['material_requested']:
                # 起点坐标
                departure_xy = np.array([work_bench_list[departure['work_bench_id']]['x'],
                                         work_bench_list[departure['work_bench_id']]['y']])
                destination_xy = np.array([work_bench_list[destination['work_bench_id']]['x'],
                                           work_bench_list[destination['work_bench_id']]['y']])
                distance = np.linalg.norm(departure_xy - destination_xy)
                potential_profit = 0
                distance_for_potential_profit = 0
                # # 只有目标送货工作台产品格没有物品且不在生产中才能获得潜在收益
                if 4 <= work_bench_list[destination['work_bench_id']]['type'] <= 6 and \
                        nearest_sell_place[destination['work_bench_id']]['id'] != -1 and \
                        work_bench_list[destination['work_bench_id']]['produce_remain_time'] != 0:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']]['type'] - 1] / 2
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']
                distribution_strategies.append(
                    {'departure_work_bench_id': departure['work_bench_id'],
                     'destination_work_bench_id': destination['work_bench_id'],
                     'product_type': departure['product_type'],
                     'profit': product_profits[departure['product_type'] - 1],
                     'potential_profit': potential_profit,
                     'distance': distance,
                     'distance_for_potential_profit': distance_for_potential_profit})

    # 若机器人R当前任务是去A工作台取货，且有另一个机器人去A工作台送货，则机器人R应该立即放弃当前任务
    for robot in robot_list:
        if strategies_of_robots[robot['id']] == {}:
            continue
        for other_robot in robot_list:
            if robot['id'] == other_robot['id'] or strategies_of_robots[other_robot['id']] == {}:
                continue
            else:
                if strategies_of_robots[other_robot['id']]['carried_product_type'] != 0 and \
                        strategies_of_robots[other_robot['id']]['destination_work_bench_id'] == \
                        strategies_of_robots[robot['id']]['departure_work_bench_id'] and \
                        strategies_of_robots[robot['id']]['carried_product_type'] == 0:
                    strategies_of_robots[robot['id']] = {}
                    break

    # 为每个机器人找出前N个平均每米收益最大的方案
    top_n_strategies_for_robots = []
    for robot in robot_list:
        # top_n_strategies_for_this_robot内容格式为{取货点工作台序号(departure_work_bench_id)，送货点工作台序号(destination_work_bench_id)，
        # 配送货物类型编号(product_type)，平均每米收益(profit_per_meter)}
        top_n_strategies_for_this_robot = []
        if strategies_of_robots[robot['id']] != {}:
            top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)
            continue
        robot_xy = np.array([robot['x'], robot['y']])
        for strategy in distribution_strategies:
            departure_xy = np.array([work_bench_list[strategy['departure_work_bench_id']]['x'],
                                     work_bench_list[strategy['departure_work_bench_id']]['y']])
            distance_from_robot_to_departure = np.linalg.norm(robot_xy - departure_xy)
            distance_from_departure_to_destination = strategy['distance']
            # 如果当前任务在游戏结束前无法完成（机器人理论需要跑动的距离*延误系数>剩余时间以最快速度最多能跑的距离），则直接pass，
            if (distance_from_robot_to_departure + distance_from_departure_to_destination) * 1.5 > (
                    9000 - frame_id) * 0.12:
                continue
            profit_per_meter = (strategy['profit'] + strategy['potential_profit']) / (
                    distance_from_robot_to_departure + distance_from_departure_to_destination + strategy[
                'distance_for_potential_profit'])

            # 插入排序，寻找插入点
            insert_index = len(top_n_strategies_for_this_robot)
            for top_50_strategy in top_n_strategies_for_this_robot:
                if top_50_strategy['profit_per_meter'] <= profit_per_meter:
                    insert_index = top_n_strategies_for_this_robot.index(top_50_strategy)
                    break
            if insert_index <= top_N - 1:
                top_n_strategies_for_this_robot.insert(insert_index,
                                                       {'departure_work_bench_id': strategy['departure_work_bench_id'],
                                                        'destination_work_bench_id': strategy[
                                                            'destination_work_bench_id'],
                                                        'product_type': strategy['product_type'],
                                                        'profit_per_meter': profit_per_meter})

        top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)

    for robot in robot_list:  # 为每一个机器人寻找方案
        if strategies_of_robots[robot['id']] != {}:
            continue
        for strategy in top_n_strategies_for_robots[robot['id']]:  # 遍历该机器人的待选方案
            rank = top_n_strategies_for_robots[robot['id']].index(strategy) + 1  # 这是平均每米收益第rank大的方案
            #     continue
            flag = False  # 该方案与其他机器人已选方案是否冲突
            for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
                if selected_strategy == {}:
                    continue
                if (selected_strategy['departure_work_bench_id'] == strategy['departure_work_bench_id'] and
                    selected_strategy['product_type'] == strategy['product_type'] and selected_strategy[
                        'carried_product_type'] == 0) or (
                        selected_strategy['product_type'] == strategy['product_type'] and
                        selected_strategy['destination_work_bench_id'] == strategy['destination_work_bench_id'] and
                        work_bench_list[strategy['destination_work_bench_id']]['type'] != 9):
                    flag = True
                    break
            if not flag:
                if strategies_of_robots[robot['id']] == {}:
                    strategies_of_robots[robot['id']] = strategy
                    strategies_of_robots[robot['id']]['carried_product_type'] = 0
                elif work_bench_list[strategy['destination_work_bench_id']]['material_state'] > 0:
                    strategies_of_robots[robot['id']] = strategy
                    strategies_of_robots[robot['id']]['carried_product_type'] = 0

                # 如果平均每米收益最大方案不会去已经有部分原材料的工作台送货，最多向后搜索到第3收益最大的方案
                if rank >= 8 or work_bench_list[strategy['destination_work_bench_id']]['material_state'] > 0:
                    break

    return strategies_of_robots


def strategy_greedy_for_map_4(work_bench_list, robot_list, strategies_of_robots, frame_id, nearest_sell_place,
                              stimulate_factor_for_product_4):
    top_N = 20

    # 候选购买材料目的地,格式为{工作台id(work_bench_id)，生产的物品类型(product_type)，剩余生产时间(produce_remain_time)}
    candidate_buy_destinations = []

    # 候选销售产品目的地,格式为{工作台id（work_bench_id），需要的物品类型(material_requested)}
    candidate_sell_destinations = []

    # 找出候选购买材料目的地和候选销售产品目的地
    for work_bench in work_bench_list:
        # 工作台id
        if work_bench['product_state'] == 1:
            candidate_buy_destinations.append(
                {'work_bench_id': work_bench['id'], 'product_type': work_bench['type'], 'produce_remain_time': 0})
        # 由于不考虑剩余生产时间分数更高，所以只在第一帧时考虑剩余时间，以免前50帧机器人静止不动
        elif work_bench['produce_remain_time'] > 0 and frame_id == 1:
            candidate_buy_destinations.append({'work_bench_id': work_bench['id'], 'product_type': work_bench['type'],
                                               'produce_remain_time': work_bench['produce_remain_time']})

        # 00000010=>2 对应材料1
        # 00000100=>4 对应材料2
        # 00001000=>8 对应材料3
        # 00010000=>16 对应材料4
        # 00100000=>32 对应材料5
        # 01000000=>64 对应材料6
        # 10000000=>128 对应材料7

        # 类型4工作台，收购1和2
        if work_bench['type'] == 4:
            # 如果材料1空缺
            if not work_bench['material_state'] & 2:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
            # 如果材料2空缺
            if not work_bench['material_state'] & 4:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
        # 类型5工作台，收购1和3
        if work_bench['type'] == 5:
            # 如果材料1空缺
            if not work_bench['material_state'] & 2:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 1})
            # 如果材料3空缺
            if not work_bench['material_state'] & 8:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
        # 类型6工作台，收购2和3
        if work_bench['type'] == 6:
            # 如果材料2空缺
            if not work_bench['material_state'] & 4:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 2})
            # 如果材料3空缺
            if not work_bench['material_state'] & 8:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 3})
        # 类型7工作台，收购4，5，6
        if work_bench['type'] == 7:
            # 如果材料4空缺
            if not work_bench['material_state'] & 16:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 4})
            # 如果材料5空缺
            if not work_bench['material_state'] & 32:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 5})
            # 如果材料6空缺
            if not work_bench['material_state'] & 64:
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 6})
        # 类型8工作台，收购7
        if work_bench['type'] == 8:
            candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': 7})
        # 类型9工作台，收购1-7
        if work_bench['type'] == 9:
            for i in range(7):
                candidate_sell_destinations.append({'work_bench_id': work_bench['id'], 'material_requested': i + 1})

    # 构造所有的送货策略组合，格式为字典{取货点工作台id（departure_work_bench_id），送货点工作台id(destination_work_bench_id)，
    # 配送货物类型编号(product_type,1-7)，收益(profit),距离（distance）
    # 潜在收益（potential_profit,送货点工作台的产物可能产生的收益的一部分作为潜在收益，4，5，6号工作台产物收益的一半作为潜在收益，7号工作台产物收益的1/3作为潜在收益）}
    # 获得潜在收益需要多跑的距离（distance_for_potential_profit）
    distribution_strategies = []
    for departure in candidate_buy_destinations:
        for destination in candidate_sell_destinations:
            if departure['product_type'] == destination['material_requested']:
                # 起点坐标
                departure_xy = np.array([work_bench_list[departure['work_bench_id']]['x'],
                                         work_bench_list[departure['work_bench_id']]['y']])
                destination_xy = np.array([work_bench_list[destination['work_bench_id']]['x'],
                                           work_bench_list[destination['work_bench_id']]['y']])
                distance = np.linalg.norm(departure_xy - destination_xy)
                profit = product_profits[departure['product_type'] - 1]
                potential_profit = 0
                distance_for_potential_profit = 0
                # 只有目标送货工作台生产且没有被阻塞才激励
                if work_bench_list[destination['work_bench_id']]['type'] == 4 and \
                        nearest_sell_place[destination['work_bench_id']]['id'] != -1 and \
                        work_bench_list[destination['work_bench_id']]['produce_remain_time'] != 0:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']][
                                               'type'] - 1] / 2 * stimulate_factor_for_product_4
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']

                if work_bench_list[destination['work_bench_id']]['type'] == 5 and \
                        nearest_sell_place[destination['work_bench_id']]['id'] != -1 and \
                        work_bench_list[destination['work_bench_id']]['produce_remain_time'] != 0:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']]['type'] - 1] / 2
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']

                if work_bench_list[destination['work_bench_id']]['type'] == 6 and \
                        nearest_sell_place[destination['work_bench_id']]['id'] != -1 and \
                        work_bench_list[destination['work_bench_id']]['produce_remain_time'] != 0:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']]['type'] - 1] / 2
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']

                if work_bench_list[destination['work_bench_id']]['type'] == 7 and \
                        nearest_sell_place[destination['work_bench_id']]['id'] != -1 and \
                        work_bench_list[destination['work_bench_id']]['produce_remain_time'] != 0:
                    potential_profit = product_profits[
                                           work_bench_list[destination['work_bench_id']]['type'] - 1] / 3
                    distance_for_potential_profit = nearest_sell_place[destination['work_bench_id']]['distance']
                if work_bench_list[destination['work_bench_id']]['type'] == 8:
                    potential_profit = 0.5 * profit
                distribution_strategies.append(
                    {'departure_work_bench_id': departure['work_bench_id'],
                     'destination_work_bench_id': destination['work_bench_id'],
                     'product_type': departure['product_type'],
                     'profit': product_profits[departure['product_type'] - 1],
                     'potential_profit': potential_profit,
                     'distance': distance,
                     'distance_for_potential_profit': distance_for_potential_profit})

    # 若机器人R当前任务是去A工作台取货，且有另一个机器人去A工作台送货，则机器人R应该立即放弃当前任务
    for robot in robot_list:
        if strategies_of_robots[robot['id']] == {}:
            continue
        for other_robot in robot_list:
            if robot['id'] == other_robot['id'] or strategies_of_robots[other_robot['id']] == {}:
                continue
            else:
                # print(str(strategies_of_robots[robot['id']]) + "--------------------------------", file=sys.stderr)
                if strategies_of_robots[other_robot['id']]['carried_product_type'] != 0 and \
                        strategies_of_robots[other_robot['id']]['destination_work_bench_id'] == \
                        strategies_of_robots[robot['id']]['departure_work_bench_id'] and \
                        strategies_of_robots[robot['id']]['carried_product_type'] == 0:
                    strategies_of_robots[robot['id']] = {}
                    break

    # 为每个机器人找出前N个平均每米收益最大的方案
    top_n_strategies_for_robots = []
    for robot in robot_list:
        # top_n_strategies_for_this_robot内容格式为{取货点工作台序号(departure_work_bench_id)，送货点工作台序号(destination_work_bench_id)，
        # 配送货物类型编号(product_type)，平均每米收益(profit_per_meter)}
        top_n_strategies_for_this_robot = []
        if strategies_of_robots[robot['id']] != {}:
            top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)
            continue
        robot_xy = np.array([robot['x'], robot['y']])
        for strategy in distribution_strategies:
            departure_xy = np.array([work_bench_list[strategy['departure_work_bench_id']]['x'],
                                     work_bench_list[strategy['departure_work_bench_id']]['y']])
            distance_from_robot_to_departure = np.linalg.norm(robot_xy - departure_xy)
            distance_from_departure_to_destination = strategy['distance']
            # 如果当前任务在游戏结束前无法完成（机器人理论需要跑动的距离*延误系数>剩余时间以最快速度最多能跑的距离），则直接pass，
            if (distance_from_robot_to_departure + distance_from_departure_to_destination) * 1.5 > (
                    9000 - frame_id) * 0.12:
                continue
            profit_per_meter = (strategy['profit'] + strategy['potential_profit']) / (
                    distance_from_robot_to_departure + distance_from_departure_to_destination + strategy[
                'distance_for_potential_profit'])

            # 插入排序，寻找插入点
            insert_index = len(top_n_strategies_for_this_robot)
            for top_50_strategy in top_n_strategies_for_this_robot:
                if top_50_strategy['profit_per_meter'] <= profit_per_meter:
                    insert_index = top_n_strategies_for_this_robot.index(top_50_strategy)
                    break
            if insert_index <= top_N - 1:
                top_n_strategies_for_this_robot.insert(insert_index,
                                                       {'departure_work_bench_id': strategy['departure_work_bench_id'],
                                                        'destination_work_bench_id': strategy[
                                                            'destination_work_bench_id'],
                                                        'product_type': strategy['product_type'],
                                                        'profit_per_meter': profit_per_meter})

        top_n_strategies_for_robots.append(top_n_strategies_for_this_robot)

    for robot in robot_list:  # 为每一个机器人寻找方案
        if strategies_of_robots[robot['id']] != {}:
            continue
        for strategy in top_n_strategies_for_robots[robot['id']]:  # 遍历该机器人的待选方案
            rank = top_n_strategies_for_robots[robot['id']].index(strategy) + 1  # 这是平均每米收益第rank大的方案
            flag = False  # 该方案与其他机器人已选方案是否冲突
            for selected_strategy in strategies_of_robots:  # 判断该待选方案是否和其他机器人已选择方案有冲突
                if selected_strategy == {}:
                    continue
                if (selected_strategy['departure_work_bench_id'] == strategy['departure_work_bench_id'] and
                    selected_strategy['product_type'] == strategy['product_type'] and selected_strategy[
                        'carried_product_type'] == 0) or (
                        selected_strategy['product_type'] == strategy['product_type'] and
                        selected_strategy['destination_work_bench_id'] == strategy['destination_work_bench_id']):
                    flag = True
                    break
            if not flag:
                if strategies_of_robots[robot['id']] == {}:
                    strategies_of_robots[robot['id']] = strategy
                    strategies_of_robots[robot['id']]['carried_product_type'] = 0
                elif work_bench_list[strategy['destination_work_bench_id']]['material_state'] > 0:
                    strategies_of_robots[robot['id']] = strategy
                    strategies_of_robots[robot['id']]['carried_product_type'] = 0

                # 如果平均每米收益最大方案不会去已经有部分原材料的工作台送货，最多向后搜索到第3收益最大的方案
                if rank >= 6 or work_bench_list[strategy['destination_work_bench_id']]['material_state'] > 0:
                    break

    return strategies_of_robots
