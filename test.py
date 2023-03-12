import sys


work_bench = [] #下面K行为工作站信息
for i in range(2):
    line = sys.stdin.readline().strip().split(' ')
    work_bench.append({'id':int(line[0]), 'x':float(line[1]), 'y':float(line[2]), 'produce_remain_time':int((line[3])), 'raw_state':int(line[4]), \
                    'product_state':int(line[5]) })
    
print(work_bench)