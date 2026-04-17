import matplotlib.pyplot as plt

# 从文件中读取数据
file_path = 'res'  # 替换成你的文件路径

x = []
y = []
with open(file_path, 'r') as file:
    for line in file:
        data = line.split()
        x.append(int(data[0]))
        y.append(float(data[1]))

# 绘制曲线图
plt.plot(x, y)
plt.xlabel('Time')
plt.ylabel('Value', rotation = 0)
plt.xticks(range(0, max(x) + 1, 100))  # 设置x轴刻度从0开始，每隔2个单位显示一个刻度


# 设置坐标轴范围
plt.axis([0, max(x), 0, 5000])  # 指定x轴和y轴的范围为从0开始到最大值
plt.grid(True)
plt.show()
