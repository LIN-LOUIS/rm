import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV 文件
df = pd.read_csv("/home/lin/Desktop/卡尔曼波//results/fitted_results4.csv")

# 绘制原始数据和卡尔曼滤波拟合数据
plt.figure(figsize=(10, 5))
plt.plot(df["Time"], df["Measured"], label="Measured",linestyle="-", color="blue")
plt.plot(df["Time"], df["Fitted"], label="Fitted (Kalman)", linestyle="-", color="red")

# 设置标题和标签
plt.xlabel("Time")
plt.ylabel("Value")
plt.title("Kalman Filter Curve Fitting")
plt.legend()
plt.grid()

# 显示图表
plt.show()
