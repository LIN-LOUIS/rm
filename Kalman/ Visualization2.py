import pandas as pd
import matplotlib.pyplot as plt

# 读取 C++ 导出的数据
df = pd.read_csv("/home/lin/Desktop/卡尔曼波/results/prediction_output.csv")

# 绘图
plt.figure(figsize=(16, 7))
plt.plot(df['Day'], df['Observed'], label='Observed Price', marker='o')
plt.plot(df['Day'], df['Predicted'], label='Kalman Predicted', linestyle='--', color='red')
plt.xlabel("Day")
plt.ylabel("Stock Price")
plt.title("Stock Price Prediction using Kalman Filter (C++)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
