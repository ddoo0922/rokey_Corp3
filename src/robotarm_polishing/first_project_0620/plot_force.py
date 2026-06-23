import csv
import matplotlib.pyplot as plt
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
log_file_path = os.path.join(_SCRIPT_DIR, "force_log.csv")

if not os.path.exists(log_file_path):
    print(f"데이터 파일이 없습니다: {log_file_path}")
    print("먼저 run_polishing_sim_3.py 를 실행하여 데이터를 생성해주세요.")
    exit()

steps = []
forces = []

with open(log_file_path, "r") as f:
    reader = csv.reader(f)
    next(reader)  # 헤더 스킵
    for row in reader:
        if len(row) == 2:
            try:
                steps.append(int(row[0]))
                forces.append(float(row[1]))
            except ValueError:
                pass

plt.figure(figsize=(10, 5))
plt.plot(steps, forces, label='Contact Force (N)', color='b', linewidth=2)

plt.title('Polishing Contact Force over Time')
plt.xlabel('Simulation Steps')
plt.ylabel('Force (N)')
plt.grid(True)
plt.legend()

plot_path = os.path.join(_SCRIPT_DIR, "force_plot.png")
plt.savefig(plot_path)
print(f"그래프가 저장되었습니다: {plot_path}")

plt.show()
