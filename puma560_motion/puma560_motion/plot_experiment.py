"""
Строит графики по CSV-логу эксперимента.
Запуск:
    python3 plot_experiment.py /root/workspace_saved/experiments/log.csv
"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')  # без GUI - сохраняем сразу в файл
import matplotlib.pyplot as plt


def load_log(path):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def plot_speed_vs_distance(rows, out_path):
    xs, ys = [], []
    for r in rows:
        if r['min_distance'] and r['speed_scaling']:
            xs.append(float(r['min_distance']))
            ys.append(float(r['speed_scaling']))

    plt.figure(figsize=(7, 5))
    plt.scatter(xs, ys, s=8, alpha=0.5)
    plt.xlabel('Дистанция до человека, м')
    plt.ylabel('Коэффициент скорости')
    plt.title('Скорость робота в зависимости от дистанции')
    plt.grid(True)
    plt.savefig(out_path, dpi=150)
    print(f'Saved: {out_path}')


def plot_reaction_time(rows, out_path):
    times = [
        float(r['reaction_time_s']) * 1000.0  # в мс
        for r in rows if r['reaction_time_s']
    ]

    if not times:
        print('No reaction_time samples found - skipping histogram')
        return

    plt.figure(figsize=(7, 5))
    plt.hist(times, bins=20, edgecolor='black')
    plt.xlabel('Время реакции, мс')
    plt.ylabel('Количество событий')
    plt.title(f'Распределение времени реакции (n={len(times)})')
    plt.grid(True)
    plt.savefig(out_path, dpi=150)
    print(f'Saved: {out_path}')

    print(f'Mean reaction time: {sum(times)/len(times):.2f} ms')
    print(f'Min: {min(times):.2f} ms, Max: {max(times):.2f} ms')


if __name__ == '__main__':
    csv_path = sys.argv[1] if len(sys.argv) > 1 else '/root/workspace_saved/experiments/log.csv'
    rows = load_log(csv_path)
    print(f'Loaded {len(rows)} rows from {csv_path}')

    base = csv_path.rsplit('.', 1)[0]
    plot_speed_vs_distance(rows, base + '_speed_vs_distance.png')
    plot_reaction_time(rows, base + '_reaction_time.png')
