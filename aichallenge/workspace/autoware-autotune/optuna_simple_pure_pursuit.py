import optuna
import subprocess
import os
import json
import numpy as np
import psutil

def objective(trial):
    # Optunaによって最適化するパラメータを定義
    external_target_vel = trial.suggest_uniform('external_target_vel', 0.1, 15.0)
    lookahead_gain = trial.suggest_uniform('lookahead_gain', 0.1, 1.5)
    lookahead_min_distance = trial.suggest_uniform('lookahead_min_distance', 0.1, 15.0)
    speed_proportional_gain = trial.suggest_uniform('speed_proportional_gain', 0.1, 2.5)

    # 最適化されたパラメータを使ってrun_evaluation.bashを起動する
    command = ['/bin/bash', '/aichallenge/run_evaluation.bash']
    env = os.environ.copy()
    env['EXTERNAL_TARGET_VEL'] = str(external_target_vel)
    env['LOOKAHEAD_GAIN'] = str(lookahead_gain)
    env['LOOKAHEAD_MIN_DISTANCE'] = str(lookahead_min_distance)
    env['SPEED_PROPORTIONAL_GAIN'] = str(speed_proportional_gain)

    process = subprocess.Popen(command, env=env)

    # 評価プロセスが終了するまで待つ（評価時間に応じてタイムアウトを調整）
    process.wait(timeout=3600)  # 評価時間に応じて適宜調整する

    # ノードをキルする
    subprocess.run("ps aux | grep ros | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh", shell=True)

    # スコアを計算する
    try:
        with open('/output/latest/result-summary.json') as f:
            result = json.load(f)
            print(result)
            if result["laps"] == []:
                score = 999.0
            elif len(result["laps"]) < 6:
                score = 900.0
            else:
                score = np.sum(result['laps'])

    except Exception as e:
        print(f"Error reading result file: {e}")
        score = 999.0

    return score

def main():
    study = optuna.create_study(direction='minimize', study_name='Autoware-turning-study', storage='sqlite:///Autoware-turning-study.db', load_if_exists=True)
    study.optimize(objective, n_trials=100)

    print('Best parameters:', study.best_params)
    print('Best score:', study.best_value)

if __name__ == '__main__':
    main()
