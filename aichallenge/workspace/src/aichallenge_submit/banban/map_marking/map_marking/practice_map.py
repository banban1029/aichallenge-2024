import pandas as pd

# CSVファイルのパス
csv_file = 'mademap/ai_challenge_smoothed.csv'



# CSVファイルを読み込む
df = pd.read_csv(csv_file)

# 1列目と2列目を取得し、小数第一位までの値に変換する
column1_rounded = df.iloc[:, 0].round(1)  # 1列目を小数第一位まで丸める
column2_rounded = df.iloc[:, 1].round(1)  # 2列目を小数第一位まで丸める

# 新しい列として追加するデータ（例として、すべての行に同じ値を設定する）
column3_value = 3  # 3列目に設定する値を指定する（ここでは例として5を設定）

# 新しいDataFrameを作成して、丸められたデータと新しい列を追加する
df_updated = pd.DataFrame({
    'positionx': column1_rounded,
    'positiony': column2_rounded,
    'longitudinal_velocity_mps': column3_value
})

# 更新されたデータを出力する（CSVファイルとして保存する場合）
output_file = 'updated_output.csv'
df_updated.to_csv(output_file, index=False)  # index=Falseで行番号を保存しないようにする

# 更新されたデータの先頭を表示する
print(df_updated.head())
