from ultralytics import YOLO

# モデル読み込み
model = YOLO("C:/Users/hiyos/OneDrive - 埼玉大学/あかとき/プログラム/SC-21/SC-21_yolomodel_v1.pt")

# 入力画像
results = model("/path",save=True)

# エクスポート.
# model.export()
