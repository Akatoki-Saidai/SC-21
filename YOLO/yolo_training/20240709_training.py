from ultralytics import YOLO

# クラス数1のYOLOv10nのyamlファイルをロード
# model = YOLO("C:/Users/hiyos/OneDrive - 埼玉大学/あかとき/プログラム/SC-21/dataset/class1_yolov10n.yaml")

# YOLOv10nのptファイルをロード
model = YOLO("yolov10n.pt")

# モデルをyamlファイルでトレーニング
results = model.train(data="C:/Users/hiyos/OneDrive - 埼玉大学/あかとき/プログラム/SC-21/dataset/20240709_training.yaml", epochs=100, imgsz=640)

# モデルをエクスポート
# model.save("yolo_20240709_training.pt")




