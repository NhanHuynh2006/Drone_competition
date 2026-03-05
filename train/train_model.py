#!/usr/bin/env python3
"""Training script (kept as Python - same as original)
Export to NCNN: yolo export model=best.pt format=ncnn
Then copy .param + .bin to models/"""
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
results = model.train(
    data='dataset/data.yaml',
    epochs=100,
    imgsz=320,
    batch=16,
    name='drone_hoops',
    classes=[0,1,2,3],  # hoop_red, hoop_yellow, hoop_blue, landing_pad
)
# Export
model = YOLO('runs/detect/drone_hoops/weights/best.pt')
model.export(format='ncnn')
print("Done! Copy ncnn model to ../models/")
