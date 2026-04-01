import os
import time
import json
import numpy as np
import cv2
import matplotlib.pyplot as plt
import torch

from yolox.exp import get_exp
from yolox.utils import fuse_model, postprocess
from yolox.data.data_augment import ValTransform
from yolox.data.datasets import COCODataset

from pycocotools.cocoeval import COCOeval

# -----------------------------
# CONFIG
# -----------------------------
EXP_FILE = "exps/example/custom/yolox_nano_pallet.py"
CKPT = "/home/gputesting/YOLO_pose_alternative_models/YOLOX/YOLOX/YOLOX_outputs/yolox_nano_pallet/best_ckpt.pth"

DATA_DIR = "/home/gputesting/YOLO_pose_alternative_models/dataset/all_data_coco_format"
ANN_FILE = "pallet_test.json"
IMG_DIR = "images/test2017"

DEVICE = "cuda"

# 🔥 FIXED THRESHOLDS
CONFTHRE = 0.4
NMSTHRE = 0.45

TEST_SIZE = (416, 416)

SAVE_DIR = "test_outputs"
os.makedirs(SAVE_DIR, exist_ok=True)

# -----------------------------
# LOAD MODEL
# -----------------------------
exp = get_exp(EXP_FILE, None)
model = exp.get_model()

ckpt = torch.load(CKPT, map_location="cpu")
model.load_state_dict(ckpt["model"])
model.eval()

if DEVICE == "cuda":
    model.cuda()

model = fuse_model(model)

# -----------------------------
# DATASET
# -----------------------------
dataset = COCODataset(
    data_dir=DATA_DIR,
    json_file=ANN_FILE,
    name=IMG_DIR,
    img_size=TEST_SIZE,
    preproc=ValTransform(legacy=False),
)

# -----------------------------
# IOU FUNCTION (for extra filtering)
# -----------------------------
def compute_iou(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter = max(0, x2 - x1) * max(0, y2 - y1)

    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])

    return inter / (area1 + area2 - inter + 1e-6)

# -----------------------------
# INFERENCE LOOP
# -----------------------------
inference_times = []
nms_times = []
results = []

for i in range(len(dataset)):
    img, _, img_info, img_id = dataset.pull_item(i)
    img_name = img_info[0]

    img_input, _ = ValTransform(legacy=False)(img, None, TEST_SIZE)
    img_input = torch.from_numpy(img_input).unsqueeze(0).float()

    if DEVICE == "cuda":
        img_input = img_input.cuda()

    # ---- FORWARD ----
    start = time.time()
    with torch.no_grad():
        outputs = model(img_input)
    forward_time = time.time() - start

    # ---- NMS ----
    start = time.time()
    outputs = postprocess(outputs, 1, CONFTHRE, NMSTHRE)
    nms_time = time.time() - start

    inference_times.append(forward_time)
    nms_times.append(nms_time)

    # -----------------------------
    # FILTERING (KEY PART)
    # -----------------------------
    if outputs[0] is not None:
        output = outputs[0].cpu().numpy()

        filtered_boxes = []

        for det in output:
            x1, y1, x2, y2, score, cls = det[:6]

            # 🔥 strong confidence filtering
            if score < 0.4:
                continue

            keep = True
            for fb in filtered_boxes:
                if compute_iou([x1,y1,x2,y2], fb[:4]) > 0.5:
                    keep = False
                    break

            if keep:
                filtered_boxes.append([x1,y1,x2,y2,score,cls])

        # 🔥 TAKE ONLY BEST BOX (VERY IMPORTANT)
        if len(filtered_boxes) > 0:
            filtered_boxes = sorted(filtered_boxes, key=lambda x: x[4], reverse=True)[:1]

        # -----------------------------
        # SAVE RESULTS + DRAW
        # -----------------------------
        for box in filtered_boxes:
            x1, y1, x2, y2, score, cls = box

            results.append({
                "image_id": int(img_id),
                "category_id": 0,
                "bbox": [float(x1), float(y1), float(x2-x1), float(y2-y1)],
                "score": float(score)
            })

            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)

    cv2.imwrite(f"{SAVE_DIR}/{img_id}.jpg", img)

# -----------------------------
# SAVE JSON
# -----------------------------
pred_json = os.path.join(SAVE_DIR, "predictions.json")
with open(pred_json, "w") as f:
    json.dump(results, f)

# -----------------------------
# COCO EVAL
# -----------------------------
cocoGt = dataset.coco
cocoDt = cocoGt.loadRes(pred_json)

cocoEval = COCOeval(cocoGt, cocoDt, "bbox")
cocoEval.evaluate()
cocoEval.accumulate()
cocoEval.summarize()

# -----------------------------
# PERFORMANCE STATS
# -----------------------------
avg_forward = np.mean(inference_times) * 1000
avg_nms = np.mean(nms_times) * 1000
total = avg_forward + avg_nms
fps = 1000 / total

print("\n===== PERFORMANCE =====")
print(f"Forward time: {avg_forward:.2f} ms")
print(f"NMS time: {avg_nms:.2f} ms")
print(f"Total inference: {total:.2f} ms")
print(f"FPS: {fps:.2f}")

# -----------------------------
# GRAPHS
# -----------------------------
plt.figure()
plt.plot(inference_times)
plt.title("Inference Time per Image")
plt.xlabel("Image Index")
plt.ylabel("Time (s)")
plt.savefig(os.path.join(SAVE_DIR, "inference_time.png"))

plt.figure()
plt.plot(nms_times)
plt.title("NMS Time per Image")
plt.xlabel("Image Index")
plt.ylabel("Time (s)")
plt.savefig(os.path.join(SAVE_DIR, "nms_time.png"))

print("Results + graphs saved in:", SAVE_DIR)
