import os
from yolox.exp import Exp as MyExp

class Exp(MyExp):
    def __init__(self):
        super(Exp, self).__init__()

        # Model
        self.depth = 0.33
        self.width = 0.25
        self.input_size = (416, 416)
        self.random_size = (10, 20)
        self.mosaic_scale = (0.5, 1.5)

        # Dataset
        self.num_classes = 1
        self.data_dir = "/home/gputesting/YOLO_pose_alternative_models/dataset/all_data_coco_format"
        self.train_ann = "pallet_train.json"
        self.val_ann = "pallet_val.json"
        self.train_image_dir = "images/train2017"   # <-- fix
        self.val_image_dir = "images/val2017"       # <-- fix

        # Training
        self.max_epochs = 200
        self.warmup_epochs = 5
        self.basic_lr_per_img = 0.01 / 8.0
        self.eval_interval = 5
        self.exp_name = "yolox_nano_pallet"
