import numpy as np
import ncnn
import torch
import cv2 as cv
from ultralytics import YOLO

cv_img = cv.imread('code_mega/utralytics/data/images/val/image0239.jpg')
out_img = cv.copyMakeBorder(cv_img, 80, 80, 0, 0, cv.BORDER_CONSTANT, None, value=0)
# out_img = out_img.transpose((2, 0, 1)).astype(np.float32)
# out_img = np.zeros_like(out_img)

# with ncnn.Net() as net:
#     net.load_param("best_ncnn_model/model.ncnn.param")
#     net.load_model("best_ncnn_model/model.ncnn.bin")
#     with net.create_extractor() as ex:
#         # ex.input("in0", ncnn.Mat(in0.squeeze(0).numpy()).clone())
#         ex.input("in0", ncnn.Mat(out_img).clone())
#         out = ex.extract("out0")

model = YOLO('code_mega/group18/src/object_detector/models/best_ncnn_model')
out = model.predict(source=out_img)
# out2 = pass
# print(out[0])
