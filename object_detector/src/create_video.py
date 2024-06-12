import cv2
import numpy
from glob import glob
from tqdm import tqdm
import os
import moviepy.video.io.ImageSequenceClip

from model_class import MLModel

if __name__ == '__main__':
    print(os.getcwd())
    out_list = []

    ml_model = MLModel('/RO47007/group18/object_detector/models/tf_lites/efficientnet_tuned_v2.tflite')

    ml_model.load_model()

    img_list = sorted(glob('/RO47007/mirte_with_real_env/JPEGImages/*_2.jpg'))
    i = 0

    video_name = '/RO47007/group18/Detections.mp4'

    video = None

    for img in tqdm(img_list):
        img = cv2.imread(img)

        bboxes = ml_model.inference(img, i)
        i += 1

        # if video is None:
        #     video = cv2.VideoWriter('/RO47007/group18/Detections.mp4v', 0, 1, (img.shape[0],img.shape[1]))


        for bbox in bboxes:
            top_left = (int(bbox.center.x - bbox.size_x / 2),
                        int(bbox.center.y - bbox.size_y / 2))
            bottom_right = (int(bbox.center.x + bbox.size_x / 2),
                            int(bbox.center.y + bbox.size_y / 2))
            img = cv2.rectangle(img, top_left, bottom_right, (255, 0, 0))
        
        out_list.append(img)
        
    # cv2.destroyAllWindows()
    # video.release()
    clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(out_list, fps=60)
    clip.write_videofile(video_name)

