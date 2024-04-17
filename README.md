# brick-anomaly

Author: Gilbert Chen

### Descripotion
This project is to calculate the pose estimation of the central brick. The input is an RGB image, a depth image, and the camera parameters. The output is the rotation and translation matrix of pose estimation.

### Demo
Please install the required package befote running by:
```bash
pip install -r requirements.txt
```
Then, to run:
```bash
python GBA_batch_run.py
```
### Versions
#### Current version: V1

1) Segmentation model not implemented (manually assigning the output by segmentation model to verify the performance of steps after the segmentation).
2) The bounding box of the segmentation model is upright, and this will be improved in V2 which fits to slanted bounding boxes.
