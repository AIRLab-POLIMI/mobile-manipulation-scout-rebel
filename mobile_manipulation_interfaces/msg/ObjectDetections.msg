# content of message: array of bounding boxes and associated labels
# bounding boxes expressed with COCO format: absolute coordinates (x_min, y_min, width, height)
std_msgs/Header header
float32[] bounding_boxes # linearized list of vectors of 4 integers
uint16[] labels # vector of labels
float32[] confidences # vector of confidence scores per predicted bounding box