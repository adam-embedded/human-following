import cv2
import numpy as np
from tflite_support.task import processor
import motorControl as mc

_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red

x_deviation = 0
y_max = 0
tolerance = 0.1
arr_track_data = [0, 0, 0, 0, 0, 0]
global_width = 0

controller = mc.motorcontroller()
controller.thread.start()


# calculate suggested speed for robot to move
def get_speed(deviation) -> int:
    global global_width
    deviation = abs(deviation)
    if deviation >= (0.4 * global_width):
        # d = 0.080
        d = mc.speed.medium_full
    elif (0.35 * global_width) <= deviation < (0.40 * global_width):
        # d = 0.060
        d = mc.speed.half
    elif (0.20 * global_width) <= deviation < (0.35 * global_width):
        # d = 0.050
        d = mc.speed.medium_slow
    else:
        # d = 0.040
        d = mc.speed.slow
    return d


def move_robot():
    global x_deviation, y_max, tolerance, arr_track_data, global_width

    # print("moving robot .............!!!!!!!!!!!!!!")
    # print(x_deviation, tolerance, arr_track_data)

    y = 1 - y_max  # distance from bottom of the frame

    # check that the deviation on the x-axis is not exceeding the tolerance
    if abs(x_deviation) < (tolerance * global_width):
        delay1 = 0
        if (y < 25):
            cmd = "Stop"
            controller.move(mc.direction.stop, 0)
        else:
            cmd = "forward"
            controller.move(mc.direction.forward, mc.speed.slow)

    else:
        if x_deviation >= (tolerance * global_width):
            cmd = "Move Left"
            # delay1 = get_delay(x_deviation)

            speed = get_speed(x_deviation)
            controller.move(mc.direction.left, speed)
            # ut.stop()

        if x_deviation <= -1 * (tolerance * global_width):
            cmd = "Move Right"
            # delay1 = get_delay(x_deviation)

            speed = get_speed(x_deviation)
            controller.move(mc.direction.right, speed)

            # ut.right()
            # time.sleep(delay1)
            # ut.stop()

    arr_track_data[4] = cmd
    arr_track_data[5] = 0  # delay1


def track(
        image: np.ndarray,
        detection_result: processor.DetectionResult,
) -> np.ndarray:
    """Draws bounding boxes on the input image and return it.

  Args:
    image: The input RGB image.
    detection_result: The list of all "Detection" entities to be visualize.

  Returns:
    Image with bounding boxes.
  """

    global x_deviation, y_max, tolerance, global_width

    flag = 0
    height, width, channels = image.shape
    global_width = width

    if len(detection_result.detections) == 0:
        # here make the robot search for a person after stop
        controller.move(mc.direction.stop, 0)
        return image

    for detection in detection_result.detections:

        if detection.categories[0].category_name == "person":

            # Draw bounding_box
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)

            box_center_x = bbox.origin_x + (bbox.width / 2)
            box_center_y = bbox.origin_y + (bbox.height / 2)

            # find the deviation from the center
            x_deviation = round((width / 2) - box_center_x, 3)
            y_max = round((bbox.origin_y + bbox.height) - height, 3)

            # print("{", x_deviation, y_max, end_point, "}")

            arr_track_data[0] = box_center_x
            arr_track_data[1] = box_center_y
            arr_track_data[2] = x_deviation
            arr_track_data[3] = y_max

            # Write x deviation to opencv
            x_dev = arr_track_data[2]
            str_x = 'X: {}'.format(x_dev)
            if (abs(x_dev) < tolerance):
                color_x = (0, 255, 0)
            else:
                color_x = (0, 0, 255)
            image = cv2.putText(image, str_x, (110, height - 8), cv2.FONT_HERSHEY_PLAIN, _FONT_SIZE, color_x, 2)

            # Write y deviation to opencv
            y_dev = arr_track_data[3]
            str_y = 'Y: {}'.format(y_dev)
            if (abs(y_dev) > 0.9):
                color_y = (0, 255, 0)
            else:
                color_y = (0, 0, 255)
            image = cv2.putText(image, str_y, (220, height - 8), cv2.FONT_HERSHEY_PLAIN, _FONT_SIZE, color_y, 2)

            # draw center cross lines
            image = cv2.rectangle(image, (0, int(height / 2) - 1), (width, int(height / 2) + 1), (255, 0, 0), -1)
            image = cv2.rectangle(image, (int(width / 2) - 1, 0), (int(width / 2) + 1, height), (255, 0, 0), -1)

            # draw the center red dot on the object
            image = cv2.circle(image, (int(arr_track_data[0]), int(arr_track_data[1])), 7,
                               (0, 0, 255), -1)
            # image = cv2.circle(image, (int(end_point[0]), int(end_point[1])), 7,
            #                   (0, 0, 255), -1)

            # draw the tolerance box
            image = cv2.rectangle(image, (int(width / 2 - tolerance * width), 0),
                                  (int(width / 2 + tolerance * width), height), (0, 255, 0), 2)

            # write command, tracking status and speed
            cmd = arr_track_data[4]
            image = cv2.putText(image, str(cmd), (int(width / 2) + 10, height - 8), cv2.FONT_HERSHEY_PLAIN, _FONT_SIZE,
                                (0, 255, 255), 1)

            delay1 = arr_track_data[5]
            str_sp = 'Speed: {}%'.format(round(delay1 / (0.1) * 100, 1))
            image = cv2.putText(image, str_sp, (int(width / 2) + 185, height - 8), cv2.FONT_HERSHEY_PLAIN, _FONT_SIZE,
                                (150, 150, 255), 1)

            if (cmd == 0):
                str1 = "No object"
            elif (cmd == 'Stop'):
                str1 = 'Acquired'
            else:
                str1 = 'Tracking'
            image = cv2.putText(image, str1, (width - 140, 18), cv2.FONT_HERSHEY_PLAIN, _FONT_SIZE, (0, 255, 255), 1)

            # Draw label and score
            category = detection.categories[0]
            category_name = category.category_name
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (_MARGIN + bbox.origin_x,
                             _MARGIN + _ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)

            #          move robot
            move_robot()

        else:
            controller.move(mc.direction.stop, 0)

    return image


def visualize(
        image: np.ndarray,
        detection_result: processor.DetectionResult,
) -> np.ndarray:
    """Draws bounding boxes on the input image and return it.

  Args:
    image: The input RGB image.
    detection_result: The list of all "Detection" entities to be visualize.

  Returns:
    Image with bounding boxes.
  """
    for detection in detection_result.detections:
        # Draw bounding_box
        bbox = detection.bounding_box

        start_point = bbox.origin_x, bbox.origin_y
        end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
        cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)

        # Draw label and score
        category = detection.categories[0]
        category_name = category.category_name
        probability = round(category.score, 2)
        result_text = category_name + ' (' + str(probability) + ')'
        text_location = (_MARGIN + bbox.origin_x,
                         _MARGIN + _ROW_SIZE + bbox.origin_y)
        cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                    _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)

    return image
