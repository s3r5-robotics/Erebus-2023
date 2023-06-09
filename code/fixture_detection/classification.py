import tensorflow as tf
import cv2 as cv


class Classifier:
    MODEL_DATA = {
        "hsu_rev-1": "models/rgb_hsu_rev-1.lit"
    }

    def __init__(self, model: str):
        self.model = self.MODEL_DATA[model]

        self.interpreter = tf.lite.Interpreter(self.model)
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.interpreter.allocate_tensors()

    def classify_fixture(self, fixture) -> str:
        image = cv.resize(fixture["image"], (100, 100), interpolation=cv.INTER_AREA)

        return ""
