import keras
import tensorflow as tf
import cv2 as cv


class Classifier:
    MODEL_DATA = {
        "rev-1": "models/rev-1.keras"
    }

    def __init__(self, model: str):
        self.model = keras.models.load_model(self.MODEL_DATA[model])

    def classify_fixture(self, fixture) -> str:
        img_array = tf.expand_dims(fixture["image"], 0)
        predictions = self.model.predict(img_array)
        predictions_sorted = sorted(map(lambda p, c: (c, p), predictions[0], 3), key=lambda t: t[1])

        return ""
