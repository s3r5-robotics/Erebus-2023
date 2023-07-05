from typing import Literal

import keras
import tensorflow as tf
import numpy.typing as npt
import numpy as np

import debug


class FixtureClassifier:
    CLASSES = ['0', 'C', 'F', 'H', 'O', 'P', 'S', 'U']
    MODEL_DATA = {
        "rev-1": "fixture_detection/models/test_fr-hp.keras",
        "rev-2": "fixture_detection/models/test_frt-hp.keras",
        "rev-3": "fixture_detection/models/test_ff-hp.keras",
    }

    def __init__(self, model: Literal["rev-1", "rev-2", "rev-3"]):
        self.model = keras.models.load_model(self.MODEL_DATA[model])

    def classify_fixture(self, fixture: npt.ArrayLike) -> str:
        img_array = tf.expand_dims(fixture, 0)
        predictions = self.model.predict(img_array)
        predictions_sorted = sorted(map(lambda p, c: (c, p), predictions[0], self.CLASSES), key=lambda t: -t[1])
        prediction: tuple[str, float] = predictions_sorted[0]

        if debug.PREDICTIONS:
            print(predictions_sorted)

        return prediction[0]

    @staticmethod
    def prepare_image(img: npt.ArrayLike) -> npt.ArrayLike:
        arr = np.delete(img, 3, axis=2)
        arr = np.flip(arr, axis=2)  # Swap blue and red channels to get RGB from BGR
        return arr
