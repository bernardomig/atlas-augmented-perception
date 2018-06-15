
class BaseDetector:
    """Base for all the detector classes.
    Detection classes should implement a predict method for making the actual predictions
    and a load_model to load the model.
    """

    def __init__(self):
        pass

    def load_model(self, model):
        """Loads a model from disk or from memory.
        Maybe handle loading from a url for easy update of the model.
        """

        raise NotImplementedError()
    
    def predict(self, image):
        """Returns the objects presented in the image for a given input image.
        The output should be an array of objects containing:
        - A 'label' property, matching the class of the object.
          For example: car, pedestrian.
        - A 'bbox' property, matching the ROI of the object in pixel coordinates.
        """
        
        raise NotImplementedError()
