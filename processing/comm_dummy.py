import random
def readSerial():
    """
    Fake Serial interface for testing purposes
    :return: Random pairs of values
    """
    return (random.randint(0,36), random.randint(0,15))