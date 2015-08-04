# THIS FILE MAY BE ENTIRELY UNNECESSARY
SensorList = []


def make_sensors(sensor_count):
    for i in range(sensor_count):
        SensorList.append(0.0)

make_sensors(16)


def update_sensor(sensor):
    return SensorList[sensor]