import ms5837


class Bar30:

    def __init__(self, bus=6):

        self.sensor = ms5837.MS5837_30BA(bus=bus)

        if not self.sensor.init():
            raise RuntimeError(
                f"Impossible d'initialiser le Bar30 sur I2C bus {bus}"
            )

        self.sensor.setFluidDensity(
            ms5837.DENSITY_SALTWATER
        )

    def read(self):

        if not self.sensor.read():
            raise RuntimeError("Erreur de lecture du Bar30")

        return {
            "mbar": self.sensor.pressure(),
            "pa" : self.sensor.pressure()*100.0,
            "temperature": self.sensor.temperature(),
            "depth": self.sensor.depth(),
        }
