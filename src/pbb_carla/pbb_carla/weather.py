"""Day and night weather choices shared by launch and the CARLA bridge.

Keep CARLA imports inside make_weather: ROS launch uses the system Python,
while the bridge uses the virtual environment containing CARLA.
"""

WEATHER_PRESETS = ('ClearNoon', 'SoftRainNoon', 'MidRainyNoon', 'HardRainNoon',
                   'ClearNight', 'SoftRainNight', 'MidRainyNight', 'HardRainNight')


def make_weather(name):
    """Copy an official preset and add explicit wetness for rainy roads."""
    if name not in WEATHER_PRESETS:
        raise ValueError(f'Unknown weather {name!r}; choose from {WEATHER_PRESETS}')
    import carla

    preset = getattr(carla.WeatherParameters, name)
    weather = carla.WeatherParameters()
    # CARLA preset objects are mutable singletons; never edit them in place.
    for field in ('cloudiness', 'precipitation', 'precipitation_deposits',
                  'wind_intensity', 'sun_azimuth_angle', 'sun_altitude_angle',
                  'fog_density', 'fog_distance', 'fog_falloff', 'wetness',
                  'scattering_intensity', 'mie_scattering_scale',
                  'rayleigh_scattering_scale', 'dust_storm'):
        setattr(weather, field, getattr(preset, field))
    # Noon rain presets need explicit wetness; night presets already include it.
    noon_wetness = {'SoftRainNoon': 50.0, 'MidRainyNoon': 60.0, 'HardRainNoon': 100.0}
    if name in noon_wetness:
        weather.wetness = noon_wetness[name]
    return weather
