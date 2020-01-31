{
    # Class names of motor controllers used.
    # Options:
    # 'Spark'
    # 'Victor'
    # 'VictorSP'
    # 'PWMTalonSRX'
    # 'PWMVictorSPX'
    # 'WPI_TalonSRX'
    # 'WPI_VictorSPX'
    "rightControllerTypes": ["WPI_TalonSRX", "WPI_TalonSRX", "WPI_TalonSRX"],
    "leftControllerTypes": ["WPI_TalonSRX", "WPI_TalonSRX", "WPI_TalonSRX"],
    # Ports for the left-side motors
    "leftMotorPorts": [1, 2, 3],
    # Ports for the right-side motors
    "rightMotorPorts": [4, 5, 6],
    # Inversions for the left-side motors
    "leftMotorsInverted": [False, False, False],
    # Inversions for the right side motors
    "rightMotorsInverted": [False, False, False],
    # Wheel diameter (in units of your choice - will dictate units of analysis)
    "wheelDiameter": 0.1524,
    # If your robot has only one encoder, set all right encoder fields to `None`
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # This value should be the edges per revolution *of the wheels*, and so
    # should take into account gearing between the encoder and the wheels
    "encoderEPR": 128,
    # Ports for the left-side encoder
    "leftEncoderPorts": [0, 1],
    # Ports for the right-side encoder
    "rightEncoderPorts": [2, 3],
    # Whether the left encoder is inverted
    "leftEncoderInverted": False,
    # Whether the right encoder is inverted:
    "rightEncoderInverted": True,
    # Your gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "NavX",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "SerialPort.Port.kMXP",
}





