from pal.products.qcar import QCar

myCar = QCar(readMode=1, frequency=10)

try:
    while True:

        # Simple constant commands
        throttle = 0.3       # forward (-2 to 2)
        steering = 0.2       # slight right turn (-0.5 to +0.5)
        
        # Send commands
        myCar.write(throttle, steering)

        # Read on-board sensors
        myCar.read()
        
        # Print essential info
        print(
            f"Battery: {myCar.batteryVoltage:.2f} V, "
            f"Motor Current: {myCar.motorCurrent:.2f}, "
            f"Tach: {myCar.motorTach:.2f}"
            )

except KeyboardInterrupt:
    print("\nProgram stopped by user (CTRL+C).")

myCar.write(0.0, 0.0)

