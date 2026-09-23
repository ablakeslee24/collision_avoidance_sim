from project_drone import Drone
from detect_conflict import predict_conflict
from resolve_conflict import Resolution


def main():
    drones = [
        Drone(1, start=[0, 0], end=[100, 100], speed=15.0, priority=1),
        Drone(2, start=[0, 100], end=[100, 0], speed =15.0, priority=2)
    ]

    predicted = predict_conflict(drones)

    print("Predicted conflicts:")
    for conflict in predicted:
        print(conflict)

    resolver = Resolution()

    dt = 0.1
    current_time = 0.0

    while not all(drone.arrived for drone in drones):
        resolver.resolve(drones, current_time)

        for drone in drones:
            drone.step(dt, current_time)

        current_time += dt


    print("\nSimulation complete")

    for drone in drones:
        print(drone)

    print("\nResolution summary:")
    print(resolver.summary())

if __name__ == "__main__":
    main()