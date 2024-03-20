from smart_gripper_pkg.scripts import Gripper

def main():
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        example = Gripper(router)
        example.gripper_width(0.545)        # example.gripper_vel(-0.1)

if __name__ == "__main__":
    main()