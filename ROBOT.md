# to run on robot

uncomment the following lines:

## decisions.py:
uncomment the following:

    #TO CHANGE / UNCOMMENT LATER: commented out for now as no odom simulation
    #  ------------------------------------        
    # spin_once(self.localizer)

    # if self.localizer.getPose() is None:
    #     print("waiting for odom msgs ....")
    #     return

to ensure live odom data can be retrieved. This need to be edited out in simulation so spin_once doesn't hang (odom will always return None otherwise)


## localization.py:
Since we have odometry data now, instead of default hard-coded pose updates, get live pose updates:

    def getPose(self):
        return self.pose


# to run in simulation
opposite of above:

    def getPose(self):
        # TO CHANGE, NEW --- handle no odom simulation method
        if self.pose is None:
            # Set a default pose if no odometry data has been received
            print("No odom, setting default pose")
            timestamp = 5
            return [0.0, 0.0, 0.0, timestamp]  # Example default pose: x=0, y=0, yaw=0
        return self.pose


ensures vel_request has a return that isn't None (since we need odom data)