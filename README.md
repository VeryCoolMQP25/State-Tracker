# State-Tracker

Maintains current floor of robot. Recieves input from the /current_floor topic. Ideally, we will publish to this topic after pressing an elevator button. 

Once the user makes a request for a specific floor, it will publish the requested floor to the /requested_floor topic.

The current and requested floors are compared. If they match, the robot publishes the user's goal pose to /goal_pose and leads the user to their destination.
If the floors do not match, the robot drives to the elevator on the current floor in preparation to enter the elevator.

Already part of tori_bringup
