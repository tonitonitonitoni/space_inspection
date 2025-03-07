gz topic -t "/model/free_flyer/joint_trajectory" -m gz.msgs.JointTrajectory -p '
    joint_names: "joint1"
    joint_names: "joint2"
    joint_names: "joint3"
    joint_names: "joint4"
    points {
      positions: 0.1
      positions: 0.1
      positions: 0.1
      positions: 0.1
      
      time_from_start {
        sec: 0
        nsec: 250000000
      }
    }
    points {
      positions: 0.4
      positions: 0.4
      positions: 0.4
      positions: 0.4
      time_from_start {
        sec: 0
        nsec: 500000000
      }
    }
    points {
      positions: 0.2
      positions: 0.2
      positions: 0.2
      positions: 0.2

      time_from_start {
        sec: 0
        nsec: 750000000
      }
    }
    points {
      positions: 0
      positions: 0
      positions: 0
      positions: 0
      
      time_from_start {
        sec: 1
        nsec: 0
      }
    }'