#!/bin/bash

ros2 action send_goal /thymio1/navigate_to_pose nav2_msgs/action/NavigateToPose "$(cat <<'EOF'
{
pose: {
  header: {
    stamp: {
      sec: 0,
      nanosec: 0,
    },
    frame_id: map
    },
  pose: {
    position: {
      x: 6.0,
      y: 5.0,
      z: 0.0
      },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
      }
    }
  }
}
EOF
)" &

ros2 action send_goal /thymio2/navigate_to_pose nav2_msgs/action/NavigateToPose "$(cat <<'EOF'
{
pose: {
  header: {
    stamp: {
      sec: 0,
      nanosec: 0,
    },
    frame_id: map
    },
  pose: {
    position: {
      x: 6.0,
      y: 0.0,
      z: 0.0
      },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
      }
    }
  }
}
EOF
)" &

ros2 action send_goal /thymio3/navigate_to_pose nav2_msgs/action/NavigateToPose "$(cat <<'EOF'
{
pose: {
  header: {
    stamp: {
      sec: 0,
      nanosec: 0,
    },
    frame_id: map
    },
  pose: {
    position: {
      x: 0.0,
      y: 5.0,
      z: 0.0
      },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
      }
    }
  }
}
EOF
)" &

ros2 action send_goal /thymio4/navigate_to_pose nav2_msgs/action/NavigateToPose "$(cat <<'EOF'
{
pose: {
  header: {
    stamp: {
      sec: 0,
      nanosec: 0,
    },
    frame_id: map
    },
  pose: {
    position: {
      x: 0.0,
      y: 0.0,
      z: 0.0
      },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
      }
    }
  }
}
EOF
)"


