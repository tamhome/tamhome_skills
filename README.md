# Tamhome skills

- 把持，物体検出などのスキルをパッケージ化
- 今後，Handymanの要素開発はすべてここで行う
- 他のタスクが来たときに対応しやすいよう，キラーアプリケーションは作らない

## How to install

```bash
pip install -e .
```

## How to use

```python
import rospy
from tamhome_skills import Grasp
rospy.init_node("test")
grasp = Grasp()
target_pose = Pose()
target_pose.position.x = 0
target_pose.position.y = 1
target_pose.position.z = 0.2
target_pose.orientation.x = 0.0
target_pose.orientation.y = 0.0
target_pose.orientation.z = 0.0
target_pose.orientation.w = 1
grasp.grasp_obj_by_pose(target_pose)
```