# Professional Presentation Guide: Dual-SCARA 3D Robotics System

This guide outlines the step-by-step workflow to deliver a high-impact presentation. It covers the setup, the demo script, and how to handle technical questions using your simulation.

---

## 1. Preparation & Setup
Before the evaluators arrive, open **four terminal windows** and have them arranged on your screen.

### Terminal 1: The Simulation Environment
**Purpose:** Launches Gazebo (Physics) and RViz (Visualization).

**Option A: Standard Profile**
```powershell
ros2 launch two_scara_collaboration initialize.launch.py
```

**Option B: Heavy Duty Profile (Visual Scaling)**
*Use this to show the teacher how the robot structure changes for heavy loads.*
```powershell
ros2 launch two_scara_collaboration initialize.launch.py profile:=heavy
```

**Option C: Slim Profile**
```powershell
ros2 launch two_scara_collaboration initialize.launch.py profile:=slim
```

### Terminal 2: The Robotics Dashboard (Telemetric)
**Purpose:** Shows real-time FK/IK and Dynamics math as requested by the teacher.
```powershell
# In project root
python3 live_kinematics_monitor.py
```

### Terminal 3: The Command Center (Manual Control)
**Purpose:** Allows you to drive the robot to answer "what if" questions.
```powershell
# In project root
python3 keyboard_joint_control.py
```

### Terminal 4: The Automation Log
**Purpose:** Shows the replication logic and CSV data logging.
```powershell
# In project root
python3 src/two_scara_collaboration/two_scara_collaboration/automation_manager.py
```

---

## 2. Presentation Script (Step-by-Step)

### Phase 1: The TRANSLATION Phase (X-Y Plane)
**Goal:** Prove the "Data One, Data Two" requirement.
1.  Point to the **Dashboard (Terminal 2)**.
2.  Use the **Keyboard (Terminal 3)** or **Automation Manager (Terminal 4)** to start a move.
3.  **Explain:** *"In Phase 1, the robot performs **Translation**. The two Revolute joints solve the Inverse Kinematics to position the end-effector accurately over the target in the horizontal plane."*

### Phase 2: The INSERTION Phase (Z-Axis)
**Goal:** Demonstrate the 3D Degree of Freedom.
1.  Observe the robot lowering its gripper in Gazebo.
2.  **Explain:** *"Once Translation is complete, the robot begins **Insertion**. This utilizes the Prismatic 3rd Degree of Freedom to move vertically along the Z-axis. This sequential decoupling ensures maximum precision and collision avoidance."*

### Phase 3: Dynamics & The Effect of Force
**Goal:** Demonstrate the impact of Mass and Gravity.
1.  Note the **"Current Torque"** on the Dashboard (Terminal 2).
2.  Press **'M'** on the Dashboard terminal to increase the **Payload Mass**.
3.  **Observe:** Show how the **"Vertical Force"** required to fight gravity significantly increases.
4.  **Explain:** *"We aren't just simulating motion; we are simulating physics. By increasing the payload mass, the dynamics engine recalculates the required torque. This proves we are accounting for mass, acceleration, and gravity compensation."*

### Phase 4: Collaborative Replication
**Goal:** Show the dual-robot coordination.
1.  Go to **Terminal 4 (Automation Manager)**.
2.  Type a world coordinate like `2.0 -3.0`.
3.  **Observe:** Both robots will move symmetrically.
4.  **Explain:** *"Our Automation Manager utilizes a mirror-mapping algorithm. When the Left robot performs a task, the Right robot creates a symmetric replication in real-time."*

---

## 3. Handling Evaluator Questions (Mapping)

| If they ask... | Show this... | Say this... |
| :--- | :--- | :--- |
| **"Where is your Forward Kinematics?"** | **Terminal 2 (Green Section)** | "The FK converts joint angles into X,Y,Z. It's working live in the 'Forward Kinematics' column of this dashboard." |
| **"How do you handle 3D?"** | **Gazebo + Keyboard 'W/S'** | "The Z-axis is modeled as a prismatic joint with limits from 0 to -0.6m in the URDF, fully integrated into the 3D FK/IK engine." |
| **"Can you change the mass?"** | **Terminal 2 (Press 'M')** | "I can change the payload mass live. Watch the 'Dynamic Force' bar increase as I add weight." |
| **"What about Inverse Kinematics?"** | **Terminal 2 (IK Check Column)** | "Our IK solver takes the XYZ position and calculates the required joints. This column shows the IK results matching the real joints, proving the loop is closed." |
| **"Is it just animation or physics?"** | **Terminal 2 (Yellow Section)** | "It is physics-based. We use the Inertia Matrix $M(q)$ and Gravity $G(q)$ to estimate high-fidelity torques for every move." |
| **"Where are the results saved?"** | **`replication_data.csv`** | "Every movement and its symmetric calculation is logged with timestamps in this CSV file for production analysis." |

---

## 4. Final Verification Checklist
- [ ] **Z-Axis motion:** 'W' moves up, 'S' moves down.
- [ ] **Torque scaling:** Does Torque increase when Mass increases? (Yes).
- [ ] **IK Loop:** Do the IK-results on the dashboard match the real joints? (Yes).
- [ ] **Replication:** Does the right robot follow the left? (Yes).
