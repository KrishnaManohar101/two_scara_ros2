# Project Archive

This folder (`project_assets`) contains all the documentation, presentations, standalone scripts, and data logs for the Two SCARA Robots project.

## Directory Structure

### 📄 `/documents`
Contains all project reports, research papers, and technical explanations.
*   **Key Files**: `Project_Report.md`, `RESEARCH_PAPER_FULL.pdf`, `KINEMATICS_EXPLANATION.md`
*   **Original READMEs**: `README.md`, `PROJECT_README.md` are archived here.

### 🐍 `/scripts`
Contains standalone Python scripts for robot control and utilities.
*   **Controls**: 
    *   `direct_joint_control.py`: Manual angle input.
    *   `torque_control_sim.py`: Forward dynamics physics sim.
    *   `keyboard_joint_control.py`: Arrow key teleop.
*   **Utilities**: `generate_ppt.py`, `convert_md_html.py`.
*   **Note**: When running these scripts, you may need to adjust the Python path or run them from the root directory pointing here (e.g., `python3 project_assets/scripts/direct_joint_control.py`).

### 📊 `/presentations`
Contains PowerPoint slides and HTML presentations.
*   `Dual_SCARA_Final_Presentation.pptx`

### 📈 `/data_and_logs`
Contains simulation logs, CSV data, and output images.
*   `replication_data.csv`
*   `trajectory_comparison.png`
