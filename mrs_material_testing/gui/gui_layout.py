import threading
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTableWidget, QCheckBox, QTableWidgetItem, QGroupBox, QTabWidget, QDoubleSpinBox, QTextEdit
from PyQt5.QtCore import QTimer, Qt
from ros_interface import start_status_update, open_rviz, run_compute_object_center, launch_drivers, quit_drivers, zero_ft_sensors, turn_on_wrench_controllers, turn_on_arm_controllers, turn_on_twist_controllers, enable_all_urs, update_ur_relative_to_object, launch_ros, move_to_initial_pose, turn_on_coop_admittance_controller 
from relative_poses import RelativePoses
from ros_interface import ROSInterface

class ROSGui(QWidget):
    def __init__(self):
        super().__init__()
        self.ros_interface = ROSInterface(self)
        self.setWindowTitle("Multi-Robot Demo")
        self.setGeometry(100, 100, 3200, 2500)  # Increased width
        
        self.workspace_name = "catkin_ws_recker"
        main_layout = QHBoxLayout()
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.ros_interface.update_button_status)
        self.status_timer.start(5000)  # Check status every 5 seconds
        
        # Left Side (Status & Buttons)
        left_layout = QVBoxLayout()
        self.status_label = QLabel("Controller Status: Not Checked")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("border: 1px solid black; padding: 5px;")
        left_layout.addWidget(self.status_label)

        # Robot and UR selection
        selection_group = QGroupBox("Robot and UR Selection")
        selection_layout = QHBoxLayout()

        robot_layout = QVBoxLayout()
        self.robots = {
            "mur620a": QCheckBox("mur620a"),
            "mur620b": QCheckBox("mur620b"),
            "mur620c": QCheckBox("mur620c"),
            "mur620d": QCheckBox("mur620d"),
        }
        for checkbox in self.robots.values():
            robot_layout.addWidget(checkbox)

        ur_layout = QVBoxLayout()
        ur_layout.addWidget(QLabel("Select URs:"))
        self.ur10_l = QCheckBox("UR10_l")
        self.ur10_r = QCheckBox("UR10_r")
        self.ur10_l.setChecked(True)
        self.ur10_r.setChecked(True)
        ur_layout.addWidget(self.ur10_l)
        ur_layout.addWidget(self.ur10_r)

        selection_layout.addLayout(robot_layout)
        selection_layout.addLayout(ur_layout)
        selection_group.setLayout(selection_layout)
        left_layout.addWidget(selection_group)
        
        self.check_set_reference = QCheckBox("Set reference at runtime")
        self.check_set_reference.setChecked(True)
        left_layout.addWidget(self.check_set_reference)
        
        # Setup Functions Group
        setup_group = QGroupBox("Setup Functions")
        setup_layout = QVBoxLayout()
        setup_buttons = {
            "Check Status": lambda: start_status_update(self),
            "Launch Drivers": lambda: launch_drivers(self),
            "Quit Drivers": lambda: quit_drivers(),
            "Open RVIZ": open_rviz,
            "Start Virtual Leader": lambda: launch_ros(self, "virtual_leader", "virtual_leader.launch"),
            "Start Virtual Object": lambda: launch_ros(self, "virtual_object", "virtual_object.launch"),
            "Start Roscore": lambda: self.ros_interface.start_roscore(),
            "Start Mocap": lambda: self.ros_interface.start_mocap(),
            "Start Sync": lambda: self.ros_interface.start_sync(),
        }

        for text, function in setup_buttons.items():
            btn = QPushButton(text)
            
            # Speichert spezielle Buttons für Status-Updates
            if text == "Start Roscore":
                self.btn_roscore = btn
            elif text == "Start Mocap":
                self.btn_mocap = btn
            elif text == "Start Sync":
                self.btn_sync = btn

            btn.clicked.connect(lambda checked, f=function: f())
            btn.setStyleSheet("background-color: lightgray;")  # Standardfarbe
            setup_layout.addWidget(btn)
        setup_group.setLayout(setup_layout)
        left_layout.addWidget(setup_group)
        
        # Utility Functions Group
        utility_group = QGroupBox("Utility Functions")
        utility_layout = QVBoxLayout()
        utility_buttons = {
            "Compute Object Center": lambda: run_compute_object_center(self),
            "Zero F/T Sensors": lambda: zero_ft_sensors(self),
            "Enable all URs": lambda: enable_all_urs(self),
            "Update UR relative to object": lambda: update_ur_relative_to_object(self),
        }
        for text, function in utility_buttons.items():
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, f=function: f())
            utility_layout.addWidget(btn)
        utility_group.setLayout(utility_layout)
        left_layout.addWidget(utility_group)
                
        main_layout.addLayout(left_layout)
        
        # Right Side (Table for Relative Poses)
        self.table = QTableWidget(9, 6)
        self.table.setHorizontalHeaderLabels(["X", "Y", "Z", "Rx", "Ry", "Rz"])
        self.table.setVerticalHeaderLabels([
            "mur620a/UR10_l", "mur620a/UR10_r", "mur620b/UR10_l", "mur620b/UR10_r", 
            "mur620c/UR10_l", "mur620c/UR10_r", "mur620d/UR10_l", "mur620d/UR10_r", "Virtual Object"
        ])
        self.load_relative_poses()
        
        # Save and Update Buttons
        self.btn_save_poses = QPushButton("Save Poses")
        self.btn_save_poses.clicked.connect(lambda: self.save_relative_poses())
        
        self.btn_update_poses = QPushButton("Update Poses")
        self.btn_update_poses.clicked.connect(self.ros_interface.update_poses)
        # Add button to manually get virtual object pose
        self.btn_get_virtual_object_pose = QPushButton("Get Virtual Object Pose")
        self.btn_get_virtual_object_pose.clicked.connect(self.ros_interface.get_virtual_object_pose_once)
        self.btn_move_virtual_object = QPushButton("Move Object to Initial Pose")
        self.btn_move_virtual_object.clicked.connect(self.ros_interface.move_virtual_object_to_initial_pose)

        
        
        
        right_layout = QVBoxLayout()

        # Füge die bestehende Tabelle hinzu
        right_layout.addWidget(self.table)

        # Buttons für "Save Poses" und "Update Poses"
        pose_button_layout = QVBoxLayout()
        pose_button_layout.addWidget(self.btn_get_virtual_object_pose)
        pose_button_layout.addWidget(self.btn_save_poses)
        pose_button_layout.addWidget(self.btn_update_poses)
        right_layout.addLayout(pose_button_layout)

        # Erstelle die "Controller Functions" Gruppe und füge sie rechts hinzu
        controller_group = QGroupBox("Controller Functions")
        controller_layout = QVBoxLayout()
        controller_buttons = {
            "Turn on Wrench Controllers": lambda: turn_on_wrench_controllers(self),
            "Turn on Arm Controllers": lambda: turn_on_arm_controllers(self),
            "Turn on Twist Controllers": lambda: turn_on_twist_controllers(self),
            "Move to Initial Pose Left": lambda: move_to_initial_pose(self, "UR10_l"),
            "Move to Initial Pose Right": lambda: move_to_initial_pose(self, "UR10_r"),
            "Move Object to Initial Pose": lambda: self.ros_interface.move_virtual_object_to_initial_pose(),
            "Turn on Admittance Controller": lambda: turn_on_coop_admittance_controller(self),
            "Update Relative Poses for Admittance Controller": lambda: self.ros_interface.update_relative_poses_for_admittance_controller(),
        }

        for text, function in controller_buttons.items():
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, f=function: f())
            controller_layout.addWidget(btn)

        controller_group.setLayout(controller_layout)
        
        # Motion Demos GroupBox
        motion_demos_group = QGroupBox("Motion Demos")
        self.motion_demos_layout = QVBoxLayout()
        
        # Add Lissajous Controls
        self.add_lissajous_controls("Lissajous 3D Position", "lissajous_3D_position_publisher.launch")
        self.add_lissajous_controls("Lissajous 3D Orientation", "lissajous_3D_orientation_publisher.launch")
        self.add_lissajous_controls("Lissajous 6D Combined", "lissajous_6D_combined_publisher.launch")

        btn_lissajous_stop = QPushButton("Stop Lissajous Motion")
        btn_lissajous_stop.clicked.connect(lambda: self.ros_interface.stop_lissajous_motion())
        self.motion_demos_layout.addWidget(btn_lissajous_stop)


        motion_demos_group.setLayout(self.motion_demos_layout)
        

        right_layout.addWidget(controller_group)
        right_layout.addWidget(motion_demos_group)

        main_layout.addLayout(right_layout)  # Fügt das Layout auf der rechten Seite hinzu

        self.setLayout(main_layout)

    
    def add_lissajous_controls(self, label_text, launch_file):
        """Adds a labeled button with a velocity input box."""
        hbox = QHBoxLayout()
        label = QLabel(label_text)
        velocity_input = QDoubleSpinBox()
        velocity_input.setRange(0.0, 5.0)
        velocity_input.setSingleStep(0.1)
        velocity_input.setValue(1.0)
        button = QPushButton(label_text)
        
        button.clicked.connect(lambda: self.ros_interface.launch_lissajous_demo(launch_file, velocity_input.value()))
        
        #hbox.addWidget(label)
        hbox.addWidget(velocity_input)
        hbox.addWidget(button)
        self.motion_demos_layout.addLayout(hbox)




    def update_virtual_object_pose(self, pose):
        """Updates the GUI table with the latest virtual object pose."""
        for col in range(6):
            self.table.setItem(8, col, QTableWidgetItem(str(round(pose[col], 4))))
    
    def get_selected_robots(self):
        return [name for name, checkbox in self.robots.items() if checkbox.isChecked()]

    def get_selected_urs(self):
        ur_prefixes = []
        if self.ur10_l.isChecked():
            ur_prefixes.append("UR10_l")
        if self.ur10_r.isChecked():
            ur_prefixes.append("UR10_r")
        return ur_prefixes

    def save_relative_poses(self, updated_poses=None):
        """Collects values from the table and saves them. If updated_poses is provided, those values are used first."""
        poses = {}

        # Convert `updated_poses` keys to match the table format ("mur620c/UR10_l")
        if updated_poses:
            formatted_updated_poses = {f"{robot}/{ur}": pos for (robot, ur), pos in updated_poses.items()}
        else:
            formatted_updated_poses = {}

        for row in range(self.table.rowCount()):
            row_label = self.table.verticalHeaderItem(row).text()

            # Use updated pose values if available; otherwise, keep existing table values
            if row_label in formatted_updated_poses:
                poses[row_label] = formatted_updated_poses[row_label]
            else:
                poses[row_label] = [
                    float(self.table.item(row, col).text()) if self.table.item(row, col) else 0.0
                    for col in range(6)  # Jetzt für X, Y, Z, Rx, Ry, Rz
                ]

        # Save values to poses.yaml
        relative_poses = RelativePoses()
        relative_poses.save_poses(poses)



    def load_relative_poses(self):
        """Lädt die gespeicherten Posen und setzt sie in die Tabelle ein."""
        relative_poses = RelativePoses()  # Instanz erstellen
        poses = relative_poses.load_poses()  # Geladene Posen als Dictionary

        for row in range(self.table.rowCount()):
            row_label = self.table.verticalHeaderItem(row).text()
            if row_label in poses:
                for col in range(self.table.columnCount()):
                    print(f"Setting {row_label} at {col} to {poses[row_label][col]}")
                    print("coloncount", self.table.columnCount())
                    value = poses[row_label][col] if col < len(poses[row_label]) else 0.0
                    self.table.setItem(row, col, QTableWidgetItem(str(value)))

                    if row_label == "Virtual Object":
                        self.ros_interface.virtual_object_pose = poses[row_label]  # Ensure it's loaded properly

    def get_relative_pose(self, robot, ur):
        """Retrieves the relative pose [x, y, z] from the table for the given robot and UR arm."""
        row_label = f"{robot}/{ur}"
        
        for row in range(self.table.rowCount()):
            if self.table.verticalHeaderItem(row).text() == row_label:
                return [
                    float(self.table.item(row, col).text()) if self.table.item(row, col) else 0.0
                    for col in range(6)  # Jetzt für X, Y, Z, Rx, Ry, Rz
                ]
        
        # Default value if no match is found
        return [0.0, 0.0, 0.0]
