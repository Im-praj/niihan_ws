import time

class SafetyManager:
    def __init__(self, timeout_sec=0.3, geofence_manager=None):
        self.estop_active = False
        self.last_command_time = 0.0
        self.timeout_sec = timeout_sec
        self.mode = "MANUAL" # MANUAL or AUTO
        self.geofence_manager = geofence_manager

    def trigger_estop(self):
        self.estop_active = True

    def clear_estop(self):
        self.estop_active = False

    def update_command_time(self):
        self.last_command_time = time.time()

    def set_mode(self, mode):
        if mode in ["MANUAL", "AUTO"]:
            self.mode = mode

    def can_move(self, current_x=None, current_y=None):
        if self.estop_active:
            return False
        
        if self.geofence_manager and current_x is not None and current_y is not None:
            if not self.geofence_manager.is_robot_inside(current_x, current_y):
                self.trigger_estop()
                return False
        
        # Check timeout for manual commands
        if self.mode == "MANUAL":
            if time.time() - self.last_command_time > self.timeout_sec:
                return False
                
        return True

    def validate_manual_command(self):
        if self.estop_active or self.mode != "MANUAL":
            return False
        self.update_command_time()
        return True

    def validate_auto_command(self):
        if self.estop_active or self.mode != "AUTO":
            return False
        return True
