"""
Andrea Favero 20260517

MirrorBallBot (MBB), an alternative ball balance robot


More info at:
  https://github.com/AndreaFavero71/mirrorballbot
  https://www.instructables.com/MirrorBallBot-MBB-An-Alternative-Ball-Balancing-Ro/

Code for centralized Settings Manager




MIT License

Copyright (c) 2026 Andrea Favero

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

__version__ = "0.0.2"

from dataclasses import dataclass, field, asdict, fields, is_dataclass
from typing import Optional, List, Dict, Any, Union
from time import time, ctime
import shutil
import json
import os


# ============================================================================
# TYPE CONVERSION UTILITIES
# ============================================================================

def convert_value_to_type(value: Any, target_type: type) -> Any:
    """
    Convert a value to the target type with intelligent type coercion.
    
    Handles:
    - Boolean conversion from string/bool/int
    - Numeric conversions (int, float)
    - String conversions
    - List/dict preservation
    """
    if value is None:
        return None
    
    # handle boolean specially (JSON true/false, string "True"/"False", int 0/1)
    if target_type == bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ('true', '1', 'yes', 'on')
        if isinstance(value, (int, float)):
            return bool(value)
        return False
    
    # handle numeric types
    if target_type == int:
        try:
            return int(float(value))  # Convert through float to handle scientific notation
        except (ValueError, TypeError):
            return 0
    
    if target_type == float:
        try:
            return float(value)
        except (ValueError, TypeError):
            return 0.0
    
    # handle string
    if target_type == str:
        return str(value)
    
    # handle lists and dicts (preserve as-is or convert from string)
    if target_type in (list, dict):
        if isinstance(value, target_type):
            return value
        # Try to parse JSON if it's a string
        if isinstance(value, str):
            try:
                return json.loads(value)
            except json.JSONDecodeError:
                return target_type()  # Return empty container on failure
        return target_type()
    
    # default: return as-is
    return value



def merge_dataclass(target: Any, source_dict: Dict[str, Any], strict: bool = False) -> Any:
    """
    Recursively merge a dictionary into a dataclass instance.
    
    Args:
        target: The target dataclass instance to update
        source_dict: Dictionary with new values
        strict: If True, only update fields that exist in target
    
    Returns:
        Updated target instance
    """
    if not is_dataclass(target):
        return target
    
    # get all field names and their types
    field_info = {f.name: f.type for f in fields(target)}
    
    for key, value in source_dict.items():
        if strict and key not in field_info:
            continue
        
        if key in field_info:
            field_type = field_info[key]
            
            # handle nested dataclasses
            if is_dataclass(field_type) and isinstance(value, dict):
                current_value = getattr(target, key, None)
                if current_value is None:
                    # create new instance if field doesn't exist
                    current_value = field_type()
                setattr(target, key, merge_dataclass(current_value, value, strict))
            else:
                # convert and set value
                converted_value = convert_value_to_type(value, field_type)
                setattr(target, key, converted_value)
    
    return target



# ============================================================================
# SETTINGS DATA CLASSES
# ============================================================================

@dataclass
class HardwareSettings:
    platform_diameter_mm: int = 220        # usable diameter (platform diameter - support)
    ball_diameter_mm: int = 30             # rough ball size, helping the auto HSV calibration to exclude noise
    camera_focus_distance_m: float = 0.4   # camera distance: camera to mirror + mirror to platform (once levelled)
    camera_resolution: Dict[str, int] = field(default_factory=lambda: {"width": 352, "height": 352})



@dataclass
class HomingSettings:
    speed_freq: int = 800
    sg_k_factor: int = 45
    sg_shifts: Dict[str, int] = field(default_factory=lambda: {"A": 0, "B": -5, "C": 0})



@dataclass
class BalanceSettings:
    steps_from_home: int = 220
    shifts: Dict[str, int] = field(default_factory=lambda: {"A": 6, "B": 10, "C": -6})



@dataclass
class MotorSettings:
    reverse: bool = False
    full_steps_per_rev: int = 200
    microsteps: int = 8
    max_speed: int = 65480
    min_speed: int = 5000
    base_speed: int = 55000
    positioning_speed: int = 15000  # speed for precise positioning moves
    homing: HomingSettings = field(default_factory=HomingSettings)
    balance: BalanceSettings = field(default_factory=BalanceSettings)

    @property
    def step_angle_tenths(self) -> float:
        """Calculate step angle in tenths of degrees."""
        return 10.0 * 360.0 / (self.full_steps_per_rev * self.microsteps) 



@dataclass
class ControllerSettings:
    target_cycle_time_ms: int = 60
    balance_angle_tenths: int = 900
    max_tilt_tenths: int = 400
    deadzone_tenths: int = 50
    deadzone_counter_thr: int = 15
    integral_zone_tenths: int = 400
    max_integral: int = 3000



@dataclass
class PIDSettings:
    kp: float = 1.3
    ki: float = 0.0
    kd: float = 1.0



@dataclass
class VisionSettings:
    hsv_lower: List[int] = field(default_factory=lambda: [0, 140, 181])
    hsv_upper: List[int] = field(default_factory=lambda: [9, 237, 247])



@dataclass
class FansSettings:
    enabled: bool = True
    pwm_frequency: int = 100
    pwm_duty_cycle: float = 0.75
    temp_on_celsius: float = 65.0
    temp_hysteresis_celsius: float = 5.0
    check_interval_s: int = 15



@dataclass
class PathsSettings:
    square_side_mm: int = 100
    circle_radius_mm: int = 70
    infinity_size_mm: int = 100
    triangle_side_mm: int = 100
    line_length_mm: int = 120
    tolerance_mm: int = 40
    settle_time_ms: int = 400
    repeats: int = 3
    speed_mm_per_s: float = 150.0



@dataclass
class DisplaySettings:
    frame_skip: int = 10
    video_update_interval_ms: int = 66
    status_update_interval_ms: int = 500



@dataclass
class GpioSettings:
    motor_a: int = 18
    motor_b: int = 20
    motor_c: int = 21
    motors_drv_enable: int = 4
    fans_gpio_pin: int = 19



@dataclass
class RobotSettings:
    """Complete robot settings container."""
    version: str = "1.0"
    last_modified: str = ""
    hardware: HardwareSettings = field(default_factory=HardwareSettings)
    motors: MotorSettings = field(default_factory=MotorSettings)
    controller: ControllerSettings = field(default_factory=ControllerSettings)
    pid: PIDSettings = field(default_factory=PIDSettings)
    vision: VisionSettings = field(default_factory=VisionSettings)
    fans: FansSettings = field(default_factory=FansSettings)
    paths: PathsSettings = field(default_factory=PathsSettings)
    display: DisplaySettings = field(default_factory=DisplaySettings)
    gpio: GpioSettings = field(default_factory=GpioSettings)
    
    def __post_init__(self):
        if not self.last_modified:
            self.last_modified = ctime()




# ============================================================================
# JSON ENHANCED ENCODER
# ============================================================================

class JSONEncoder(json.JSONEncoder):
    """Enhanced JSON encoder that handles dataclasses and custom types."""
    
    def default(self, obj):
        if is_dataclass(obj):
            return asdict(obj)
        if isinstance(obj, (set, tuple)):
            return list(obj)
        return super().default(obj)


# ============================================================================
# SETTINGS MANAGER
# ============================================================================

class SettingsManager:
    """
    Centralized settings manager for MirrorBallBot.
    
    Handles loading/saving settings to JSON, maintains defaults,
    and provides access to all configuration parameters.
    
    Features:
    - Flexible type conversion (handles JSON booleans properly)
    - Automatic schema migration
    - Backup creation on save
    - Validation hooks
    """
    
    DEFAULT_SETTINGS_FILE = "mbb_settings_default.json"
    USER_SETTINGS_FILE = "mbb_settings.json"
    BACKUP_SUFFIX = ".backup"
    
    
    def __init__(self, auto_load: bool = True, verbose: bool = False, 
                 create_backups: bool = True, strict_merge: bool = False):
        """
        Initialize settings manager.
        
        Args:
            auto_load: If True, automatically load settings on init
            verbose: If True, print debug information
            create_backups: If True, create backup files before saving
            strict_merge: If True, only update fields that exist in schema
        """
        self.verbose = verbose
        self.strict_merge = strict_merge
        self.create_backups = create_backups
        self.settings: Optional[RobotSettings] = None
        self._default_settings: Optional[RobotSettings] = None
        
        if auto_load:
            self.load()
    
    
    
    def _get_default_settings_path(self) -> str:
        """Get path to default settings file."""
        # look in current directory first, then script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        default_path = os.path.join(script_dir, self.DEFAULT_SETTINGS_FILE)
        if os.path.exists(default_path):
            return default_path
        
        # fallback to current working directory
        return self.DEFAULT_SETTINGS_FILE
    
    
    
    def _get_user_settings_path(self) -> str:
        """Get path to user settings file."""
        return self.USER_SETTINGS_FILE
    
    
    
    def _get_backup_path(self, file_path: str) -> str:
        """Get backup path for a settings file."""
        return file_path + self.BACKUP_SUFFIX
    
    
    
    def _dict_to_settings(self, data: Dict[str, Any]) -> RobotSettings:
        """Flexibly convert dictionary to RobotSettings dataclass."""
        # create default settings instance
        settings = RobotSettings()
        
        # merge data into settings
        return merge_dataclass(settings, data, strict=self.strict_merge)
    
    
    
    def _settings_to_dict(self, settings: RobotSettings) -> Dict[str, Any]:
        """Convert RobotSettings to dictionary for JSON serialization."""
        result = asdict(settings)
        
        # ensure last_modified is updated
        result["last_modified"] = ctime()
        
        # recursively convert all booleans to strings
        def convert_bools(obj):
            if isinstance(obj, dict):
                return {k: convert_bools(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [convert_bools(item) for item in obj]
            elif isinstance(obj, bool):
                return "True" if obj else "False"
            return obj
        
        return convert_bools(result)
    
    
    
    def load_defaults(self) -> RobotSettings:
        """
        Load default settings from JSON file.
        Creates default file if it doesn't exist.
        """
        default_path = self._get_default_settings_path()
        
        if os.path.exists(default_path):
            try:
                with open(default_path, 'r') as f:
                    data = json.load(f)
                self._default_settings = self._dict_to_settings(data)
                if self.verbose:
                    print(f"Loaded default settings from {default_path}")
                return self._default_settings
            except Exception as e:
                print(f"Error loading default settings: {e}")
        
        # create default settings if file doesn't exist or failed to load
        if self.verbose:
            print(f"Creating new default settings file: {default_path}")
        self._default_settings = RobotSettings()
        self.save_defaults()
        return self._default_settings
    
    
    
    def save_defaults(self) -> bool:
        """Save current defaults to JSON file."""
        default_path = self._get_default_settings_path()
        if self._default_settings is None:
            self._default_settings = RobotSettings()
        
        try:
            with open(default_path, 'w') as f:
                json.dump(self._settings_to_dict(self._default_settings), f, 
                         indent=4, cls=JSONEncoder)
            if self.verbose:
                print(f"Saved default settings to {default_path}")
            return True
        except Exception as e:
            print(f"Error saving default settings: {e}")
            return False
    
    
    
    def _create_backup(self, file_path: str) -> bool:
        """Create a backup of existing settings file."""
        if not self.create_backups:
            return True
        
        if os.path.exists(file_path):
            backup_path = self._get_backup_path(file_path)
            try:
                shutil.copy2(file_path, backup_path)
                if self.verbose:
                    print(f"Created backup: {backup_path}")
                return True
            except Exception as e:
                print(f"Warning: Could not create backup: {e}")
                return False
        return True
    
    
    
    def load(self) -> RobotSettings:
        """
        Load user settings, falling back to defaults if needed.
        
        Returns:
            RobotSettings object
        """
        user_path = self._get_user_settings_path()
        
        # load defaults first
        defaults = self.load_defaults()
        
        if os.path.exists(user_path):
            try:
                with open(user_path, 'r') as f:
                    user_data = json.load(f)
                
                # start with defaults
                self.settings = self._dict_to_settings(self._settings_to_dict(defaults))
                
                # merge user settings (this handles type conversion including booleans)
                self.settings = merge_dataclass(self.settings, user_data, strict=self.strict_merge)
                
                if self.verbose:
                    print(f"Loaded user settings from {user_path}")
                
                # check for version updates
                if user_data.get("version") != self.settings.version:
                    if self.verbose:
                        print(f"Settings version mismatch (old: {user_data.get('version')}, new: {self.settings.version})")
                        print(f"Migrating settings to version {self.settings.version}")
                    self.save()  # Save with updated version and schema
                
            except Exception as e:
                print(f"Error loading user settings: {e}")
                if self.verbose:
                    import traceback
                    traceback.print_exc()
                self.settings = defaults
        else:
            # no user settings file, create from defaults
            if self.verbose:
                print(f"No user settings file found, creating from defaults")
            self.settings = defaults
            self.save()  # save immediately
        
        return self.settings
    
    
    
    def save(self) -> bool:
        """Save current settings to user JSON file."""
        if self.settings is None:
            if self.verbose:
                print("No settings to save")
            return False
        
        user_path = self._get_user_settings_path()
        
        # create backup before saving
        self._create_backup(user_path)
        
        try:
            # ensure last_modified is updated
            self.settings.last_modified = ctime()
            
            with open(user_path, 'w') as f:
                json.dump(self._settings_to_dict(self.settings), f, 
                         indent=4, cls=JSONEncoder)
            if self.verbose:
                print(f"Saved settings to {user_path}")
            return True
        except Exception as e:
            print(f"Error saving settings: {e}")
            return False
    
    def get(self) -> RobotSettings:
        """Get current settings."""
        if self.settings is None:
            self.load()
        return self.settings
    
    
    
    def reload(self) -> RobotSettings:
        """Reload settings from disk."""
        return self.load()
    
    
    
    def update_hsv(self, lower: List[int], upper: List[int]) -> None:
        """Update HSV calibration values and save."""
        if self.settings:
            self.settings.vision.hsv_lower = lower
            self.settings.vision.hsv_upper = upper
            self.save()
            if self.verbose:
                print(f"Updated HSV settings: Lower={lower}, Upper={upper}")
    
    
    
    def update_balance_shifts(self, shift_a: int, shift_b: int, shift_c: int) -> None:
        """Update motor balance shifts and save."""
        if self.settings:
            self.settings.motors.balance.shifts = {"A": shift_a, "B": shift_b, "C": shift_c}
            self.save()
            if self.verbose:
                print(f"Updated balance shifts: A={shift_a}, B={shift_b}, C={shift_c}")
    
    
    
    def get_homing_sg_factor(self) -> int:
        """Get StallGuard sensitivity factor."""
        if self.settings:
            return self.settings.motors.homing.sg_k_factor
        return 45
    
    
    
    def get_homing_sg_shifts(self) -> Dict[str, int]:
        """Get per-motor StallGuard shifts."""
        if self.settings:
            return self.settings.motors.homing.sg_shifts
        return {"A": 0, "B": 0, "C": 0}
    
    
    
    def update_homing_sg_factor(self, value: int) -> None:
        """Update StallGuard sensitivity factor and save."""
        if self.settings:
            self.settings.motors.homing.sg_k_factor = value
            self.save()
            if self.verbose:
                print(f"Updated homing SG factor: {value}")
    
    
    
    def update_homing_sg_shifts(self, shifts: Dict[str, int]) -> None:
        """Update per-motor StallGuard shifts and save."""
        if self.settings:
            self.settings.motors.homing.sg_shifts = shifts
            self.save()
            if self.verbose:
                print(f"Updated homing SG shifts: {shifts}")
    
    
    
    def update_pid(self, kp: float, ki: float, kd: float) -> None:
        """Update PID gains and save."""
        if self.settings:
            self.settings.pid.kp = kp
            self.settings.pid.ki = ki
            self.settings.pid.kd = kd
            self.save()
            if self.verbose:
                print(f"Updated PID: kp={kp}, ki={ki}, kd={kd}")
    
    
    
    def update_deadzone(self, deadzone_tenths: int) -> None:
        """Update deadzone and save."""
        if self.settings:
            self.settings.controller.deadzone_tenths = deadzone_tenths
            self.save()
            if self.verbose:
                print(f"Updated deadzone: {deadzone_tenths/10:.1f}mm")
    
    
    
    def reset_to_defaults(self) -> RobotSettings:
        """Reset user settings to defaults and save."""
        self.settings = self.load_defaults()
        self.save()
        if self.verbose:
            print("Reset settings to defaults")
        return self.settings
    
    
    
    def export_settings(self, filepath: str) -> bool:
        """Export current settings to a file."""
        try:
            with open(filepath, 'w') as f:
                json.dump(self._settings_to_dict(self.get()), f, 
                         indent=4, cls=JSONEncoder)
            if self.verbose:
                print(f"Exported settings to {filepath}")
            return True
        except Exception as e:
            print(f"Error exporting settings: {e}")
            return False
    
    
    
    def validate(self) -> Dict[str, List[str]]:
        """
        Validate current settings.
        
        Returns:
            Dictionary with 'errors' and 'warnings' lists
        """
        errors = []
        warnings = []
        
        if self.settings is None:
            errors.append("No settings loaded")
            return {"errors": errors, "warnings": warnings}
        
        # validate hardware settings
        if self.settings.hardware.platform_diameter_mm <= 0:
            errors.append("Platform diameter must be positive")
        
        if self.settings.hardware.ball_diameter_mm <= 0:
            errors.append("Ball diameter must be positive")
        
        # validate motor settings
        if self.settings.motors.full_steps_per_rev <= 0:
            errors.append("Full steps per revolution must be positive")
        
        if self.settings.motors.microsteps <= 0:
            errors.append("Microsteps must be positive")
        
        # validate PID settings
        if self.settings.pid.kp < 0 or self.settings.pid.ki < 0 or self.settings.pid.kd < 0:
            warnings.append("Negative PID gains may cause instability")
        
        # validate HSV ranges
        if len(self.settings.vision.hsv_lower) != 3 or len(self.settings.vision.hsv_upper) != 3:
            errors.append("HSV settings must have exactly 3 values")
        
        return {"errors": errors, "warnings": warnings}


# ============================================================================
# MAIN TEST
# ============================================================================

if __name__ == "__main__":
    # test the settings manager
    print("Testing Settings Manager with JSON boolean handling...")
    mgr = SettingsManager(verbose=True, create_backups=True)
    settings = mgr.get()
    
    print("\n" + "="*60)
    print("SETTINGS MANAGER TEST")
    print("="*60)
    print(f"Hardware: {settings.hardware.platform_diameter_mm}mm platform, {settings.hardware.ball_diameter_mm}mm ball")
    print(f"Motor reverse: {settings.motors.reverse} (type: {type(settings.motors.reverse).__name__})")
    print(f"Fans enabled: {settings.fans.enabled} (type: {type(settings.fans.enabled).__name__})")
    print(f"Display show_fps: {settings.display.show_fps} (type: {type(settings.display.show_fps).__name__})")
    print(f"Motor step angle: {settings.motors.step_angle_tenths:.3f} tenths/step")
    print(f"PID: kp={settings.pid.kp}, ki={settings.pid.ki}, kd={settings.pid.kd}")
    print(f"HSV: Lower={settings.vision.hsv_lower}, Upper={settings.vision.hsv_upper}")
    
    # validate settings
    validation = mgr.validate()
    if validation["errors"]:
        print(f"\nErrors found: {validation['errors']}")
    if validation["warnings"]:
        print(f"\nWarnings: {validation['warnings']}")
    
    # test boolean update
    print("\nTesting boolean toggling...")
    mgr.settings.fans.enabled = not mgr.settings.fans.enabled
    mgr.settings.display.show_fps = not mgr.settings.display.show_fps
    mgr.save()
    
    print(f"Fans enabled toggled to: {mgr.settings.fans.enabled}")
    print(f"Show FPS toggled to: {mgr.settings.display.show_fps}")
    
    # test reset
    print("\nResetting to defaults...")
    mgr.reset_to_defaults()
    print(f"After reset - Fans enabled: {mgr.settings.fans.enabled}")
    print(f"After reset - Show FPS: {mgr.settings.display.show_fps}")
    
    print("\nSettings manager test complete")
