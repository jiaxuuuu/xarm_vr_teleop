from xarm.wrapper import XArmAPI
import time

def check_robot_status():
    """Check robot status and help diagnose issues"""
    print("Checking xArm robot status...")
    
    # Connect to robot
    arm = XArmAPI("192.168.1.226")
    
    try:
        # Get basic connection info
        print(f"Robot IP: 192.168.1.226")
        print(f"SDK Version: {arm.version}")
        
        # Check connection
        code, state = arm.get_state()
        print(f"\nConnection result: code={code}")
        
        if code != 0:
            print("❌ Failed to connect to robot")
            return
            
        print("✅ Connected successfully")
        
        # Check robot state
        states = {
            0: 'READY',
            1: 'SUSPENDED', 
            2: 'STOPPED',
            3: 'PLAYING',
            4: 'PAUSED'
        }
        print(f"Robot state: {states.get(state, f'UNKNOWN({state})')}")
        
        # Check for errors
        code, error_code = arm.get_err_warn_code()
        print(f"Error code: {error_code}")
        
        # Check servo status
        code, servo_status = arm.get_servo_angle()
        if code == 0:
            print("✅ Can read joint angles - servos responding")
        else:
            print(f"❌ Cannot read joint angles, code: {code}")
            
        # Check current position
        code, position = arm.get_position()
        if code == 0:
            print(f"✅ Current position: {position[:3]} mm")
            print(f"✅ Current orientation: {position[3:6]} (rad)")
        else:
            print(f"❌ Cannot read position, code: {code}")
            
        # Try to enable motion (test)
        print("\nTesting motion enable...")
        ret = arm.motion_enable(enable=True)
        if ret == 0:
            print("✅ Motion can be enabled")
        else:
            print(f"❌ Motion enable failed, code: {ret}")
            if ret == 9:
                print("   → Emergency stop is active!")
                print("   → Check physical emergency stop buttons")
                print("   → Check robot collision sensors") 
                print("   → Check if robot is in safe position")
                
        # Try to set mode (test)
        print("Testing mode setting...")
        ret = arm.set_mode(0)
        if ret == 0:
            print("✅ Mode can be set")
        else:
            print(f"❌ Set mode failed, code: {ret}")
            
        # Try to set state (test)
        print("Testing state setting...")
        ret = arm.set_state(0)
        if ret == 0:
            print("✅ State can be set to READY")
        else:
            print(f"❌ Set state failed, code: {ret}")
            
        print("\n" + "="*50)
        print("DIAGNOSIS:")
        if ret == 9:
            print("🚨 EMERGENCY STOP DETECTED")
            print("Solutions:")
            print("1. Check all physical emergency stop buttons")
            print("2. Reset emergency stop buttons by twisting and releasing")
            print("3. Check robot is not in collision")
            print("4. Restart robot controller if needed")
            print("5. Check robot cables and connections")
        else:
            print("✅ Robot appears to be in good state")
            print("Ready for normal operation")
            
    except Exception as e:
        print(f"❌ Exception occurred: {e}")
    finally:
        arm.disconnect()

if __name__ == "__main__":
    check_robot_status()