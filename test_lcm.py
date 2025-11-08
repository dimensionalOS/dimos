import lcm
try:
    l = lcm.LCM()
    print("Successfully created LCM object")
except Exception as e:
    print(f"Error creating LCM object: {e}")
