import sys

print("Testing PyBullet import...")
try:
    import pybullet as p
    import pybullet_data
    print("SUCCESS: PyBullet and pybullet_data imported successfully!")
    cid = p.connect(p.DIRECT)
    print(f"SUCCESS: Connected to PyBullet physics server (id={cid})")
    p.disconnect(cid)
    print("PyBullet is fully functional and ready for 3D visual simulation!")
    sys.exit(0)
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
