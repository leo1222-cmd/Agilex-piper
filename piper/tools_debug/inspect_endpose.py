import time
from piper_sdk import C_PiperInterface


def dump_obj(name, obj):
    print(f"\n========== {name} ==========")
    print("type:", type(obj))
    print("str :", obj)

    print("\n----- dir fields -----")
    for k in dir(obj):
        if k.startswith("_"):
            continue
        try:
            v = getattr(obj, k)
            if callable(v):
                continue
            print(f"{k}: {v}")
        except Exception as e:
            print(f"{k}: <ERR {e}>")


def main():
    can = "can1"
    piper = C_PiperInterface(can)
    piper.ConnectPort()

    print("[INFO] connected to", can)
    time.sleep(1.0)

    for i in range(5):
        print(f"\n\n################ sample {i+1} ################")

        try:
            end_pose = piper.GetArmEndPoseMsgs()
            dump_obj("GetArmEndPoseMsgs", end_pose)
        except Exception as e:
            print("[ERR] GetArmEndPoseMsgs failed:", e)

        try:
            joint = piper.GetArmJointMsgs()
            dump_obj("GetArmJointMsgs", joint)
        except Exception as e:
            print("[ERR] GetArmJointMsgs failed:", e)

        time.sleep(0.5)


if __name__ == "__main__":
    main()
