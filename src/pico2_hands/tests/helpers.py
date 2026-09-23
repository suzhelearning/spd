"""Offline synthetic frames and native-worker availability for backend tests."""
from dataclasses import replace

import numpy as np
from scipy.spatial.transform import Rotation

from pico2_hands.reference.pico import parse_pico_packet
from pico2_hands.resources import native_executable, ResourceNotFound
from pico2_hands.tests.test_pico_hand_tracking import _packet


def dls_available():
    try:
        native_executable("pico2_dls_worker")
    except ResourceNotFound:
        return False
    return True


def frame(height=1.62, stamp=1_000_000_000, yaw=0., generation=1, move=None):
    raw=parse_pico_packet(_packet(),receiver_instance_id="synthetic",connection_generation=generation,
        receiver_frame_sequence=stamp,received_timestamp_ns=stamp)
    r=Rotation.from_euler("z",yaw)
    head=r.apply([0.,0,height*.93])
    hands={}
    for i,side in enumerate(("left","right")):
        wrist=r.apply([height*(.155882+.152941), (1 if i==0 else -1)*height*.1828/2, height*.80])
        if move is not None:
            wrist+=r.apply(np.array(move)*height)
        hand=raw.hands[side]
        joints=list(hand.joints)
        for j in range(26):
            joints[j]=replace(joints[j],pose=np.r_[wrist+r.apply([.05,j*.0001,0]),r.as_quat()])
        joints[12]=replace(joints[12],pose=np.r_[wrist+r.apply([height*.05,0,0]),r.as_quat()])
        hands[side]=replace(hand,wrist_pose=np.r_[wrist,r.as_quat()],joints=tuple(joints))
    return replace(raw,source_timestamp_ms=stamp//1_000_000,head_pose=np.r_[head,r.as_quat()],hands=hands)
