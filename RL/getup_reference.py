import numpy as np


GETUP_SEQUENCE = np.array(
    [
        [1.532655, -0.450321],
        [1.432846, 0.075935],
        [-0.570885, 1.072732],
        [-0.713748, 0.631464],
        [0.0, 0.0],
    ],
    dtype=np.float64,
)


POSE_SEQUENCE_SIGNS = {
    "roll_pos": np.array([1.0, 1.0], dtype=np.float64),
    "roll_neg": np.array([-1.0, 1.0], dtype=np.float64),
}


def getup_sequence_for_pose(pose):
    if pose == "roll_pos":
        return np.array(
            [
                [1.524535, 1.483570],
                [1.550000, 0.069879],
                [0.0, 0.0],
            ],
            dtype=np.float64,
        )
    if pose == "roll_neg":
        return np.array(
            [
                [-1.55, -1.55],
                [-1.55, 0.0],
                [0.0, 0.0],
            ],
            dtype=np.float64,
        )
    if pose == "pitch_pos":
        return np.array(
            [
                [0.930212, 1.488389],
                [1.489351, 1.473381],
                [1.394132, 1.327853],
                [0.224632, 0.997530],
                [0.0, 0.0],
            ],
            dtype=np.float64,
        )
    if pose == "pitch_neg":
        return np.array(
            [
                [0.249033, -1.151603],
                [1.462818, -1.341321],
                [1.042669, 1.015654],
                [0.024410, 1.435544],
                [0.0, 0.0],
            ],
            dtype=np.float64,
        )
    return GETUP_SEQUENCE * POSE_SEQUENCE_SIGNS[pose]
