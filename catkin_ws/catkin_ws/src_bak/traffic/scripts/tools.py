import cv2
import json
import numpy as np
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]


def constraint(value, lb, ub):
    if value > ub:
        return ub
    if value < lb:
        return lb
    return value


def remap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def calc_pose(corners):
    x, y = corners.mean(axis=0)
    head = (corners[0] + corners[1]) / 2
    rear = (corners[2] + corners[3]) / 2
    theta = np.arctan2(head[1] - rear[1], head[0] - rear[0])
    sz = np.linalg.norm(head - rear)
    return x, y, theta, sz


def calc_paths(points, scale):
    crossing1_1 = points[1][0, :]
    corner_1 = points[1][1, :]
    crossing2_1 = points[1][2, :]
    r_1 = 0.2 * abs(corner_1[1] - crossing2_1[1])
    path1_1 = np.linspace(crossing1_1, [crossing1_1[0], corner_1[1] + r_1], int(8 * scale), endpoint=False)
    angles1_1 = np.linspace(np.pi, np.pi / 2, int(5 * scale), endpoint=False)
    path2_1 = np.array([crossing1_1[0] + r_1 + r_1 * np.cos(angles1_1), corner_1[1] + r_1 - r_1 * np.sin(angles1_1)]).T
    path3_1 = np.linspace(
        [crossing1_1[0] + r_1, corner_1[1]], [corner_1[0] - r_1, corner_1[1]], int(8 * scale), endpoint=False
    )
    angles2_1 = np.linspace(np.pi / 2, 0, int(5 * scale), endpoint=False)
    path4_1 = np.array([corner_1[0] - r_1 + r_1 * np.cos(angles2_1), corner_1[1] + r_1 - r_1 * np.sin(angles2_1)]).T
    path5_1 = np.linspace(
        [corner_1[0], corner_1[1] + r_1], [corner_1[0], crossing2_1[1] - r_1], int(10 * scale), endpoint=False
    )
    angles3_1 = np.linspace(0, -np.pi / 2, int(5 * scale), endpoint=False)
    path6_1 = np.array([corner_1[0] - r_1 + r_1 * np.cos(angles3_1), crossing2_1[1] - r_1 - r_1 * np.sin(angles3_1)]).T
    path7_1 = np.linspace([corner_1[0] - r_1, crossing2_1[1]], crossing2_1, int(8 * scale))
    lane_path_1 = np.vstack([path1_1, path2_1, path3_1, path4_1, path5_1, path6_1, path7_1])

    crossing1_3 = points[3][0, :]
    corner_3 = points[3][1, :]
    crossing2_3 = points[3][2, :]

    r_3 = 0.2 * abs(corner_3[1] - crossing2_3[1])
    path1_3 = np.linspace(crossing1_3, [crossing1_3[0], corner_3[1] - r_3], int(8 * scale), endpoint=False)
    angles1_3 = np.linspace(0, -np.pi / 2, int(5 * scale), endpoint=False)
    path2_3 = np.array([crossing1_3[0] - r_3 + r_3 * np.cos(angles1_3), corner_3[1] - r_3 - r_3 * np.sin(angles1_3)]).T
    path3_3 = np.linspace(
        [crossing1_3[0] - r_3, corner_3[1]], [corner_3[0] + r_3, corner_3[1]], int(8 * scale), endpoint=False
    )
    angles2_3 = np.linspace(-np.pi / 2, -np.pi, int(5 * scale), endpoint=False)
    path4_3 = np.array([corner_3[0] + r_3 + r_3 * np.cos(angles2_3), corner_3[1] - r_3 - r_3 * np.sin(angles2_3)]).T
    path5_3 = np.linspace(
        [corner_3[0], corner_3[1] - r_1], [corner_3[0], crossing2_3[1] + r_1], int(10 * scale), endpoint=False
    )
    angles3_3 = np.linspace(np.pi, np.pi / 2, int(5 * scale), endpoint=False)
    path6_3 = np.array([corner_3[0] + r_3 + r_3 * np.cos(angles3_3), crossing2_3[1] + r_3 - r_3 * np.sin(angles3_3)]).T
    path7_3 = np.linspace([corner_3[0] + r_1, crossing2_3[1]], crossing2_3, int(8 * scale))
    lane_path_3 = np.vstack([path1_3, path2_3, path3_3, path4_3, path5_3, path6_3, path7_3])

    crossing1_2 = points[2][0, :]
    entrance_2 = points[2][1, :]
    spot1_2 = points[2][2, :]
    spot2_2 = points[2][3, :]
    exit_2 = points[2][4, :]
    crossing2_2 = points[2][5, :]
    switch1_2 = [spot1_2[0], exit_2[1] + 0.1 * abs(exit_2[1] - spot1_2[1])]
    switch2_2 = [spot2_2[0], exit_2[1] + 0.1 * abs(exit_2[1] - spot2_2[1])]

    r1_2 = abs(crossing1_2[0] - entrance_2[0]) * 0.2
    path1_2 = np.linspace(crossing1_2, [entrance_2[0] + r1_2, crossing1_2[1]], int(15 * scale), endpoint=False)
    angles1_2 = np.linspace(-np.pi / 2, -np.pi, int(5 * scale), endpoint=False)
    path2_2 = np.array(
        [entrance_2[0] + r1_2 + r1_2 * np.cos(angles1_2), crossing1_2[1] - r1_2 - r1_2 * np.sin(angles1_2)]).T
    path3_2 = np.linspace([entrance_2[0], crossing1_2[1] - r1_2], entrance_2, int(5 * scale))
    lane_path_2 = np.vstack([path1_2, path2_2, path3_2])

    delta_x1_2, delta_y1_2 = abs(entrance_2[0] - switch1_2[0]), abs(entrance_2[1] - switch1_2[1]) * 0.8
    delta_x2_2, delta_y2_2 = abs(entrance_2[0] - switch2_2[0]), abs(entrance_2[1] - switch2_2[1]) * 0.8
    y1_2 = np.linspace(0, delta_y1_2, int(20 * scale))
    y2_2 = np.linspace(0, delta_y2_2, int(25 * scale))
    x1_2 = delta_x1_2 / 2 * (np.cos(y1_2 / delta_y1_2 * np.pi) - 1)
    x2_2 = delta_x2_2 / 2 * (np.cos(y2_2 / delta_y2_2 * np.pi) - 1)
    path4_2 = np.array([entrance_2[0] - x1_2, entrance_2[1] - y1_2]).T
    path5_2 = np.array([entrance_2[0] - x2_2, entrance_2[1] - y2_2]).T
    path6_2 = np.linspace(
        switch1_2, [switch1_2[0], entrance_2[1] - delta_y1_2], int(4 * scale), endpoint=False
    )
    path7_2 = np.linspace(
        switch2_2, [switch2_2[0], entrance_2[1] - delta_y2_2], int(4 * scale), endpoint=False
    )
    spot1_in_fw_path_2 = np.vstack([path4_2, np.flipud(path6_2)])
    spot2_in_fw_path_2 = np.vstack([path5_2, np.flipud(path7_2)])

    spot1_in_bw_path_2 = np.linspace(switch1_2, spot1_2, int(15 * scale))
    spot2_in_bw_path_2 = np.linspace(switch2_2, spot2_2, int(15 * scale))

    r2_2 = abs(exit_2[0] - spot2_2[0]) / 3
    angles2_2 = np.linspace(np.pi, np.pi / 2, int(5 * scale), endpoint=False)
    path8_2 = np.linspace(
        spot1_2, [spot1_2[0], exit_2[1] + r2_2], int(10 * scale), endpoint=False
    )
    path9_2 = np.array([spot1_2[0] + r2_2 + r2_2 * np.cos(angles2_2), exit_2[1] + r2_2 - r2_2 * np.sin(angles2_2)]).T
    path10_2 = np.linspace(
        [spot1_2[0] + r2_2, exit_2[1]], [exit_2[0] - r2_2, exit_2[1]], int(5 * scale), endpoint=False
    )
    angles3_2 = np.linspace(np.pi / 2, 0, int(5), endpoint=False)
    path11_2 = np.array([exit_2[0] - r2_2 + r2_2 * np.cos(angles3_2), exit_2[1] + r2_2 - r2_2 * np.sin(angles3_2)]).T
    path12_2 = np.linspace(
        [exit_2[0], exit_2[1] + r2_2], crossing2_2, int(10 * scale)
    )
    spot1_out_path_2 = np.vstack([path8_2, path9_2, path10_2, path11_2, path12_2])

    path13_2 = np.linspace(
        spot2_2, [spot2_2[0], exit_2[1] + r2_2], int(10 * scale), endpoint=False
    )
    path14_2 = np.array([spot2_2[0] + r2_2 + r2_2 * np.cos(angles2_2), exit_2[1] + r2_2 - r2_2 * np.sin(angles2_2)]).T
    path15_2 = np.linspace(
        [spot2_2[0] + r2_2, exit_2[1]], [exit_2[0] - r2_2, exit_2[1]], int(5 * scale), endpoint=False
    )
    spot2_out_path_2 = np.vstack([path13_2, path14_2, path15_2, path11_2, path12_2])

    crossing1_0 = points[0][0, :]
    entrance_0 = points[0][1, :]
    spot1_0 = points[0][2, :]
    spot2_0 = points[0][3, :]
    exit_0 = points[0][4, :]
    crossing2_0 = points[0][5, :]
    switch1_0 = [spot1_0[0], exit_0[1] - 0.1 * abs(exit_0[1] - spot1_0[1])]
    switch2_0 = [spot2_0[0], exit_0[1] - 0.1 * abs(exit_0[1] - spot2_0[1])]

    r1_0 = abs(crossing1_0[0] - entrance_0[0]) * 0.2
    path1_0 = np.linspace(crossing1_0, [entrance_0[0] - r1_0, crossing1_0[1]], int(15 * scale), endpoint=False)
    angles1_0 = np.linspace(np.pi / 2, 0, int(5 * scale), endpoint=False)
    path2_0 = np.array(
        [entrance_0[0] - r1_0 + r1_0 * np.cos(angles1_0), crossing1_0[1] + r1_0 - r1_0 * np.sin(angles1_0)]).T
    path3_0 = np.linspace([entrance_0[0], crossing1_0[1] + r1_0], entrance_0, int(5 * scale))
    lane_path_0 = np.vstack([path1_0, path2_0, path3_0])

    delta_x1_0, delta_y1_0 = abs(entrance_0[0] - switch1_0[0]), abs(entrance_0[1] - switch1_0[1]) * 0.8
    delta_x2_0, delta_y2_0 = abs(entrance_0[0] - switch2_0[0]), abs(entrance_0[1] - switch2_0[1]) * 0.8
    y1_0 = np.linspace(0, delta_y1_0, int(20 * scale))
    y2_0 = np.linspace(0, delta_y2_0, int(25 * scale))
    x1_0 = delta_x1_0 / 2 * (np.cos(y1_0 / delta_y1_0 * np.pi) - 1)
    x2_0 = delta_x2_0 / 2 * (np.cos(y2_0 / delta_y2_0 * np.pi) - 1)
    path4_0 = np.array([entrance_0[0] + x1_0, entrance_0[1] + y1_0]).T
    path5_0 = np.array([entrance_0[0] + x2_0, entrance_0[1] + y2_0]).T
    path6_0 = np.linspace(
        switch1_0, [switch1_0[0], entrance_0[1] + delta_y1_0], int(4 * scale), endpoint=False
    )
    path7_0 = np.linspace(
        switch2_0, [switch2_0[0], entrance_0[1] + delta_y2_0], int(4 * scale), endpoint=False
    )
    spot1_in_fw_path_0 = np.vstack([path4_0, np.flipud(path6_0)])
    spot2_in_fw_path_0 = np.vstack([path5_0, np.flipud(path7_0)])

    spot1_in_bw_path_0 = np.linspace(switch1_0, spot1_0, int(15 * scale))
    spot2_in_bw_path_0 = np.linspace(switch2_0, spot2_0, int(15 * scale))

    r2_0 = abs(exit_0[0] - spot2_0[0]) / 3
    angles2_0 = np.linspace(0, -np.pi / 2, int(5 * scale), endpoint=False)
    path8_0 = np.linspace(
        spot1_0, [spot1_0[0], exit_0[1] - r2_0], int(10 * scale), endpoint=False
    )
    path9_0 = np.array([spot1_0[0] - r2_0 + r2_0 * np.cos(angles2_0), exit_0[1] - r2_0 - r2_0 * np.sin(angles2_0)]).T
    path10_0 = np.linspace(
        [spot1_0[0] - r2_0, exit_0[1]], [exit_0[0] + r2_0, exit_0[1]], int(5 * scale), endpoint=False
    )
    angles3_0 = np.linspace(-np.pi / 2, -np.pi, int(5 * scale), endpoint=False)
    path11_0 = np.array([exit_0[0] + r2_0 + r2_0 * np.cos(angles3_0), exit_0[1] - r2_0 - r2_0 * np.sin(angles3_0)]).T
    path12_0 = np.linspace(
        [exit_0[0], exit_0[1] - r2_0], crossing2_0, int(10 * scale)
    )
    spot1_out_path_0 = np.vstack([path8_0, path9_0, path10_0, path11_0, path12_0])

    path13_0 = np.linspace(
        spot2_0, [spot2_0[0], exit_0[1] - r2_0], int(10 * scale), endpoint=False
    )
    path14_0 = np.array([spot2_0[0] - r2_0 + r2_0 * np.cos(angles2_0), exit_0[1] - r2_0 - r2_0 * np.sin(angles2_0)]).T
    path15_0 = np.linspace(
        [spot2_0[0] - r2_0, exit_0[1]], [exit_0[0] + r2_0, exit_0[1]], int(5 * scale), endpoint=False
    )
    spot2_out_path_0 = np.vstack([path13_0, path14_0, path15_0, path11_0, path12_0])

    return {
        'lane0': lane_path_0,
        'spot1_in_fw0': spot1_in_fw_path_0,
        'spot1_in_bw0': spot1_in_bw_path_0,
        'spot1_out0': spot1_out_path_0,
        'spot2_in_fw0': spot2_in_fw_path_0,
        'spot2_in_bw0': spot2_in_bw_path_0,
        'spot2_out0': spot2_out_path_0,
        'lane1': lane_path_1,
        'lane2': lane_path_2,
        'spot1_in_fw2': spot1_in_fw_path_2,
        'spot1_in_bw2': spot1_in_bw_path_2,
        'spot1_out2': spot1_out_path_2,
        'spot2_in_fw2': spot2_in_fw_path_2,
        'spot2_in_bw2': spot2_in_bw_path_2,
        'spot2_out2': spot2_out_path_2,
        'lane3': lane_path_3,
    }


def save_paths_to_json(paths):
    paths_copy = {}
    for key in paths.keys():
        paths_copy[key] = paths[key].tolist()
    with open(ROOT / 'paths.json', 'w') as file:
        json.dump(paths_copy, file, indent=4)


def save_matrices_to_json(matrices):
    matrices_copy = {}
    for key in matrices.keys():
        matrices_copy[key] = matrices[key].tolist()
    with open(ROOT / 'matrices.json', 'w') as file:
        json.dump(matrices_copy, file, indent=4)


def load_paths_from_json():
    with open(ROOT / 'paths.json', 'r') as file:
        paths = json.load(file)
    for key in paths.keys():
        paths[key] = np.array(paths[key])
    return paths


def load_matrices_from_json():
    with open(ROOT / 'matrices.json', 'r') as file:
        matrices = json.load(file)
    for key in matrices.keys():
        matrices[key] = np.array(matrices[key])
    return matrices


def preprocess(images, img_sz, recalibrate=False, rechoose=False):
    if recalibrate:
        matrices = {}
        corner_positions = ['zuo shang', 'zuo xia', 'you xia', 'you shang']

        for i, image in enumerate(images):
            corners = []

            def get_corners(event, x, y, flags, param):
                nonlocal corners
                if event == cv2.EVENT_LBUTTONDBLCLK:
                    corners.append([x, y])

            cv2.namedWindow('camera ' + str(i))
            cv2.setMouseCallback('camera ' + str(i), get_corners)
            while len(corners) < 4:
                image_copy = image.copy()
                cv2.putText(
                    image_copy, f'double click {corner_positions[len(corners)]} corner', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2
                )
                cv2.imshow('camera ' + str(i), image_copy)
                cv2.waitKey(1)
            cv2.destroyAllWindows()
            matrix = cv2.getPerspectiveTransform(
                np.array(corners, dtype=np.float32),
                np.array([[0, 0], [0, img_sz], [img_sz, img_sz], [img_sz, 0]], dtype=np.float32)
            )
            matrices['cam' + str(i)] = matrix
        save_matrices_to_json(matrices)

    matrices = load_matrices_from_json()
    if recalibrate or rechoose:
        for i, image in enumerate(images):
            images[i] = cv2.warpPerspective(image, matrices['cam' + str(i)], (img_sz, img_sz))
        top = np.hstack([images[2], images[1]])
        bottom = np.hstack([images[3], images[0]])
        image = np.vstack([top, bottom])

        i = 0
        count = 0
        points = {k: [] for k in range(4)}

        def get_points(event, x, y, flags, param):
            nonlocal points, i, count
            if event == cv2.EVENT_LBUTTONDBLCLK:
                points[i].append([x, y])
                count += 1

        while True:
            cv2.namedWindow('camera')
            cv2.setMouseCallback('camera', get_points, {'i': i, 'count': count})
            image_copy = image.copy()
            cv2.putText(
                image_copy, f'double click point {i}-{count}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2
            )
            if (i in [0, 2] and count == 6) or (i == 1 and count == 3):
                i += 1
                count = 0
            if i == 3 and count == 3:
                break
            cv2.imshow('camera', image_copy)
            cv2.waitKey(1)
        cv2.destroyAllWindows()
        for key in points.keys():
            points[key] = np.array(points[key])
        paths = calc_paths(points, img_sz / 500)

        save_paths_to_json(paths)
    else:
        paths = load_paths_from_json()
    return matrices, paths


def check_pos(point1, point2, distance):
    if np.linalg.norm(point1 - point2) < distance:
        return True
    else:
        return False


def check_angle(angle1, angle2):
    if abs(remap_angle(angle1 - angle2)) < np.pi / 12:
        return True
    else:
        return False


def init_pos_check(paths, cars, scale):
    init_pos_up = [
        paths['spot1_out0'][-1, :], paths['lane1'][0, :], paths['lane2'][-1, :],
        paths['spot1_out2'][0, :], paths['spot2_out2'][0, :]
    ]
    init_pos_left = [paths['lane1'][-1, :], paths['lane2'][0, :]]
    init_pos_down = [
        paths['lane0'][-1, :], paths['spot1_out0'][0, :], paths['spot2_out0'][0, :],
        paths['spot1_out2'][-1, :], paths['lane3'][0, :]
    ]
    init_pos_right = [paths['lane0'][0, :], paths['lane3'][-1, :]]
    for key in cars.keys():
        x, y, theta = cars[key]
        if check_angle(theta, np.pi / 2):  # down
            flag = False
            for init_pos in init_pos_down:
                if check_pos(np.array([x, y]), init_pos, 30 * scale):
                    flag = True
            if not flag:
                return False
        elif check_angle(theta, 0):  # right
            flag = False
            for init_pos in init_pos_right:
                if check_pos(np.array([x, y]), init_pos, 30 * scale):
                    flag = True
            if not flag:
                return False
        elif check_angle(theta, -np.pi / 2):  # up
            flag = False
            for init_pos in init_pos_up:
                if check_pos(np.array([x, y]), init_pos, 30 * scale):
                    flag = True
            if not flag:
                return False
        elif check_angle(theta, np.pi):  # left
            flag = False
            for init_pos in init_pos_left:
                if check_pos(np.array([x, y]), init_pos, 30 * scale):
                    flag = True
            if not flag:
                return False
        else:
            return False
    return True


class StateMachine:
    def __init__(self, init_pose, paths, img_sz):
        x, y, _ = init_pose
        self.paths = {
            '1_0': None,
            '2_0': paths['lane0'],
            '3_0': None,
            '4-1_0': paths['spot1_in_fw0'],
            '4-2_0': paths['spot2_in_fw0'],
            '5-1_0': paths['spot1_in_bw0'],
            '5-2_0': paths['spot2_in_bw0'],
            '6-1_0': None,
            '6-2_0': None,
            '7-1_0': paths['spot1_out0'],
            '7-2_0': paths['spot2_out0'],
            '8_0': None,
            '9_0': paths['lane1'][0, :].reshape(-1, 2),
            '10_0': None,
            '11_0': paths['lane1'],
            '12_0': None,
            '13_0': paths['lane2'][0, :].reshape(-1, 2),
            '1_1': None,
            '2_1': paths['lane2'],
            '3_1': None,
            '4-1_1': paths['spot1_in_fw2'],
            '4-2_1': paths['spot2_in_fw2'],
            '5-1_1': paths['spot1_in_bw2'],
            '5-2_1': paths['spot2_in_bw2'],
            '6-1_1': None,
            '6-2_1': None,
            '7-1_1': paths['spot1_out2'],
            '7-2_1': paths['spot2_out2'],
            '8_1': None,
            '9_1': paths['lane3'][0, :].reshape(-1, 2),
            '10_1': None,
            '11_1': paths['lane3'],
            '12_1': None,
            '13_1': paths['lane0'][0, :].reshape(-1, 2),
        }
        self.brake_flag = False
        self.brake_count = 0

        self.img_sz = img_sz
        self.scale = img_sz / 500
        self.fw_controller = PIDController(2, 0, 4)
        self.bw_controller = PIDController(3, 0, 5)

        if check_pos(np.array([x, y]), paths['lane0'][0, :], 30 * self.scale):
            self.state = '1_0'
        if check_pos(np.array([x, y]), paths['lane0'][-1, :], 30 * self.scale):
            self.state = '3_0'
        if check_pos(np.array([x, y]), paths['spot1_out0'][0, :], 30 * self.scale):
            self.state = '6-1_0'
        if check_pos(np.array([x, y]), paths['spot2_out0'][0, :], 30 * self.scale):
            self.state = '6-2_0'
        if check_pos(np.array([x, y]), paths['spot1_out0'][-1, :], 30 * self.scale):
            self.state = '8_0'
        if check_pos(np.array([x, y]), paths['lane1'][0, :], 30 * self.scale):
            self.state = '10_0'
        if check_pos(np.array([x, y]), paths['lane1'][-1, :], 30 * self.scale):
            self.state = '12_0'
        if check_pos(np.array([x, y]), paths['lane2'][0, :], 30 * self.scale):
            self.state = '1_1'
        if check_pos(np.array([x, y]), paths['lane2'][-1, :], 30 * self.scale):
            self.state = '3_1'
        if check_pos(np.array([x, y]), paths['spot1_out2'][0, :], 30 * self.scale):
            self.state = '6-1_1'
        if check_pos(np.array([x, y]), paths['spot2_out2'][0, :], 30 * self.scale):
            self.state = '6-2_1'
        if check_pos(np.array([x, y]), paths['spot1_out2'][-1, :], 30 * self.scale):
            self.state = '8_1'
        if check_pos(np.array([x, y]), paths['lane3'][0, :], 30 * self.scale):
            self.state = '10_1'
        if check_pos(np.array([x, y]), paths['lane3'][-1, :], 30 * self.scale):
            self.state = '12_1'

    def update(self, pose, states):
        x, y, theta = pose
        state, suffix = self.state.split('_')
        if state.split('-')[0] == '1':
            if not {s + '_' + suffix for s in ['2', '3']} & set(states):
                self.state = '2_' + suffix
        elif state.split('-')[0] == '2':
            if check_pos(np.array([x, y]), self.paths[self.state][-1, :], 30 * self.scale):
                self.state = '3_' + suffix
        elif state.split('-')[0] == '3':
            if not {s + '_' + suffix for s in ['4-1', '4-2', '5-1', '5-2', '7-1', '7-2']} & set(states):
                if '6-1_' + suffix not in states:
                    self.state = '4-1_' + suffix
                elif '6-2_' + suffix not in states:
                    self.state = '4-2_' + suffix
        elif state.split('-')[0] == '4':
            if check_pos(np.array([x, y]), self.paths[self.state][-1, :], 30 * self.scale):
                self.state = '5' + state[1:] + '_' + suffix
        elif state.split('-')[0] == '5':
            if abs(self.paths[self.state][-1, 1] - y) < 30 * self.scale:
                self.state = '6' + state[1:] + '_' + suffix
                self.brake_flag, self.brake_count = True, 0
        elif state.split('-')[0] == '6':
            if not {s + '_' + suffix for s in ['4-1', '4-2', '5-1', '5-2', '7-1', '7-2', '8']} & set(states):
                if state[-1] == '1':
                    self.state = '7-1_' + suffix
                else:  # 2
                    self.state = '7-2_' + suffix
                self.brake_flag, self.brake_count = False, 0
        elif state.split('-')[0] == '7':
            if check_pos(np.array([x, y]), self.paths[self.state][-1, :], 30 * self.scale):
                self.state = '8_' + suffix
        elif state.split('-')[0] == '8':
            if not {'9_' + suffix, '13_0', '13_1', '10_' + suffix} & set(states):
                self.state = '9_' + suffix
        elif state.split('-')[0] == '9':
            if check_pos(np.array([x, y]), self.paths[self.state][-1, :], 30 * self.scale):
                self.state = '10_' + suffix
        elif state.split('-')[0] == '10':
            if not {s + '_' + suffix for s in ['11', '12']} & set(states):
                self.state = '11_' + suffix
        elif state.split('-')[0] == '11':
            if check_pos(np.array([x, y]), self.paths[self.state][-1, :], 30 * self.scale):
                self.state = '12_' + suffix
        elif state.split('-')[0] == '12':
            if not {'9_0', '9_1', '13_' + suffix, '1_' + str(1 - int(suffix))} & set(states):
                self.state = '13_' + suffix
        else:  # 13
            if check_pos(np.array([x, y]), self.paths[self.state][-1, :], 30 * self.scale):
                self.state = '1_' + str(1 - int(suffix))

        path = self.paths[self.state]
        if path is None:
            if self.brake_flag:
                self.brake_count += 1
                if self.brake_count >= 10:
                    self.brake_flag = False
                return [90, 1, 20]
            else:
                return [90, 0, 0]
        else:
            direction = 0 if state.split('-')[0] == '5' else 1
            theta = remap_angle(-theta + np.pi) if direction == 0 else -theta

            distance = np.linalg.norm(np.array([[x, y]]) - path, axis=1)
            closest_idx = np.argmin(distance)
            try:
                xd, yd = path[closest_idx + (3 if direction == 1 else 4), :]
                d = distance[closest_idx + (3 if direction == 1 else 4)]
            except IndexError:
                xd, yd = path[-1, :]
                d = distance[-1]
            y = self.img_sz - y
            yd = self.img_sz - yd

            theta_d = np.arctan2(yd - y, xd - x)
            theta_e = remap_angle(theta_d - theta)
            u = self.fw_controller.output(theta_e) if direction == 1 else self.bw_controller.output(-theta_e)
            angle = 90 + 15 * np.tanh(u)

            speed = 14 * np.tanh(0.05 * d)
            return [angle, direction, speed]


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e = 0
        self.e_sum = 0

    def output(self, e):
        de = e - self.e
        self.e_sum = self.e_sum + e
        self.e = e
        return self.kp * e + self.ki * self.e_sum + self.kd * de
