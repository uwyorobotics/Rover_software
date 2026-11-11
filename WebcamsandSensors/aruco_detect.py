#!/usr/bin/env python3
"""
ArUco marker capture + detection (DICT_4X4_50), for the keyboard / keyhole
alignment markers (2x2 cm on the keyboard, 1x1 cm on the lock).

Install:  pip install opencv-contrib-python numpy

Usage:
  python aruco_detect.py                      # live webcam 0
  python aruco_detect.py --camera 1           # different camera
  python aruco_detect.py --image photo.jpg    # detect in a saved image
  python aruco_detect.py --marker-size 0.02 --calib calib.npz   # add 3D pose

Keys (live mode):  s = save frame (+ annotated copy)   q / ESC = quit
calib.npz must contain 'camera_matrix' and 'dist_coeffs' (from cv2.calibrateCamera).
"""
import argparse
import os
import time

import cv2
import numpy as np


def make_detector():
    """Works on both new (>=4.7) and old OpenCV aruco APIs."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return lambda gray: detector.detectMarkers(gray)
    params = aruco.DetectorParameters_create()
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)


def estimate_pose(corners, marker_size, K, dist):
    """Pose of one marker via solvePnP. Returns (rvec, tvec) in meters."""
    h = marker_size / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        obj, corners.reshape(4, 2), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    return (rvec, tvec) if ok else (None, None)


def process(frame, detect, marker_size=None, K=None, dist=None):
    """Detect markers, draw them on frame, return list of result dicts."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect(gray)
    results = []
    if ids is None:
        return results

    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    for c, i in zip(corners, ids.flatten()):
        pts = c.reshape(4, 2)
        center = pts.mean(axis=0)
        info = {"id": int(i), "center_px": tuple(center.round(1)), "corners_px": pts}

        label = f"ID {i}"
        if marker_size and K is not None:
            rvec, tvec = estimate_pose(c, marker_size, K, dist)
            if rvec is not None:
                info["rvec"], info["tvec"] = rvec.ravel(), tvec.ravel()
                cv2.drawFrameAxes(frame, K, dist, rvec, tvec, marker_size * 0.75)
                x, y, z = tvec.ravel()
                label += f"  ({x*100:.1f},{y*100:.1f},{z*100:.1f}) cm"
        cv2.putText(frame, label, (int(pts[0][0]), int(pts[0][1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        results.append(info)
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--image", help="process a single image instead of the camera")
    ap.add_argument("--marker-size", type=float, default=None,
                    help="marker side length in meters (0.02 keyboard, 0.01 lock)")
    ap.add_argument("--calib", help=".npz with camera_matrix and dist_coeffs")
    ap.add_argument("--outdir", default="captures")
    args = ap.parse_args()

    K = dist = None
    if args.calib:
        data = np.load(args.calib)
        K, dist = data["camera_matrix"], data["dist_coeffs"]
    elif args.marker_size:
        print("Note: --marker-size needs --calib for pose; showing IDs only.")

    detect = make_detector()
    os.makedirs(args.outdir, exist_ok=True)

    # ---- single image mode ----
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"Could not read {args.image}")
        res = process(frame, detect, args.marker_size, K, dist)
        print(f"Found {len(res)} marker(s): {sorted(r['id'] for r in res)}")
        out = os.path.join(args.outdir, "annotated_" + os.path.basename(args.image))
        cv2.imwrite(out, frame)
        print("Saved", out)
        return

    # ---- live camera mode ----
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {args.camera}")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed")
            break

        raw = frame.copy()
        res = process(frame, detect, args.marker_size, K, dist)
        ids = sorted(r["id"] for r in res)
        cv2.putText(frame, f"Markers: {ids}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("ArUco 4x4_50  (s=save, q=quit)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            stamp = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(os.path.join(args.outdir, f"raw_{stamp}.png"), raw)
            cv2.imwrite(os.path.join(args.outdir, f"detected_{stamp}.png"), frame)
            print(f"Saved capture {stamp}  markers={ids}")
            for r in res:
                print("  ", r["id"], r["center_px"],
                      "tvec(m)=" + str(r["tvec"].round(3)) if "tvec" in r else "")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
