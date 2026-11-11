"""Yellow centerline + right road-edge prototype for the uploaded CARLA footage.
Run: py road_boundary_detection.py [image_or_video] [--no-display]
Without an input, a synthetic straight-road demo is used.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def _fit_yellow(frame):
    """Fit x=a*y+b to yellow pixels; robust against isolated yellow objects."""
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (15, 45, 90), (40, 255, 255))
    roi = np.zeros((h, w), np.uint8)
    poly = np.array([[(0, int(.97*h)), (int(.38*w), int(.53*h)),
                      (int(.55*w), int(.53*h)), (int(.52*w), int(.97*h))]], np.int32)
    cv2.fillPoly(roi, poly, 255)
    mask = cv2.bitwise_and(mask, roi)
    ys, xs = np.nonzero(mask)
    if len(xs) < 30:
        return None, "yellow_not_found"
    rng = np.random.default_rng(7)
    take = rng.choice(len(xs), min(len(xs), 2200), replace=False)
    y, x = ys[take].astype(float), xs[take].astype(float)
    tolerance = max(3., .006*w)
    best = None
    for _ in range(200):
        i, j = rng.choice(len(x), 2, replace=False)
        if abs(y[j]-y[i]) < .035*h:
            continue
        a = (x[j]-x[i])/(y[j]-y[i]); b = x[i]-a*y[i]
        if not (-3.5 < a < -.12):
            continue
        if not (.25*w < a*(.53*h)+b < .65*w):
            continue
        good = np.abs(x-(a*y+b)) < tolerance
        if good.sum() < 25:
            continue
        span = np.ptp(y[good])
        # Vertical coverage matters, so compact yellow objects don't win.
        score = good.sum()*min(1., span/(.25*h))
        if best is None or score > best[0]:
            best = (score, good)
    if best is None:
        return None, "yellow_fit_failed"
    good = best[1]
    a, b = np.polyfit(y[good], x[good], 1)
    good = np.abs(x-(a*y+b)) < tolerance
    if good.sum() < 25 or np.ptp(y[good]) < .12*h:
        return None, "yellow_too_short"
    a, b = np.polyfit(y[good], x[good], 1)
    # Reject fits dominated by scatter; the evidence must hug the line.
    residual = np.abs(x[good]-(a*y[good]+b))
    if np.median(residual) > .004*w:
        return None, "yellow_support_scattered"
    ymin, ymax = float(y[good].min()), float(y[good].max())
    if .85*h-ymax > .30*h:
        return None, "yellow_extrapolation_too_far"
    return (float(a), float(b), ymin, ymax), "ok"


def _fit_curb(frame):
    """Appearance-based road-edge candidate, NOT semantic curb recognition.
    Assumes this camera faces a straight road with the right curb in the ROI.
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 40, 100)
    roi = np.zeros_like(gray)
    poly = np.array([[(int(.50*w), int(.53*h)), (int(.65*w), int(.53*h)),
                      (w-1, h-1), (int(.57*w), h-1)]], np.int32)
    cv2.fillPoly(roi, poly, 255)
    lines = cv2.HoughLinesP(cv2.bitwise_and(edges, roi), 1, np.pi/180, 25,
                           minLineLength=int(.07*h), maxLineGap=int(.05*h))
    candidates = []
    if lines is not None:
        for x1, y1, x2, y2 in lines.reshape(-1, 4):
            if abs(int(y2)-int(y1)) < .04*h:
                continue
            a = (float(x2)-x1)/(float(y2)-y1); b = x1-a*y1
            xt, xb = a*(.53*h)+b, a*(.95*h)+b
            if .25 < a < 2.0 and .46*w < xt < .69*w and .62*w < xb < 1.12*w:
                candidates.append((a, b, np.hypot(x2-x1, y2-y1)))
    if not candidates:
        return None
    values = np.array(candidates)
    best = None
    for a, b, _ in candidates:
        good = (np.abs((values[:,0]-a)*(.75*h)+(values[:,1]-b)) < .025*w)
        good &= np.abs((values[:,0]-a)*(.95*h)+(values[:,1]-b)) < .035*w
        score = values[good,2].sum()
        if best is None or score > best[0]:
            best = (score, good)
    a, b = np.average(values[best[1],:2], axis=0, weights=values[best[1],2])
    return float(a), float(b)


def process_frame(frame):
    """BGR uint8 -> result. + error: estimated corridor center is to the right.
    Pixel midpoint is a visual proxy, NOT a metric lane center or steering command.
    Camera liveness/timeout detection must be done by the calling application.
    """
    result = {"camera_ok": frame is not None, "path_valid": False,
              "lane_valid": False, "center_error_px": None,
              "center_error_normalized": None, "yellow_line": None,
              "curb_candidate": None, "sample_y": None,
              "mode": "yellow_centerline_and_right_road_edge",
              "reason": "no_frame"}
    if frame is None:
        return result
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("Expected uint8 BGR image (height, width, 3)")
    h, w = frame.shape[:2]
    if min(h, w) < 64:
        raise ValueError("Image must be at least 64x64")
    yellow, why = _fit_yellow(frame)
    curb = _fit_curb(frame)
    top, bottom = int(.56*h), int(.95*h)
    for model, key in [(yellow, "yellow_line"), (curb, "curb_candidate")]:
        if model is not None:
            a, b = model[:2]
            result[key] = [int(a*bottom+b), bottom, int(a*top+b), top]
    if yellow is None:
        result["reason"] = why
        return result
    if curb is None:
        result["reason"] = "right_road_edge_not_found"
        return result
    y = int(.85*h)
    lx, rx = yellow[0]*y+yellow[1], curb[0]*y+curb[1]
    if not (-.15*w < lx < .65*w and .5*w < rx < 1.15*w and .20*w < rx-lx < 1.1*w):
        result["reason"] = "invalid_corridor_geometry"
        return result
    error = (lx+rx)/2-w/2
    result.update(path_valid=True, center_error_px=round(float(error),2),
                  center_error_normalized=round(float(error)/(w/2),4),
                  sample_y=y, reason="ok")
    # lane_valid stays False: two painted lane boundaries were NOT detected.
    return result


class BoundaryTracker:
    """One instance per camera stream; timestamps must be monotonic seconds.
    Brief held results are explicitly estimated, never fresh detections.
    Thresholds are prototype settings, not validated driving limits.
    """
    def __init__(self):
        self.filtered = None
        self.last_accepted = None
        self.last_timestamp = None
        self.pending = []
        self.shape = None

    @staticmethod
    def geometry_distance(a, b):
        # Endpoint movement detects rotation even when the midpoint is unchanged.
        return max(abs(a[key][j]-b[key][j])
                   for key in ("yellow_line", "curb_candidate") for j in (0, 2))

    def update(self, frame, timestamp_s):
        if not np.isfinite(timestamp_s):
            raise ValueError("timestamp must be finite")
        if self.last_timestamp is not None and timestamp_s <= self.last_timestamp:
            raise ValueError("timestamp must increase")
        self.last_timestamp = timestamp_s
        raw = process_frame(frame)
        out = dict(raw)
        out.update(raw_path_valid=raw["path_valid"], raw_center_error_px=raw["center_error_px"],
                   tracking_status="lost", detection_fresh=False, estimate_age_s=None)
        if frame is None:
            self.filtered = None; self.pending = []
            return out
        h,w = frame.shape[:2]
        if self.shape != (h,w):
            self.filtered = None; self.pending = []; self.shape = (h,w)
        age = timestamp_s-self.last_accepted if self.last_accepted is not None else float('inf')
        if age > .20:
            self.filtered = None
        accept = False
        if raw["path_valid"]:
            error = raw["center_error_px"]
            jump = self.filtered is not None and self.geometry_distance(raw, self.filtered) > .045*w
            if self.filtered is None or jump:
                if self.pending and self.geometry_distance(raw, self.pending[-1]) > .025*w:
                    self.pending = []
                self.pending.append(raw)
                if len(self.pending) >= 5:
                    accept = True; self.pending = []
                out["tracking_status"] = "confirming" if self.filtered is None else "jump_rejected"
            else:
                self.pending = []; accept = True
        else:
            self.pending = []
        if accept:
            if self.filtered is None:
                self.filtered = dict(raw)
            else:
                delta = self.geometry_distance(raw, self.filtered)
                alpha = min(.25, (.008*w)/max(delta, 1e-6))
                for key in ("yellow_line", "curb_candidate"):
                    self.filtered[key] = [int(round((1-alpha)*a+alpha*b))
                                          for a,b in zip(self.filtered[key],raw[key])]
                y = raw["sample_y"]
                xs = []
                for key in ("yellow_line", "curb_candidate"):
                    xb,yb,xt,yt = self.filtered[key]
                    xs.append(xt+(xb-xt)*(y-yt)/(yb-yt))
                self.filtered["center_error_px"] = round(sum(xs)/2-w/2,2)
                self.filtered["center_error_normalized"] = round(self.filtered["center_error_px"]/(w/2),4)
            self.last_accepted = timestamp_s
            age = 0.
            out["tracking_status"] = "tracked"; out["detection_fresh"] = True
        if self.filtered is not None and age <= .20:
            for key in ("yellow_line", "curb_candidate", "center_error_px", "center_error_normalized", "sample_y"):
                out[key] = self.filtered[key]
            out["path_valid"] = True
            out["estimate_age_s"] = round(age,4)
            if not accept:
                if out["tracking_status"] != "jump_rejected":out["tracking_status"] = "held"
                out["reason"] = "short_hold:"+raw["reason"]
        else:
            out.update(path_valid=False, center_error_px=None, center_error_normalized=None,
                       yellow_line=None, curb_candidate=None, sample_y=None)
        return out


def draw_result(frame, result):
    out = frame.copy(); h, w = out.shape[:2]
    for key, color in [("yellow_line", (0,255,255)), ("curb_candidate", (255,100,0))]:
        line = result[key]
        if line is not None:
            cv2.line(out, tuple(line[:2]), tuple(line[2:]), color, 3)
    cv2.putText(out, "Yellow: centerline | Blue: road-edge candidate", (12,28),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (0,0,255), 2)
    if result["path_valid"]:
        y = result["sample_y"]; x = int(w/2+result["center_error_px"])
        cv2.circle(out, (x,y), 7, (0,255,0), -1)
        cv2.line(out, (w//2,y-25), (w//2,y+25), (255,0,255), 2)
        text = f"Visual center error: {result['center_error_px']:+.1f} px (not metres)"
    else:
        text = "NO PATH: "+result["reason"]
    cv2.putText(out, text, (12,56), cv2.FONT_HERSHEY_SIMPLEX, .55, (0,0,255), 2)
    if "tracking_status" in result:
        cv2.putText(out, "Status: "+result["tracking_status"]+" | age: "+str(result["estimate_age_s"]),
                    (12,84), cv2.FONT_HERSHEY_SIMPLEX, .55, (0,0,255), 2)
    return out


def demo_frame():
    # Synthetic input only: confirms execution, not real-world accuracy.
    frame = np.full((540, 960, 3), 55, np.uint8)
    cv2.line(frame, (170, 539), (430, 310), (0, 255, 255), 8)
    cv2.line(frame, (790, 539), (530, 310), (255, 255, 255), 8)
    cv2.putText(frame, "SYNTHETIC DEMO", (15, 80),
                cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    return frame


def save_image(path, frame):
    ok, encoded = cv2.imencode(".png", frame)
    if not ok:
        raise RuntimeError("image encoding failed")
    encoded.tofile(str(path))  # Supports Windows Korean paths.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", help="image or video path")
    parser.add_argument("--no-display", action="store_true")
    args = parser.parse_args()
    output = Path(__file__).resolve().parent / "boundary_output"
    output.mkdir(exist_ok=True)
    cap = None
    try:
        if args.input is None:
            frame = demo_frame()
        else:
            path = Path(args.input)
            if not path.is_file():
                raise FileNotFoundError(f"Input not found: {path}")
            frame = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                cap = cv2.VideoCapture(str(path))
                if not cap.isOpened():
                    raise RuntimeError("Cannot open input as an image or video")
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError("Cannot read first video frame")
        index = 0
        tracker = BoundaryTracker()
        fps = cap.get(cv2.CAP_PROP_FPS) if cap else 0
        if cap and (not np.isfinite(fps) or fps <= 0):
            raise RuntimeError("Video FPS missing; reliable timestamps required")
        with (output / "results.jsonl").open("w", encoding="utf-8") as log:
            while frame is not None:
                result = tracker.update(frame, index/fps) if cap else process_frame(frame)
                result["frame_id"] = index
                result["timestamp_s"] = (index/fps if cap else None)
                log.write(json.dumps(result, ensure_ascii=False)+"\n")
                annotated = draw_result(frame, result)
                if index == 0:
                    save_image(output / "first_frame.png", annotated)
                    print(json.dumps(result, indent=2))
                if not args.no_display:
                    cv2.imshow("Lane detection - Q or ESC to exit", annotated)
                    key = cv2.waitKey(max(1, int(1000/fps)) if cap else 0) & 0xff
                    if key in (ord("q"), 27):
                        break
                if cap is None:
                    break
                ok, frame = cap.read()
                if not ok:
                    break
                index += 1
        print(f"Saved result image and JSONL log to: {output}")
    finally:
        if cap is not None:
            cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
