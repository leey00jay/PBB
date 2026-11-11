# 이전 경로 호환 및 원본 교재 자료

현재 주행 코드는 [인지](../perception/README.md), [판단](../decision/README.md), [제어](../control/README.md)로 분리했습니다.
이 폴더의 `lane_following.py`, `lane_detector.py`, `camera_geometry.py`, `road_boundary_recovery.py`는 이전 import를 유지하는 연결 파일입니다.
실제 알고리즘은 해당 파트 폴더에서 수정하세요. `fastai_model.pth`와 원본의 학습·교재 자료는 이 위치에 유지합니다.

---

The following files are provided with the course and it is **not** expected that the student/reader would write them as part of the exercises:

* `collect_data.py`
* `camera_geometry_numba.py` : You can import the `CameraGeometry` class from this file instead of importing it from `camera_geometry.py`. This will lead to a speed-up due to [numba](http://numba.pydata.org/)