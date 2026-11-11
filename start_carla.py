import os
import signal
import subprocess
from pathlib import Path

# 1. CARLA가 설치된 폴더
carla_folder = Path.home() / "CARLA"

# 2. 이 노트북에서 NVIDIA 그래픽카드를 사용하도록 설정
environment = os.environ.copy()
environment.update({
    "__NV_PRIME_RENDER_OFFLOAD": "1",
    "__VK_LAYER_NV_optimus": "NVIDIA_only",
    "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
    "VK_ICD_FILENAMES": "/usr/share/vulkan/icd.d/nvidia_icd.json",
})

# 3. 실행할 프로그램과 옵션
command = [
    str(carla_folder / "CarlaUE4.sh"),
    "-quality-level=low",
    "-windowed",
    "-ResX=800",
    "-ResY=600",
    "-nosound",
    "-carla-rpc-port=2000",
    "-graphicsadapter=0",
]

# 4. CARLA 실행
process = subprocess.Popen(
    command,
    cwd=carla_folder,
    env=environment,
    start_new_session=True,
)

try:
    print("CARLA를 시작합니다. 종료하려면 Ctrl+C를 누르세요.")
    process.wait()

except KeyboardInterrupt:
    print("CARLA를 종료합니다.")

finally:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)

        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()