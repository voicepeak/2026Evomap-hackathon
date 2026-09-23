"""服务重启等待：HTTP 端口释放不代表设备清理已完成。"""
import subprocess
import time


def process_alive(pid):
    result = subprocess.run(['ps', '-p', str(pid), '-o', 'stat='], capture_output=True, text=True)
    state = result.stdout.strip()
    return bool(state and not state.startswith('Z'))


def wait_for_exit(pids, timeout=30.0, alive=process_alive, sleep=time.sleep):
    for _ in range(max(1, int(timeout / .25))):
        if not any(alive(pid) for pid in pids):
            return
        sleep(.25)
    if any(alive(pid) for pid in pids):
        raise RuntimeError('旧进程尚未完成设备清理；未强制终止，也未启动新服务。')
