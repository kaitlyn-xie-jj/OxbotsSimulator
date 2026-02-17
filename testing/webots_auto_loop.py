#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
webots_auto_loop_crossplatform.py

跨平台（macOS / Linux / Windows）自动循环脚本：
启动 Webots 加载 world，实时读取 stdout/stderr，等待目标文本（默认 "Supervisor controller exiting."）
出现后优雅终止/重启，重复执行。

用法示例：
  python3 webots_auto_loop_crossplatform.py
  python3 webots_auto_loop_crossplatform.py --webots "/Applications/Webots.app/Contents/MacOS/webots" \
    --world "~/Desktop/OxbotsSimulator/worlds/Decision_making.wbt" --max-runs 10 --delay-after 2

注意：
- 请确保 Supervisor 控制器会把目标文本输出到 stdout 或 stderr（区分大小写）。
- 在 Windows 上，若通过 PowerShell 运行，注意路径引号。
"""

import argparse
import csv
import subprocess
import sys
import time
import threading
import queue
import os
import signal
from datetime import datetime
from typing import Optional, Tuple

# 默认路径（你之前给出的 mac 路径 & world）
DEFAULT_WEBOTS_MAC = "/Applications/Webots.app/Contents/MacOS/webots"
DEFAULT_WEBOTS_LINUX = "/usr/local/webots/bin/webots"
DEFAULT_WEBOTS_WINDOWS = r"C:\Program Files\Webots\webots.exe"
DEFAULT_WORLD = os.path.expanduser("~/Desktop/OxbotsSimulator/worlds/Decision_making.wbt")

WATCH_TEXT = "Supervisor controller exiting."

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

DEFAULT_MODE_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "mode.txt")
DEFAULT_COLLISION_CONFIG_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "collision_avoiding.txt")
DEFAULT_PLANNED_INDEX_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "real_time_data", "planned_waypoints_index.txt")
DEFAULT_BALL_HISTORY_FILE = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "real_time_data", "ball_taken_history.txt")
DEFAULT_COLLISION_COUNTER_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "real_time_data", "collision_counter.txt")

DEFAULT_BENCHMARK_MODES = ["nearest", "realistic_nearest", "improved_nearest", "planned"]
DEFAULT_AVOIDANCE_SETTINGS = ["off", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0"]

# ---------- 辅助函数 ----------
def enqueue_stream(stream, q):
    """后台线程：把流逐行放入队列，避免阻塞主线程"""
    try:
        for line in iter(stream.readline, ''):
            if not line:
                break
            q.put(line)
    except Exception:
        pass
    finally:
        try:
            stream.close()
        except Exception:
            pass

def build_default_webots_path() -> str:
    """根据平台返回默认 webots 可执行路径（优先使用存在的路径）"""
    candidates = []
    if sys.platform.startswith("darwin"):
        candidates = [DEFAULT_WEBOTS_MAC, DEFAULT_WEBOTS_LINUX, DEFAULT_WEBOTS_WINDOWS]
    elif sys.platform.startswith("win"):
        candidates = [DEFAULT_WEBOTS_WINDOWS, DEFAULT_WEBOTS_MAC, DEFAULT_WEBOTS_LINUX]
    else:
        candidates = [DEFAULT_WEBOTS_LINUX, DEFAULT_WEBOTS_MAC, DEFAULT_WEBOTS_WINDOWS]

    for p in candidates:
        if os.path.isabs(p) and os.path.exists(p):
            return p
    # 若都不存在，返回第一个候选（让用户覆盖）
    return candidates[0]

def run_and_wait_for_text(webots_bin: str, world_file: str, extra_args: Optional[list]=None,
                          timeout: Optional[float]=None, log_file: Optional[str]=None
                         ) -> Tuple[bool, subprocess.Popen]:
    """
    启动 webots 进程并监听输出，直到找到 WATCH_TEXT（返回 True）或进程自然结束/超时（返回 False）。
    返回 (found_flag, proc) —— proc 是 subprocess.Popen 实例（可能已结束）。
    """
    cmd = [webots_bin, world_file]
    if extra_args:
        cmd += extra_args

    nowstr = datetime.now().strftime("%Y%m%d_%H%M%S")
    logfile = None
    if log_file:
        logfile = open(log_file, "a", buffering=1, encoding="utf-8")
        logfile.write(f"\n===== run start {nowstr} cmd: {' '.join(cmd)} =====\n")

    # Windows: 为了能发送 CTRL_BREAK_EVENT，需要在创建进程时设置 CREATE_NEW_PROCESS_GROUP
    creationflags = 0
    start_new_session = False
    if sys.platform.startswith("win"):
        creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    else:
        # Unix 系统可以考虑 start_new_session=True 来隔离进程组（等同 setsid）
        start_new_session = True

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            creationflags=creationflags,
            start_new_session=start_new_session
        )
    except FileNotFoundError as e:
        raise RuntimeError(f"无法启动 Webots，可执行文件找不到：{webots_bin}") from e
    except Exception as e:
        raise RuntimeError(f"启动 Webots 失败：{e}") from e

    q = queue.Queue()
    t = threading.Thread(target=enqueue_stream, args=(proc.stdout, q), daemon=True)
    t.start()

    start_time = time.time()
    found = False

    try:
        while True:
            try:
                line = q.get(timeout=0.2)
            except queue.Empty:
                line = None

            if line is not None:
                # 输出到终端与日志
                print(line, end='')
                if logfile:
                    logfile.write(line)
                if WATCH_TEXT in line:
                    found = True
                    break

            # 检查进程状态
            ret = proc.poll()
            if ret is not None:
                # 进程已退出，继续吐尽队列
                while not q.empty():
                    extra = q.get_nowait()
                    print(extra, end='')
                    if logfile:
                        logfile.write(extra)
                break

            # 检查超时
            if timeout is not None and (time.time() - start_time) > timeout:
                print(f"[timeout] 超过 {timeout} 秒仍未找到目标文本。")
                break

        return found, proc
    finally:
        if logfile:
            logfile.write(f"\n===== run end {datetime.now().strftime('%Y%m%d_%H%M%S')} =====\n")
            logfile.flush()
            logfile.close()

def terminate_process(proc: subprocess.Popen, grace: int = 5):
    """
    尝试优雅终止 proc：
    - Windows: 先发送 CTRL_BREAK_EVENT（需要子进程在新进程组），然后 terminate，再 kill。
    - Unix: 先发送 SIGINT，再 terminate/kll。
    """
    if proc is None:
        return

    if proc.poll() is not None:
        return  # 已退出

    try:
        if sys.platform.startswith("win"):
            # 先发送 CTRL_BREAK_EVENT 给进程组（需要 CREATE_NEW_PROCESS_GROUP）
            try:
                proc.send_signal(signal.CTRL_BREAK_EVENT)
                # 等待短暂时间看是否退出
                proc.wait(timeout=grace)
                return
            except Exception:
                # 继续后续步骤
                pass

            # 再尝试 terminate
            try:
                proc.terminate()
            except Exception:
                pass

            try:
                proc.wait(timeout=grace)
                return
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                except Exception:
                    pass
                try:
                    proc.wait(timeout=5)
                except Exception:
                    pass

        else:
            # Unix: 先 SIGINT
            try:
                proc.send_signal(signal.SIGINT)
            except Exception:
                try:
                    proc.terminate()
                except Exception:
                    pass

            try:
                proc.wait(timeout=grace)
                return
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                except Exception:
                    pass
                try:
                    proc.wait(timeout=5)
                except Exception:
                    pass
    except Exception as e:
        # 忽略任何终止中出现的问题，但记录到 stdout
        print(f"[terminate_process] 终止过程中发生异常：{e}")


def read_text_file(path: str) -> Optional[str]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()
    except Exception:
        return None


def write_text_file(path: str, content: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def parse_ball_taken_history_last_line(path: str) -> Tuple[Optional[float], Optional[int], Optional[str]]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]
        if not lines:
            return None, None, None

        last = lines[-1]
        parts = [p.strip() for p in last.split(",")]
        if len(parts) < 2:
            return None, None, last

        last_ball_time = float(parts[0])
        total_balls = int(float(parts[1]))
        return last_ball_time, total_balls, last
    except Exception:
        return None, None, None


def parse_collision_counter(path: str) -> Optional[int]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = f.read().strip()
        if not raw:
            return None
        if "=" in raw:
            raw = raw.split("=", 1)[1].strip()
        return int(float(raw))
    except Exception:
        return None


def reset_runtime_metric_files(ball_history_file: str,
                               collision_counter_file: str,
                               planned_index_file: Optional[str] = None) -> None:
    write_text_file(ball_history_file, "")
    write_text_file(collision_counter_file, "count=0\n")
    if planned_index_file:
        write_text_file(planned_index_file, "")


def run_benchmark_matrix(args) -> int:
    modes = [m.strip() for m in args.modes if str(m).strip()]
    avoidance_settings = [a.strip() for a in args.avoidance if str(a).strip()]

    if not modes:
        print("[错误] modes 为空，无法执行基准测试。")
        return 1
    if not avoidance_settings:
        print("[错误] avoidance 为空，无法执行基准测试。")
        return 1

    mode_file = os.path.expanduser(os.path.expandvars(args.mode_file))
    collision_config_file = os.path.expanduser(os.path.expandvars(args.collision_config_file))
    planned_index_file = os.path.expanduser(os.path.expandvars(args.planned_index_file))
    ball_history_file = os.path.expanduser(os.path.expandvars(args.ball_history_file))
    collision_counter_file = os.path.expanduser(os.path.expandvars(args.collision_counter_file))

    nowstr = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_csv = args.result_csv
    if not result_csv:
        result_csv = os.path.join(SCRIPT_DIR, f"benchmark_results_{nowstr}.csv")
    result_csv = os.path.expanduser(os.path.expandvars(result_csv))

    total_runs = len(modes) * len(avoidance_settings)
    print(f"开始矩阵测试：modes={modes} avoidance={avoidance_settings} 总计 {total_runs} 组")
    print(f"结果将写入：{result_csv}")

    original_mode_content = read_text_file(mode_file)
    original_collision_content = read_text_file(collision_config_file)

    results = []
    run_count = 0

    try:
        for mode in modes:
            for avoidance in avoidance_settings:
                run_count += 1
                if avoidance.lower() == "off":
                    collision_text = "off\n"
                    avoidance_label = "off"
                else:
                    collision_text = f"smart_factor = {avoidance}\n"
                    avoidance_label = f"smart_factor={avoidance}"

                print(
                    f"\n=== [{run_count}/{total_runs}] mode={mode} collision={avoidance_label} ({datetime.now().isoformat()}) ==="
                )

                write_text_file(mode_file, f"{mode}\n")
                write_text_file(collision_config_file, collision_text)
                reset_runtime_metric_files(
                    ball_history_file=ball_history_file,
                    collision_counter_file=collision_counter_file,
                    planned_index_file=planned_index_file,
                )

                started_at = datetime.now().isoformat()
                found = False
                return_code = None
                process_error = ""

                try:
                    found, proc = run_and_wait_for_text(
                        webots_bin=args.webots,
                        world_file=args.world,
                        extra_args=args.extra_args,
                        timeout=args.timeout,
                        log_file=args.log_file,
                    )
                    if proc and proc.poll() is None:
                        terminate_process(proc, grace=5)
                    return_code = proc.returncode if proc else None
                except RuntimeError as e:
                    process_error = str(e)
                    print(f"[错误] 运行失败：{e}")
                    return_code = -1

                last_ball_time, total_balls, last_history_line = parse_ball_taken_history_last_line(ball_history_file)
                collision_count = parse_collision_counter(collision_counter_file)

                row = {
                    "run_index": run_count,
                    "mode": mode,
                    "avoidance": avoidance_label,
                    "watch_text_found": found,
                    "last_ball_time": "" if last_ball_time is None else f"{last_ball_time}",
                    "total_balls_taken": "" if total_balls is None else str(total_balls),
                    "collision_count": "" if collision_count is None else str(collision_count),
                    "ball_history_last_line": "" if last_history_line is None else last_history_line,
                    "process_return_code": "" if return_code is None else str(return_code),
                    "process_error": process_error,
                    "started_at": started_at,
                    "finished_at": datetime.now().isoformat(),
                }
                results.append(row)

                print(
                    f"[结果] time={row['last_ball_time'] or 'N/A'} balls={row['total_balls_taken'] or 'N/A'} "
                    f"collision={row['collision_count'] or 'N/A'} found={found}"
                )

                time.sleep(max(args.delay_after, args.run_interval))

    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，提前结束矩阵测试。")
    finally:
        if original_mode_content is not None:
            write_text_file(mode_file, original_mode_content)
        if original_collision_content is not None:
            write_text_file(collision_config_file, original_collision_content)

    os.makedirs(os.path.dirname(result_csv), exist_ok=True)
    with open(result_csv, "w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "run_index",
            "mode",
            "avoidance",
            "watch_text_found",
            "last_ball_time",
            "total_balls_taken",
            "collision_count",
            "ball_history_last_line",
            "process_return_code",
            "process_error",
            "started_at",
            "finished_at",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    print(f"矩阵测试结束，共写入 {len(results)} 条结果到 {result_csv}")
    return 0

# ---------- 主流程 ----------
def main():
    global WATCH_TEXT

    parser = argparse.ArgumentParser(description="跨平台自动循环：启动 webots，等待 Supervisor 输出，然后重启循环。")
    default_webots = build_default_webots_path()
    parser.add_argument("--webots", default=default_webots, help=f"webots 可执行路径（默认尝试：{default_webots}）")
    parser.add_argument("--world", default=DEFAULT_WORLD, help=f"要加载的 .wbt 世界文件路径（默认已填）")
    parser.add_argument("--extra-args", nargs="*", default=[], help="传给 webots 的额外参数，例如 --batch --no-rendering")
    parser.add_argument("--delay-after", type=float, default=1.0, help="检测到文本后，在重启前等待的秒数（默认 1s）")
    parser.add_argument("--max-runs", type=int, default=0, help="最大循环次数，0 表示无限循环")
    parser.add_argument("--run-interval", type=float, default=0.5, help="每次启动间的最小间隔（秒）")
    parser.add_argument("--timeout", type=float, default=None, help="每次运行查找文本的超时时间（秒），空表示不超时")
    parser.add_argument("--log-file", default=None, help="追加日志到此文件（可选）")
    parser.add_argument("--watch-text", default=WATCH_TEXT, help="要匹配的目标文本（默认 'Supervisor controller exiting.'）")
    parser.add_argument("--benchmark-matrix", action="store_true", help="按 mode × collision 配置矩阵批量测试并导出 CSV")
    parser.add_argument("--modes", nargs="*", default=DEFAULT_BENCHMARK_MODES, help="矩阵测试使用的 mode 列表")
    parser.add_argument("--avoidance", nargs="*", default=DEFAULT_AVOIDANCE_SETTINGS, help="矩阵测试避障设置列表，例如 off 1 2 3")
    parser.add_argument("--mode-file", default=DEFAULT_MODE_FILE, help="mode.txt 路径")
    parser.add_argument("--collision-config-file", default=DEFAULT_COLLISION_CONFIG_FILE, help="collision_avoiding.txt 路径")
    parser.add_argument("--planned-index-file", default=DEFAULT_PLANNED_INDEX_FILE, help="planned_waypoints_index.txt 路径")
    parser.add_argument("--ball-history-file", default=DEFAULT_BALL_HISTORY_FILE, help="ball_taken_history.txt 路径")
    parser.add_argument("--collision-counter-file", default=DEFAULT_COLLISION_COUNTER_FILE, help="collision_counter.txt 路径")
    parser.add_argument("--result-csv", default=None, help="矩阵测试结果 CSV 路径（默认 testing/benchmark_results_时间戳.csv）")
    args = parser.parse_args()

    WATCH_TEXT = args.watch_text

    # expand ~ 和 env
    args.world = os.path.expanduser(os.path.expandvars(args.world))
    args.webots = os.path.expanduser(os.path.expandvars(args.webots))

    if args.benchmark_matrix:
        return run_benchmark_matrix(args)

    if not os.path.isabs(args.webots) or not os.path.exists(args.webots):
        print(f"[警告] 指定的 webots 可执行可能不存在：{args.webots}")
        # 仍继续尝试，让用户覆盖或修正

    if not os.path.exists(args.world):
        print(f"[警告] 指定的 world 文件不存在：{args.world}")
        # 仍继续尝试，让用户覆盖或修正

    run_count = 0
    print(f"自动化开始，webots={args.webots} world={args.world}\n目标文本：{WATCH_TEXT}")

    try:
        while True:
            run_count += 1
            if args.max_runs > 0 and run_count > args.max_runs:
                print("达到最大运行次数，退出。")
                break

            print(f"\n=== 启动第 {run_count} 次运行（{datetime.now().isoformat()}） ===")
            try:
                found, proc = run_and_wait_for_text(
                    webots_bin=args.webots,
                    world_file=args.world,
                    extra_args=args.extra_args,
                    timeout=args.timeout,
                    log_file=args.log_file
                )
            except RuntimeError as e:
                print(f"[错误] 无法启动 webots: {e}")
                return 1

            if found:
                print(f"[检测到] 在第 {run_count} 次运行中找到文本：{WATCH_TEXT}")
            else:
                print(f"[未检测到] 进程退出或超时（run_count={run_count}），将尝试终止进程并重启（若进程仍在运行）")

            # 尝试优雅终止仍在运行的进程
            if proc and proc.poll() is None:
                print("[信息] 进程仍在运行，尝试优雅终止...")
                terminate_process(proc, grace=5)
            else:
                if proc:
                    print(f"[信息] 进程已退出，返回码 {proc.returncode}")
                else:
                    print("[信息] 没有可操作的进程句柄。")

            # 等待一会儿再重新启动（避免连续重启过快）
            time.sleep(max(args.delay_after, args.run_interval))
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，正在退出...")
        return 0

    print("自动化结束。")
    return 0

if __name__ == "__main__":
    sys.exit(main())