#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import sys
import time
import shutil
import queue
import threading
import subprocess
from dataclasses import dataclass
from pathlib import Path

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# --------------------------
# Config
# --------------------------

DEFAULT_APT_PACKAGES = [
    "docker.io",
    "containerd",
    "runc",
    "docker-compose",
    "iptables",
    "ca-certificates",
    "curl",
    "openssl",
    "procps",
    "lsof",
    "net-tools",
    "jq",
]

OPEN_WEBUI_IMAGE = "ghcr.io/open-webui/open-webui:main"

SUPPORTED_LINUX_OSES = [
    # Ubuntu
    "Ubuntu 24.04 LTS (Noble)",
    "Ubuntu 22.04 LTS (Jammy)",
    "Ubuntu 20.04 LTS (Focal)",

    # Debian
    "Debian 12 (Bookworm)",
    "Debian 11 (Bullseye)",

    # RHEL family
    "RHEL 9",
    "RHEL 8",
    "Rocky Linux 9",
    "Rocky Linux 8",
    "AlmaLinux 9",
    "AlmaLinux 8",
    "CentOS Stream 9",
    "CentOS Stream 8",
    "Oracle Linux 9",
    "Oracle Linux 8",

    # Fedora
    "Fedora 41",
    "Fedora 40",

    # SUSE
    "openSUSE Leap 15.6",
    "openSUSE Tumbleweed",
    "SLES 15 SP6",

    # Arch family
    "Arch Linux",
    "Manjaro",

    # Enterprise/Server misc
    "Amazon Linux 2023",
    "Amazon Linux 2",

    # Cloud-native / minimal
    "Ubuntu Server 24.04 LTS",
    "Debian 12 (minimal)",
    "Alpine Linux 3.20",
    "Alpine Linux 3.19",

    # Desktop popular
    "Linux Mint 22",
    "Linux Mint 21.3",
    "Pop!_OS 22.04",
    "Zorin OS 17",
    "elementary OS 7",
    "Kali Linux",
    "Parrot OS",

    # Other notable
    "Gentoo",
    "Void Linux",
    "NixOS 24.05",
    "Slackware 15.0",
]

# Where Ollama tends to store blobs on different installs
OLLAMA_STORE_CANDIDATES = [
    Path("/usr/share/ollama/.ollama"),
    Path("/var/lib/ollama/.ollama"),
    Path("/var/lib/ollama"),
    Path("/usr/share/ollama"),
    Path.home() / ".ollama",
]

# If you're running as root (sudo -s), apt's _apt sandbox often can't read/write
# into root-owned working dirs. Force sandbox user to root for downloads to
# eliminate the warning and avoid permission churn.
APT_DOWNLOAD_ARGS = "-o APT::Sandbox::User=root"

# --------------------------
# Utilities
# --------------------------

def which(exe: str):
    return shutil.which(exe)

def safe_mkdir(p: Path):
    p.mkdir(parents=True, exist_ok=True)

def now_stamp():
    return time.strftime("%Y%m%d_%H%M%S")

def read_os_release():
    d = {}
    try:
        with open("/etc/os-release", "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                d[k] = v.strip().strip('"')
    except FileNotFoundError:
        pass
    return d

def format_bytes(num: int) -> str:
    suffixes = ["B", "KB", "MB", "GB", "TB"]
    for suffix in suffixes:
        if num < 1024 or suffix == suffixes[-1]:
            return f"{num:.2f} {suffix}"
        num /= 1024
    return f"{num:.2f} B"

def is_ubuntu():
    osr = read_os_release()
    return osr.get("ID", "").lower() == "ubuntu"

def dpkg_arch():
    try:
        out = subprocess.check_output(["dpkg", "--print-architecture"], text=True).strip()
        return out
    except Exception:
        return "unknown"

def is_wsl() -> bool:
    try:
        with open("/proc/version", "r", encoding="utf-8", errors="ignore") as f:
            v = f.read().lower()
        return ("microsoft" in v) or ("wsl" in v)
    except Exception:
        return False

def pid1_name() -> str:
    try:
        out = subprocess.check_output(["ps", "-p", "1", "-o", "comm="], text=True).strip()
        return out
    except Exception:
        return "unknown"

@dataclass
class Profile:
    pretty: str
    os_id: str
    version_id: str
    arch: str
    has_apt: bool
    has_docker: bool
    has_ollama: bool
    is_wsl: bool
    pid1: str

# --------------------------
# Streaming command runner
# --------------------------

class StreamRunner:
    """
    Runs commands and streams stdout/stderr lines to a callback.
    """
    def __init__(self, log_cb):
        self.log = log_cb

    def run(
        self,
        cmd,
        cwd=None,
        env=None,
        sudo=False,
        check=True,
        heartbeat=None,
        heartbeat_interval: float = 5.0,
        line_cb=None,  # per-line hook
    ):
        if sudo:
            cmd = ["sudo", *cmd]
        self.log(f"$ {' '.join(cmd)}\n")

        p = subprocess.Popen(
            cmd,
            cwd=cwd,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            universal_newlines=True,
        )
        assert p.stdout is not None

        def _reader():
            for line in p.stdout:
                if line_cb:
                    try:
                        line_cb(line)
                    except Exception:
                        pass
                self.log(line)

        reader = threading.Thread(target=_reader, daemon=True)
        reader.start()

        start = time.time()
        last_hb = start
        while True:
            rc = p.poll()
            now = time.time()
            if heartbeat and now - last_hb >= heartbeat_interval:
                try:
                    heartbeat(p, now - start)
                except Exception as hb_exc:
                    self.log(f"[heartbeat error: {hb_exc}]\n")
                last_hb = now
            if rc is not None:
                break
            time.sleep(0.25)

        reader.join()

        if check and rc != 0:
            raise RuntimeError(f"Command failed ({rc}): {' '.join(cmd)}")
        return rc

# --------------------------
# Builder
# --------------------------

class Builder:
    def __init__(self, log_cb, progress_cb, status_cb):
        self.log = log_cb
        self.set_progress = progress_cb  # (value, maximum, mode)
        self.set_status = status_cb
        self.runner = StreamRunner(log_cb)

    def profile(self) -> Profile:
        osr = read_os_release()
        return Profile(
            pretty=osr.get("PRETTY_NAME", "unknown"),
            os_id=osr.get("ID", "unknown"),
            version_id=osr.get("VERSION_ID", "unknown"),
            arch=dpkg_arch(),
            has_apt=bool(which("apt-get")),
            has_docker=bool(which("docker")),
            has_ollama=bool(which("ollama")),
            is_wsl=is_wsl(),
            pid1=pid1_name(),
        )

    def ensure_root(self):
        """
        You said you run as sudo -s. Enforce that up-front.
        """
        if os.geteuid() != 0:
            raise RuntimeError("Run as root (sudo -s) before launching this tool.")

    def apt_update(self):
        if not which("apt-get"):
            raise RuntimeError("apt-get not found; this builder currently supports Ubuntu/Debian.")
        self.set_status("Updating apt index...")
        self.runner.run(["apt-get", "update"], sudo=False, check=True)

    def download_debs_closure(self, out_dir: Path, packages: list[str]):
        safe_mkdir(out_dir)
        if not which("apt-get"):
            raise RuntimeError("apt-get not found; cannot download .debs.")

        self.apt_update()

        self.set_status("Downloading base packages (.deb)...")
        self.set_progress(0, 100, "indeterminate")
        self.set_progress(None, None, "start")

        # Force sandbox user to root to eliminate _apt permission warnings when
        # working in root-owned directories.
        for pkg in packages:
            self.runner.run(
                ["bash", "-lc", f"cd '{out_dir}' && apt-get {APT_DOWNLOAD_ARGS} download {pkg}"],
                check=False,
            )

        self.set_progress(None, None, "stop")

        self.set_status("Resolving dependency closure...")
        self.set_progress(0, 6, "determinate")
        self.set_progress(1, 6, "set")

        cmd = [
            "apt-cache", "depends", "--recurse",
            "--no-recommends", "--no-suggests",
            "--no-conflicts", "--no-breaks", "--no-replaces",
            "--no-enhances",
            *packages
        ]
        out = subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)
        deps = set()
        for line in out.splitlines():
            line = line.strip()
            m = re.search(r":\s*([A-Za-z0-9.+-]+)$", line)
            if m:
                deps.add(m.group(1))
        deps = sorted(deps)
        self.log(f"Dependency closure size (best-effort): {len(deps)} packages\n")

        self.set_progress(2, 6, "set")
        self.set_status("Downloading dependency closure (.deb)...")

        chunk = 80
        total_chunks = max(1, (len(deps) + chunk - 1) // chunk)
        self.set_progress(0, total_chunks, "determinate")

        for i in range(total_chunks):
            part = deps[i*chunk:(i+1)*chunk]
            self.set_status(f"Downloading dependency closure chunk {i+1}/{total_chunks}...")
            cmdline = " ".join(part)
            self.runner.run(
                ["bash", "-lc", f"cd '{out_dir}' && apt-get {APT_DOWNLOAD_ARGS} download {cmdline}"],
                check=False
            )
            self.set_progress(i+1, total_chunks, "set")

        for junk in ("partial", "lock"):
            jp = out_dir / junk
            if jp.exists():
                try:
                    if jp.is_dir():
                        shutil.rmtree(jp)
                    else:
                        jp.unlink()
                except Exception:
                    pass

        debs = list(out_dir.glob("*.deb"))
        total = sum(p.stat().st_size for p in debs)
        self.set_status(f".deb ready: {len(debs)} files, {total/1024/1024:.1f} MB")
        self.log(f".deb count: {len(debs)}  size: {total/1024/1024:.1f} MB\n")

    def ensure_build_prereqs(self, auto_install: bool):
        prof = self.profile()

        if prof.is_wsl and prof.pid1 != "systemd":
            self.log("NOTE: WSL detected and PID1 is not systemd. systemctl enable/start docker may not work.\n")

        if not auto_install:
            if not prof.has_docker:
                raise RuntimeError("docker is required on build machine to pull/save Open WebUI image.")
            if not prof.has_ollama:
                raise RuntimeError("ollama is required on build machine to pull models and snapshot the store.")
            return

        if not prof.has_docker:
            self.set_status("Installing Docker on build machine...")
            self.apt_update()
            self.runner.run(["apt-get", "install", "-y", "docker.io", "containerd", "runc"], sudo=False, check=True)
            self.runner.run(["systemctl", "enable", "--now", "docker"], sudo=False, check=False)

        if not which("ollama"):
            self.set_status("Installing Ollama on build machine...")
            self.runner.run(["bash", "-lc", "curl -fsSL https://ollama.com/install.sh | bash"], check=False)
            if not which("ollama"):
                self.runner.run(["bash", "-lc", "curl -fsSL https://ollama.com/install.sh | bash"], check=True)

    def download_ollama_installer(self, out_path: Path):
        safe_mkdir(out_path.parent)
        self.set_status("Downloading Ollama installer script into kit...")
        self.runner.run(["bash", "-lc", f"curl -fsSL https://ollama.com/install.sh -o '{out_path}'"], check=True)
        os.chmod(out_path, 0o755)

    def ollama_pull_models(self, models: list[str]):
        if not which("ollama"):
            raise RuntimeError("ollama not found on build machine.")

        self.set_status("Pulling Ollama models (streaming)...")
        self.set_progress(0, 100, "indeterminate")
        self.set_progress(None, None, "start")
        for m in models:
            self.log(f"--- ollama pull {m} ---\n")
            self.runner.run(["ollama", "pull", m], check=True)
        self.set_progress(None, None, "stop")

    def _find_ollama_store_root(self) -> Path:
        existing = [p for p in OLLAMA_STORE_CANDIDATES if p.exists()]
        if not existing:
            raise RuntimeError("Could not locate Ollama store directory on this machine.")

        def dir_size(p: Path) -> int:
            try:
                out = subprocess.check_output(
                    ["bash", "-lc", f"du -sb '{p}' 2>/dev/null | cut -f1 || echo 0"],
                    text=True,
                ).strip()
                return int(out) if out else 0
            except Exception:
                return 0

        existing.sort(key=dir_size, reverse=True)
        return existing[0]

    def ollama_snapshot_store(self, out_dir: Path) -> Path:
        if not which("ollama"):
            raise RuntimeError("ollama not found on build machine.")
        safe_mkdir(out_dir)

        store = self._find_ollama_store_root()
        out_tgz = out_dir / "ollama_store.tgz"

        store_size = 0
        try:
            out = subprocess.check_output(
                ["bash", "-lc", f"du -sb '{store}' 2>/dev/null | cut -f1"],
                text=True
            ).strip()
            store_size = int(out) if out else 0
        except Exception:
            store_size = 0

        if store_size:
            self.log(f"Ollama store: {store}  size: {format_bytes(store_size)} ({store_size} bytes)\n")
        else:
            self.log(f"Ollama store: {store} (size unknown; du failed)\n")

        self.set_status(f"Snapshotting Ollama store: {store}")
        self.set_progress(0, 100, "indeterminate")
        self.set_progress(None, None, "start")

        def _tar_heartbeat(proc: subprocess.Popen, elapsed: float):
            seconds = int(elapsed)
            status_msg = f"Snapshotting Ollama store... {seconds}s elapsed"
            try:
                ps_out = subprocess.check_output(
                    ["ps", "-p", str(proc.pid), "-o", "%cpu,%mem,etime"],
                    text=True,
                ).splitlines()
                if len(ps_out) >= 2:
                    stats = " ".join(ps_out[1].split())
                    status_msg += f" (cpu/mem/etime: {stats})"
            except Exception:
                pass
            self.set_status(status_msg)

        self.log(f"Creating tarball: {out_tgz}\n")
        self.runner.run(
            ["bash", "-lc",
             f"tar -czf '{out_tgz}' --checkpoint=200 --checkpoint-action=dot -C '{store.parent}' '{store.name}'"],
            check=True,
            heartbeat=_tar_heartbeat,
            heartbeat_interval=5.0,
        )

        self.set_progress(None, None, "stop")
        self.log(f"Ollama store snapshot: {out_tgz} ({out_tgz.stat().st_size/1024/1024/1024:.2f} GB)\n")
        self.set_status("Ollama store snapshot complete.")
        return out_tgz

    def docker_export_openwebui(self, out_tar: Path):
        if not which("docker"):
            raise RuntimeError("docker not found on build machine.")
        safe_mkdir(out_tar.parent)

        self.set_status("Pulling Open WebUI image (streaming)...")
        self.set_progress(0, 100, "indeterminate")
        self.set_progress(None, None, "start")
        self.runner.run(["docker", "pull", OPEN_WEBUI_IMAGE], check=True)
        self.set_progress(None, None, "stop")

        img_size = 0
        try:
            out = subprocess.check_output(
                ["docker", "image", "inspect", OPEN_WEBUI_IMAGE, "--format", "{{.Size}}"],
                text=True,
            ).strip()
            img_size = int(out or "0")
        except Exception:
            img_size = 0

        tmp_tar = Path("/tmp") / out_tar.name

        usage = shutil.disk_usage(out_tar.parent)
        buffer_bytes = 512 * 1024 * 1024
        needed = img_size + buffer_bytes if img_size else None
        if needed and usage.free < needed:
            raise RuntimeError(
                f"Not enough free space in {out_tar.parent} to save image. "
                f"Required (approx): {format_bytes(needed)}, available: {format_bytes(usage.free)}."
            )

        self.set_status("Saving Open WebUI image (docker save with progress)...")
        self.set_progress(0, 100, "indeterminate")
        self.set_progress(None, None, "start")

        try:
            if not which("pv"):
                self.runner.run(["bash", "-lc", "apt-get update && apt-get install -y pv"], check=False)

            if which("pv") and img_size > 0:
                cmd = (
                    "set -euo pipefail; "
                    f"rm -f '{tmp_tar}'; "
                    f"docker save '{OPEN_WEBUI_IMAGE}' | pv -s {img_size} > '{tmp_tar}'"
                )
            else:
                cmd = (
                    "set -euo pipefail; "
                    f"rm -f '{tmp_tar}'; "
                    f"docker save '{OPEN_WEBUI_IMAGE}' -o '{tmp_tar}'"
                )

            self.runner.run(["bash", "-lc", cmd], check=True)

            self.set_status("Copying image tar into kit output folder...")
            self.runner.run(["bash", "-lc", f"cp -f '{tmp_tar}' '{out_tar}' && sync"], check=True)

        except Exception as exc:
            try:
                if tmp_tar.exists():
                    tmp_tar.unlink()
            except Exception:
                pass
            try:
                if out_tar.exists():
                    out_tar.unlink()
            except Exception:
                pass
            raise RuntimeError(f"Failed to export Open WebUI image. Original error: {exc}")

        finally:
            self.set_progress(None, None, "stop")

        self.log(f"open-webui.tar size: {out_tar.stat().st_size/1024/1024/1024:.2f} GB\n")

    def write_offline_scripts(self, scripts_dir: Path, bundle_store_snapshot: bool):
        safe_mkdir(scripts_dir)
        install_sh = scripts_dir / "install_airgap_llm.sh"
        start_sh = scripts_dir / "start_webui.sh"

        restore_models_cmd = ""
        if bundle_store_snapshot:
            restore_models_cmd = r'''
# Restore Ollama store snapshot (version-locked but reliable)
if [ -f "$MODELS/ollama_store.tgz" ]; then
  echo "[*] Restoring Ollama store snapshot..."
  if [ -d /usr/share/ollama ]; then
    mkdir -p /usr/share/ollama
    tar -xzf "$MODELS/ollama_store.tgz" -C /usr/share/ollama || true
  elif [ -d /var/lib/ollama ]; then
    mkdir -p /var/lib/ollama
    tar -xzf "$MODELS/ollama_store.tgz" -C /var/lib/ollama || true
  else
    mkdir -p "$HOME/.ollama"
    tar -xzf "$MODELS/ollama_store.tgz" -C "$HOME" || true
  fi

  if id ollama >/dev/null 2>&1; then
    chown -R ollama:ollama /usr/share/ollama 2>/dev/null || true
    chown -R ollama:ollama /var/lib/ollama 2>/dev/null || true
  fi
fi
'''
        else:
            restore_models_cmd = 'echo "[*] Model snapshot not bundled (unchecked)."\n'

        install_contents = f"""#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PKGS="$ROOT/pkgs"
OLLAMA_DIR="$ROOT/ollama"
IMAGES="$ROOT/images"
MODELS="$ROOT/models"

[[ $EUID -eq 0 ]] || {{ echo "Run as root: sudo $0"; exit 1; }}

echo "[*] Installing .deb packages from $PKGS"
dpkg -i "$PKGS"/*.deb || true
apt-get -f install -y --no-download || true
dpkg -i "$PKGS"/*.deb

echo "[*] Attempting to enable/start docker (best-effort)"
systemctl enable --now docker >/dev/null 2>&1 || true

echo "[*] Installing Ollama"
if [[ -x "$OLLAMA_DIR/ollama-install.sh" ]]; then
  bash "$OLLAMA_DIR/ollama-install.sh" || true
else
  echo "Missing: $OLLAMA_DIR/ollama-install.sh"
  exit 1
fi

systemctl restart ollama >/dev/null 2>&1 || true

echo "[*] Loading Open WebUI image"
docker load -i "$IMAGES/open-webui.tar" || true

{restore_models_cmd}

echo "[*] Install complete."
"""

        start_contents = """#!/usr/bin/env bash
set -euo pipefail

systemctl restart ollama >/dev/null 2>&1 || true

if ! curl -fsS --max-time 2 http://127.0.0.1:11434/api/version >/dev/null 2>&1; then
  nohup ollama serve >/tmp/ollama.log 2>&1 &
  sleep 1
fi

docker rm -f open-webui >/dev/null 2>&1 || true
docker run -d --name open-webui --network=host \
  -e OLLAMA_BASE_URL="http://127.0.0.1:11434" \
  --restart unless-stopped \
  ghcr.io/open-webui/open-webui:main >/dev/null

echo "Open WebUI: http://localhost:8080"
"""

        install_sh.write_text(install_contents, encoding="utf-8")
        start_sh.write_text(start_contents, encoding="utf-8")
        os.chmod(install_sh, 0o755)
        os.chmod(start_sh, 0o755)

    def pack_kit(self, kit_dir: Path, out_tgz: Path):
        safe_mkdir(out_tgz.parent)

        build_dir = Path("/tmp") if str(out_tgz).startswith("/mnt/") else out_tgz.parent
        tmp_final = build_dir / out_tgz.name
        tmp_out = tmp_final.with_suffix(tmp_final.suffix + ".partial")

        for p in (tmp_out, tmp_final, out_tgz):
            try:
                if p.exists():
                    p.unlink()
            except Exception:
                pass

        total_bytes = 0
        try:
            out = subprocess.check_output(
                ["bash", "-lc", f"du -sb '{kit_dir}' 2>/dev/null | cut -f1"],
                text=True,
            ).strip()
            total_bytes = int(out or "0")
        except Exception:
            total_bytes = 0

        if not which("pv"):
            self.runner.run(["bash", "-lc", "apt-get update && apt-get install -y pv"], check=False)

        self.set_status("Packing final bundle (.tgz)...")
        self.set_progress(0, 100, "determinate")
        self.set_progress(0, 100, "set")
        if total_bytes:
            self.log(f"Packing input size (du): {format_bytes(total_bytes)} ({total_bytes} bytes)\n")
        else:
            self.log("Packing input size unknown (du failed); pv progress may be less accurate.\n")

        pv_re = re.compile(r"^\s*([0-9]+(?:\.[0-9]+)?)\s*$")
        last_pct = {"v": 0.0}

        def _line_cb(line: str):
            m = pv_re.match(line.strip())
            if not m:
                return
            pct = float(m.group(1))
            if pct + 0.5 < last_pct["v"]:
                return
            last_pct["v"] = pct
            self.set_progress(pct, 100, "set")

        def _hb(proc: subprocess.Popen, elapsed: float):
            out_size = 0
            try:
                if tmp_out.exists():
                    out_size = tmp_out.stat().st_size
                elif tmp_final.exists():
                    out_size = tmp_final.stat().st_size
                elif out_tgz.exists():
                    out_size = out_tgz.stat().st_size
            except Exception:
                out_size = 0
            self.set_status(
                f"Packing final bundle... {last_pct['v']:5.1f}%  elapsed={int(elapsed)}s  out={format_bytes(out_size)}"
            )

        cmd = (
            "set -euo pipefail; "
            f"rm -f '{tmp_out}'; "
            f"tar -C '{kit_dir.parent}' -cf - '{kit_dir.name}' "
            f"| pv -n -s {max(1, total_bytes)} 2>&1 "
            f"| gzip -1 > '{tmp_out}'; "
            f"mv -f '{tmp_out}' '{tmp_final}'"
        )

        self.log(f"Creating bundle: {out_tgz}\n")
        self.runner.run(
            ["bash", "-lc", cmd],
            check=True,
            heartbeat=_hb,
            heartbeat_interval=2.0,
            line_cb=_line_cb,
        )

        if tmp_final.resolve() != out_tgz.resolve():
            self.set_status("Copying final bundle to output folder...")
            self.runner.run(["bash", "-lc", f"cp -f '{tmp_final}' '{out_tgz}' && sync"], check=True)
            try:
                tmp_final.unlink()
            except Exception:
                pass

        self.set_progress(100, 100, "set")
        self.log(f"Bundle: {out_tgz} ({out_tgz.stat().st_size/1024/1024/1024:.2f} GB)\n")
        self.set_status("Pack complete.")

# --------------------------
# Tkinter GUI
# --------------------------

class PongGame:
    def __init__(self, parent, width: int, height: int, bg: str, paddle_color: str, ball_color: str):
        self.width = width
        self.height = height
        self.canvas = tk.Canvas(parent, width=width, height=height, bg=bg, highlightthickness=0)
        self.paddle_height = 22
        self.paddle_width = 6
        self.ball_radius = 4
        self.paddle_color = paddle_color
        self.ball_color = ball_color
        self.running = False
        self._after = None
        self._ball_pos = [width / 2, height / 2]
        self._ball_vel = [2.6, 1.8]
        self._paddle = None
        self._ball = None
        self.reset()

    def reset(self):
        self.canvas.delete("all")
        mid_y = self.height / 2
        paddle_x = self.width - self.paddle_width - 2
        self._paddle = self.canvas.create_rectangle(
            paddle_x,
            mid_y - self.paddle_height / 2,
            paddle_x + self.paddle_width,
            mid_y + self.paddle_height / 2,
            fill=self.paddle_color,
            width=0,
        )
        self._ball_pos = [self.width / 2, self.height / 2]
        self._ball_vel = [abs(self._ball_vel[0]), self._ball_vel[1]]
        self._ball = self.canvas.create_oval(
            self._ball_pos[0] - self.ball_radius,
            self._ball_pos[1] - self.ball_radius,
            self._ball_pos[0] + self.ball_radius,
            self._ball_pos[1] + self.ball_radius,
            fill=self.ball_color,
            width=0,
        )

    def start(self):
        if self.running:
            return
        self.running = True
        self.reset()
        self._tick()

    def stop(self):
        self.running = False
        if self._after:
            self.canvas.after_cancel(self._after)
            self._after = None

    def move_paddle(self, delta: float):
        if self._paddle is None:
            return
        x1, y1, x2, y2 = self.canvas.coords(self._paddle)
        new_y1 = max(0, min(self.height - self.paddle_height, y1 + delta))
        new_y2 = new_y1 + self.paddle_height
        self.canvas.coords(self._paddle, x1, new_y1, x2, new_y2)

    def _tick(self):
        if not self.running:
            return

        x, y = self._ball_pos
        vx, vy = self._ball_vel

        x += vx
        y += vy

        if y - self.ball_radius <= 0 or y + self.ball_radius >= self.height:
            vy *= -1
            y = max(self.ball_radius, min(self.height - self.ball_radius, y))

        if x - self.ball_radius <= 0:
            vx = abs(vx)
            x = self.ball_radius

        paddle_x1, paddle_y1, paddle_x2, paddle_y2 = self.canvas.coords(self._paddle)
        if x + self.ball_radius >= paddle_x1 and paddle_y1 <= y <= paddle_y2 and vx > 0:
            vx = -abs(vx)
            x = paddle_x1 - self.ball_radius

        if x - self.ball_radius > self.width + 6:
            self.reset()
            self._schedule_next()
            return

        self._ball_pos = [x, y]
        self._ball_vel = [vx, vy]
        self.canvas.coords(
            self._ball,
            x - self.ball_radius,
            y - self.ball_radius,
            x + self.ball_radius,
            y + self.ball_radius,
        )

        self._schedule_next()

    def _schedule_next(self):
        self._after = self.canvas.after(16, self._tick)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        if sys.platform != "linux":
            raise RuntimeError("This builder must run on Linux/WSL.")

        self.title("Airgap LLM Kit Builder")
        self.minsize(980, 680)

        self._bg = "#f5f5f5"
        self._panel_bg = "#ffffff"
        self._fg = "#000000"
        self._accent = "#0b57d0"
        self._apply_light_theme()

        self.msg_q = queue.Queue()

        self.profile_var = tk.StringVar(value="(profiling...)")
        self.target_os_var = tk.StringVar(value=SUPPORTED_LINUX_OSES[0])
        self.remote_docker_var = tk.BooleanVar(value=False)
        self.remote_ollama_var = tk.BooleanVar(value=False)
        self.outdir_var = tk.StringVar(value=str(Path.cwd() / f"airgap_kit_out_{now_stamp()}"))
        self.auto_install_var = tk.BooleanVar(value=True)
        self.snapshot_models_var = tk.BooleanVar(value=False)

        self.status_var = tk.StringVar(value="Idle")
        self.progress_value = tk.DoubleVar(value=0.0)
        self.progress_max = tk.DoubleVar(value=1.0)

        self._build_ui()

        self.builder = Builder(self._enqueue_log, self._enqueue_progress, self._enqueue_status)

        self._profile_now()
        self.after(50, self._drain_queue)

        self.bind_all("<Up>", lambda _e: self.pong_game.move_paddle(-8))
        self.bind_all("<Down>", lambda _e: self.pong_game.move_paddle(8))

    def _apply_light_theme(self):
        self.configure(bg=self._bg)
        style = ttk.Style(self)
        try:
            style.theme_use("default")
        except tk.TclError:
            pass
        style.configure(".", background=self._bg, foreground=self._fg)
        style.configure("TFrame", background=self._bg)
        style.configure("TLabelframe", background=self._bg, foreground=self._fg)
        style.configure("TLabelframe.Label", background=self._bg, foreground=self._fg)
        style.configure("TLabel", background=self._bg, foreground=self._fg)
        style.configure("TButton", background=self._panel_bg, foreground=self._fg)
        style.configure("TCheckbutton", background=self._bg, foreground=self._fg, wraplength=0)
        style.configure("TProgressbar", background=self._accent, troughcolor="#d9d9d9")

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)

        top = ttk.Frame(self, padding=10)
        top.grid(row=0, column=0, sticky="nsew")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="Target Linux OS (airgapped host):").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            top,
            textvariable=self.target_os_var,
            values=SUPPORTED_LINUX_OSES,
            state="readonly",
        ).grid(row=0, column=1, sticky="ew")

        remote_opts = ttk.Frame(top)
        remote_opts.grid(row=1, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Checkbutton(
            remote_opts,
            text="Docker already installed on airgapped host",
            variable=self.remote_docker_var,
        ).pack(side="left")
        ttk.Checkbutton(
            remote_opts,
            text="Ollama already installed on airgapped host",
            variable=self.remote_ollama_var,
        ).pack(side="left", padx=(16, 0))

        ttk.Label(top, text="Build host profile:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Label(top, textvariable=self.profile_var).grid(row=2, column=1, sticky="w", pady=(8, 0))

        ttk.Label(top, text="Output Folder:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        out_entry = ttk.Entry(top, textvariable=self.outdir_var)
        out_entry.grid(row=3, column=1, sticky="ew", pady=(8, 0))
        ttk.Button(top, text="Browse", command=self._browse_outdir).grid(row=3, column=2, padx=(8, 0), pady=(8, 0))

        opts = ttk.Frame(top)
        opts.grid(row=4, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Checkbutton(opts, text="Auto-install build prerequisites (docker + ollama)", variable=self.auto_install_var).pack(side="left")
        ttk.Checkbutton(opts, text="Bundle Ollama store snapshot into kit (recommended)", variable=self.snapshot_models_var).pack(side="left", padx=(16, 0))

        models_frame = ttk.LabelFrame(self, text="Models to pull (one per line)", padding=10)
        models_frame.grid(row=1, column=0, sticky="nsew", padx=10)
        models_frame.columnconfigure(0, weight=1)
        models_frame.rowconfigure(0, weight=1)

        self.models_text = tk.Text(
            models_frame,
            height=6,
            wrap="none",
            bg=self._panel_bg,
            fg=self._fg,
            insertbackground=self._fg,
            highlightthickness=0,
            relief="flat",
        )
        self.models_text.insert("1.0", "deepseek-coder-v2\nqwen3:1.7b\n")
        self.models_text.grid(row=0, column=0, sticky="nsew")

        mscroll_y = ttk.Scrollbar(models_frame, orient="vertical", command=self.models_text.yview)
        mscroll_x = ttk.Scrollbar(models_frame, orient="horizontal", command=self.models_text.xview)
        self.models_text.configure(yscrollcommand=mscroll_y.set, xscrollcommand=mscroll_x.set)
        mscroll_y.grid(row=0, column=1, sticky="ns")
        mscroll_x.grid(row=1, column=0, sticky="ew")

        bar = ttk.Frame(self, padding=(10, 8))
        bar.grid(row=2, column=0, sticky="ew")
        bar.columnconfigure(2, weight=1)

        ttk.Button(bar, text="Re-Profile", command=self._profile_now).grid(row=0, column=0, sticky="w")
        ttk.Button(bar, text="Build Kit", command=self._build_kit_thread).grid(row=0, column=1, sticky="w", padx=(8, 0))

        ttk.Label(bar, textvariable=self.status_var).grid(row=0, column=2, sticky="w", padx=(12, 0))

        self.pong_game = PongGame(
            bar,
            width=100,
            height=60,
            bg=self._panel_bg,
            paddle_color=self._accent,
            ball_color=self._fg,
        )
        self.pong_game.canvas.grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.pong_game.canvas.grid_remove()

        self._pbar_grid = {"row": 1, "column": 1, "columnspan": 2, "sticky": "ew", "pady": (8, 0)}
        self.pbar = ttk.Progressbar(
            bar,
            mode="determinate",
            variable=self.progress_value,
            maximum=self.progress_max.get(),
        )
        self.pbar.grid(**self._pbar_grid)

        log_frame = ttk.LabelFrame(self, text="Log", padding=10)
        log_frame.grid(row=3, column=0, sticky="nsew", padx=10, pady=(0, 10))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(
            log_frame,
            wrap="none",
            bg=self._panel_bg,
            fg=self._fg,
            insertbackground=self._fg,
            highlightthickness=0,
            relief="flat",
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")

        self.log_text.tag_configure("cmd", foreground="#1f75fe")

        lscroll_y = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        lscroll_x = ttk.Scrollbar(log_frame, orient="horizontal", command=self.log_text.xview)
        self.log_text.configure(yscrollcommand=lscroll_y.set, xscrollcommand=lscroll_x.set)
        lscroll_y.grid(row=0, column=1, sticky="ns")
        lscroll_x.grid(row=1, column=0, sticky="ew")

        self.log_text.configure(state="disabled")

        self.grid_rowconfigure(1, weight=0)
        self.grid_rowconfigure(3, weight=1)

    def _enqueue_log(self, line: str):
        self.msg_q.put(("log", line))

    def _enqueue_status(self, s: str):
        self.msg_q.put(("status", s))

    def _enqueue_progress(self, value, maximum, mode):
        self.msg_q.put(("progress", value, maximum, mode))

    def _drain_queue(self):
        try:
            while True:
                item = self.msg_q.get_nowait()
                kind = item[0]
                if kind == "log":
                    self._log_append(item[1])
                elif kind == "status":
                    self.status_var.set(item[1])
                elif kind == "progress":
                    _, value, maximum, mode = item
                    self._progress_apply(value, maximum, mode)
        except queue.Empty:
            pass
        self.after(50, self._drain_queue)

    def _log_append(self, line: str):
        text = (line or "").replace("\r\n", "\n")
        self.log_text.configure(state="normal")

        is_cmd = text.lstrip().startswith("$")
        tags = ("cmd",) if is_cmd else ()

        idx = 0
        while idx < len(text):
            ch = text[idx]
            if ch == "\r":
                line_start = self.log_text.index("end-1c linestart")
                line_end = self.log_text.index("end-1c lineend")
                try:
                    self.log_text.delete(line_start, line_end)
                    self.log_text.mark_set("insert", line_start)
                except Exception:
                    pass
            elif ch == "\n":
                self.log_text.insert("end", "\n", tags)
            else:
                self.log_text.insert("end", ch, tags)
            idx += 1

        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _progress_apply(self, value, maximum, mode):
        if mode in ("determinate", "set"):
            self.pong_game.stop()
            self.pong_game.canvas.grid_remove()
            if not self.pbar.winfo_ismapped():
                self.pbar.grid(**self._pbar_grid)
            self.pbar.configure(mode="determinate")
        elif mode == "indeterminate":
            self.pbar.configure(mode="indeterminate")
        if maximum is not None:
            self.progress_max.set(float(maximum))
            self.pbar.configure(maximum=self.progress_max.get())
        if value is not None:
            self.progress_value.set(float(value))
        if mode == "start":
            self.pbar.grid_remove()
            self.pong_game.canvas.grid()
            self.pong_game.start()
        elif mode == "stop":
            self.pong_game.stop()
            self.pong_game.canvas.grid_remove()
            if not self.pbar.winfo_ismapped():
                self.pbar.grid(**self._pbar_grid)

    def _browse_outdir(self):
        d = filedialog.askdirectory(initialdir=str(Path.cwd()))
        if d:
            self.outdir_var.set(d)

    def _profile_now(self):
        try:
            p = Builder(lambda _: None, lambda *_: None, lambda *_: None).profile()
            extra = []
            if p.is_wsl:
                extra.append(f"WSL pid1={p.pid1}")
            self.profile_var.set(
                f"{p.pretty}  arch={p.arch}  apt={'yes' if p.has_apt else 'no'}  "
                f"docker={'yes' if p.has_docker else 'no'}  ollama={'yes' if p.has_ollama else 'no'}"
                + (f"  ({', '.join(extra)})" if extra else "")
            )
            if not is_ubuntu():
                self._log_append("Warning: Non-Ubuntu OS detected. apt-based downloads may fail.\n")
        except Exception as e:
            self.profile_var.set(f"(profile failed: {e})")

    def _build_kit_thread(self):
        t = threading.Thread(target=self._build_kit, daemon=True)
        t.start()

    def _build_kit(self):
        try:
            self.builder.ensure_root()

            phases = [
                "Prepare output folders",
                "Download offline .debs",
                "Download Ollama installer into kit",
                "Ensure build prereqs",
                "Pull models",
                "Snapshot Ollama store",
                "Export Open WebUI image",
                "Write offline scripts",
                "Pack final tgz",
            ]
            total_phases = len(phases)

            def phase_status(idx: int) -> str:
                return f"[{idx+1}/{total_phases}] {phases[idx]}"

            self._enqueue_progress(0, total_phases, "determinate")
            self._enqueue_status(f"[0/{total_phases}] Starting...")

            outdir = Path(self.outdir_var.get()).expanduser().resolve()
            session_state = outdir / ".kit_session"

            models_raw = self.models_text.get("1.0", "end").strip()
            models = [m.strip() for m in models_raw.splitlines() if m.strip()]
            if not models:
                raise RuntimeError("No models specified.")

            target_os = self.target_os_var.get()
            remote_docker = bool(self.remote_docker_var.get())
            remote_ollama = bool(self.remote_ollama_var.get())

            self._enqueue_log(f"Target Linux OS: {target_os}\n")
            self._enqueue_log(f"Remote host docker installed: {'yes' if remote_docker else 'no'}\n")
            self._enqueue_log(f"Remote host ollama installed: {'yes' if remote_ollama else 'no'}\n")

            prof = self.builder.profile()
            kit_name = None
            if session_state.exists():
                try:
                    kit_name = session_state.read_text(encoding="utf-8").strip()
                except Exception:
                    kit_name = None
            if kit_name:
                self._enqueue_log(f"Reusing existing session kit: {kit_name}\n")
            else:
                kit_name = f"airgap_kit_{prof.os_id}{prof.version_id}_{prof.arch}_{now_stamp()}"
                safe_mkdir(outdir)
                session_state.write_text(kit_name, encoding="utf-8")
                self._enqueue_log(f"Session state saved to {session_state}\n")

            kit_dir = outdir / kit_name
            pkgs_dir = kit_dir / "pkgs"
            ollama_dir = kit_dir / "ollama"
            models_dir = kit_dir / "models"
            images_dir = kit_dir / "images"
            scripts_dir = kit_dir / "scripts"

            self._enqueue_status(phase_status(0))
            safe_mkdir(pkgs_dir)
            safe_mkdir(ollama_dir)
            safe_mkdir(models_dir)
            safe_mkdir(images_dir)
            safe_mkdir(scripts_dir)
            self._enqueue_log(f"Kit dir: {kit_dir}\n")
            self._enqueue_progress(1, len(phases), "set")

            self._enqueue_status(phase_status(1))
            if any(pkgs_dir.glob("*.deb")):
                self._enqueue_log("Reusing previously downloaded .deb files.\n")
            else:
                self.builder.download_debs_closure(pkgs_dir, DEFAULT_APT_PACKAGES)
            self._enqueue_progress(2, len(phases), "set")

            self._enqueue_status(phase_status(2))
            ollama_installer = ollama_dir / "ollama-install.sh"
            if ollama_installer.exists():
                self._enqueue_log("Reusing existing ollama installer.\n")
            else:
                self.builder.download_ollama_installer(ollama_installer)
            self._enqueue_progress(3, len(phases), "set")

            self._enqueue_status(phase_status(3))
            self.builder.ensure_build_prereqs(auto_install=bool(self.auto_install_var.get()))
            self._enqueue_progress(4, len(phases), "set")

            self._enqueue_status(phase_status(4))
            existing_snapshot = (models_dir / "ollama_store.tgz").exists()
            if existing_snapshot:
                self._enqueue_log("Existing Ollama store snapshot found; skipping model pull.\n")
            else:
                self.builder.ollama_pull_models(models)
            self._enqueue_progress(5, len(phases), "set")

            bundle_store = bool(self.snapshot_models_var.get())
            if bundle_store:
                self._enqueue_status(phase_status(5))
                if existing_snapshot:
                    self._enqueue_log("Reusing Ollama store snapshot.\n")
                else:
                    self.builder.ollama_snapshot_store(models_dir)
            else:
                self._enqueue_log("Skipping Ollama store snapshot (unchecked).\n")
            self._enqueue_progress(6, len(phases), "set")

            self._enqueue_status(phase_status(6))
            openwebui_tar = images_dir / "open-webui.tar"
            if openwebui_tar.exists():
                self._enqueue_log("Reusing previously saved Open WebUI image.\n")
            else:
                self.builder.docker_export_openwebui(openwebui_tar)
            self._enqueue_progress(7, len(phases), "set")

            self._enqueue_status(phase_status(7))
            self.builder.write_offline_scripts(scripts_dir, bundle_store_snapshot=bundle_store)
            self._enqueue_progress(8, len(phases), "set")

            self._enqueue_status(phase_status(8))
            out_tgz = outdir / f"{kit_name}.tgz"
            self.builder.pack_kit(kit_dir, out_tgz)
            self._enqueue_progress(9, len(phases), "set")

            self._enqueue_status("DONE\n")
            self._enqueue_log(f"\nDeliverable: {out_tgz}\n")
            self._enqueue_log("Offline machine:\n")
            self._enqueue_log(f"  tar -xzf {out_tgz.name}\n")
            self._enqueue_log(f"  cd {kit_name}\n")
            self._enqueue_log("  sudo ./scripts/install_airgap_llm.sh\n")
            self._enqueue_log("  ./scripts/start_webui.sh\n")
            self._enqueue_log("  Open: http://localhost:8080\n")

            self.after(0, lambda: messagebox.showinfo("Success", f"Kit built:\n{out_tgz}"))

            if session_state.exists():
                session_state.unlink(missing_ok=True)

        except Exception as e:
            self._enqueue_log(f"\nERROR: {e}\n")
            self._enqueue_status("FAILED")
            self.after(0, lambda err=e: messagebox.showerror("Build failed", str(err)))

def main():
    def _noop(*_args, **_kwargs):
        return None

    # Enforce root since you run this via sudo -s
    if os.geteuid() != 0:
        print("Run as root (sudo -s) before launching.", file=sys.stderr)
        sys.exit(1)

    app = App()
    app.mainloop()

if __name__ == "__main__":
    main()

