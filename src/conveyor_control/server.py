"""High-performance HTTP API and Web Dashboard Server for Conveyor Motor Control."""

import csv
import io
import json
import mimetypes
from pathlib import Path
import subprocess
import sys
import threading
from typing import Any, Dict
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

from .live_engine import LiveSimulationEngine
from .experiments import run_suite, run_pwm_frequency_sweep, make_controller
from .simulation import SimulationConfig, run_simulation
from .metrics import calculate_metrics
from .spectrum import analyze_speed_spectrum
from .results import interpretation

# Global singleton live engine
engine = LiveSimulationEngine(dt=0.01)

# Path to static frontend files
STATIC_DIR = Path(__file__).parent / "web"


class DashboardHandler(BaseHTTPRequestHandler):
    """Handles REST API and static frontend assets."""

    def log_message(self, format: str, *args: Any) -> None:
        """Suppress noisy request logs in production console."""
        pass

    def _set_headers(self, content_type: str = "application/json", status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.end_headers()

    def do_OPTIONS(self) -> None:
        self._set_headers("text/plain", 204)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/telemetry":
            data = engine.get_telemetry()
            self._set_headers("application/json")
            self.wfile.write(json.dumps(data).encode("utf-8"))
            return

        elif path == "/api/export-csv":
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Disposition", 'attachment; filename="conveyor_live_telemetry.csv"')
            self.end_headers()
            output = io.StringIO()
            if engine.session_data:
                writer = csv.DictWriter(output, fieldnames=list(engine.session_data[0].keys()))
                writer.writeheader()
                writer.writerows(engine.session_data)
            else:
                output.write("time_s,target_speed_m_s,actual_speed_m_s,target_rpm,actual_rpm,error_m_s,command,load_torque_Nm,controller\n")
            self.wfile.write(output.getvalue().encode("utf-8"))
            return

        # Serve static assets or index.html
        if path == "/" or path == "/index.html":
            file_path = STATIC_DIR / "index.html"
        else:
            file_path = STATIC_DIR / path.lstrip("/")

        if file_path.exists() and file_path.is_file():
            mime, _ = mimetypes.guess_type(str(file_path))
            self._set_headers(mime or "text/plain")
            with open(file_path, "rb") as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404, f"File not found: {path}")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length).decode("utf-8") if content_length > 0 else "{}"
        try:
            req_data = json.loads(body) if body else {}
        except Exception:
            req_data = {}

        if path == "/api/action":
            action = req_data.get("action")
            if action == "start":
                engine.start()
                resp = {"status": "started", "is_running": True}
            elif action == "pause":
                engine.pause()
                resp = {"status": "paused", "is_running": False}
            elif action == "reset":
                engine.reset()
                resp = {"status": "reset", "is_running": False}
            elif action == "step":
                sample = engine.step()
                resp = {"status": "stepped", "sample": sample}
            elif action == "disturbance":
                dtype = req_data.get("type", "heavy_box")
                res = engine.inject_disturbance(dtype)
                resp = {"status": "injected", **res}
            elif action == "launch_pybullet":
                def _launch_bullet():
                    try:
                        # Generate 3 seconds of reference data to visualize
                        cfg = SimulationConfig(duration=8.0, target_speed=engine.target_speed)
                        ctrl = make_controller(engine.active_controller_name)
                        rows = run_simulation(ctrl, "load_cycle", cfg)
                        from .visualization import replay_pybullet
                        replay_pybullet(rows)
                    except Exception as e:
                        print(f"PyBullet visualizer error: {e}", file=sys.stderr)

                threading.Thread(target=_launch_bullet, daemon=True).start()
                resp = {"status": "launched", "message": "PyBullet 3D visualizer started in separate window"}
            else:
                resp = {"error": f"Unknown action: {action}"}

            self._set_headers("application/json")
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        elif path == "/api/control":
            if "target_speed" in req_data:
                engine.set_target_speed(float(req_data["target_speed"]))
            if "manual_load" in req_data:
                engine.set_manual_load(float(req_data["manual_load"]))
            if "controller" in req_data:
                engine.set_controller(str(req_data["controller"]))
            if "kp" in req_data or "ki" in req_data or "kd" in req_data:
                engine.update_pid_params(
                    kp=float(req_data.get("kp", 0.7)),
                    ki=float(req_data.get("ki", 0.8)),
                    kd=float(req_data.get("kd", 0.01)),
                )
            if "pwm_freq" in req_data:
                engine.update_pwm_params(frequency_hz=float(req_data["pwm_freq"]))
            if "bangbang_deadband" in req_data:
                engine.update_bangbang_params(deadband=float(req_data["bangbang_deadband"]))

            resp = {"status": "updated", "telemetry": engine.get_telemetry()}
            self._set_headers("application/json")
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        elif path == "/api/compare":
            # Run comparison suite across all 3 controllers on the requested scenario
            scenario = req_data.get("scenario", "baseline")
            cfg = SimulationConfig(duration=8.0, target_speed=engine.target_speed)
            results = {}
            for name in ("bang-bang", "pwm", "pid"):
                ctrl = make_controller(name)
                rows = run_simulation(ctrl, scenario, cfg)
                metrics = calculate_metrics(rows)
                spectrum = analyze_speed_spectrum(rows)
                metrics.update({
                    "dominant_ripple_frequency_hz": spectrum["dominant_frequency_hz"],
                    "dominant_ripple_amplitude_m_s": spectrum["dominant_amplitude_m_s"],
                })
                # Subsample rows for lightweight web transfer (every 4th sample)
                subsampled = rows[::4]
                results[name] = {"metrics": metrics, "rows": subsampled}

            interp = interpretation(results)
            resp = {"scenario": scenario, "results": results, "interpretation": interp}
            self._set_headers("application/json")
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        elif path == "/api/experiment":
            exp_id = req_data.get("experiment", "baseline")
            cfg = SimulationConfig(duration=8.0, target_speed=engine.target_speed)
            results = {}
            for name in ("bang-bang", "pwm", "pid"):
                ctrl = make_controller(name)
                rows = run_simulation(ctrl, exp_id, cfg)
                metrics = calculate_metrics(rows)
                spectrum = analyze_speed_spectrum(rows)
                metrics.update({
                    "dominant_ripple_frequency_hz": spectrum["dominant_frequency_hz"],
                    "dominant_ripple_amplitude_m_s": spectrum["dominant_amplitude_m_s"],
                })
                results[name] = {"metrics": metrics, "rows": rows[::4]}

            resp = {"experiment": exp_id, "results": results, "interpretation": interpretation(results)}
            self._set_headers("application/json")
            self.wfile.write(json.dumps(resp).encode("utf-8"))
            return

        self.send_error(404, "Endpoint not found")


def start_server(host: str = "127.0.0.1", port: int = 8050) -> ThreadingHTTPServer:
    """Start the dashboard server on the specified port."""
    server = ThreadingHTTPServer((host, port), DashboardHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    return server
