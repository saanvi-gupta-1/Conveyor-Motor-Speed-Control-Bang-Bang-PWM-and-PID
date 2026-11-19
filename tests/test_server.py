import json
import urllib.request
import urllib.error
import pytest

from conveyor_control.server import start_server


@pytest.fixture(scope="module")
def server_instance():
    server = start_server(host="127.0.0.1", port=8055)
    yield "http://127.0.0.1:8055"
    server.shutdown()


def test_server_serves_html(server_instance):
    url = f"{server_instance}/"
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req) as resp:
        assert resp.status == 200
        content = resp.read().decode("utf-8")
        assert "Conveyor Motor Speed Control" in content
        assert "conveyorCanvas" in content


def test_server_telemetry_endpoint(server_instance):
    url = f"{server_instance}/api/telemetry"
    with urllib.request.urlopen(url) as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode("utf-8"))
        assert "actual_speed_m_s" in data
        assert "target_speed_m_s" in data
        assert "controller" in data


def test_server_action_endpoint(server_instance):
    url = f"{server_instance}/api/action"
    payload = json.dumps({"action": "step"}).encode("utf-8")
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode("utf-8"))
        assert data["status"] == "stepped"


def test_server_control_endpoint(server_instance):
    url = f"{server_instance}/api/control"
    payload = json.dumps({"target_speed": 2.1, "controller": "pwm"}).encode("utf-8")
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode("utf-8"))
        assert data["status"] == "updated"
        assert data["telemetry"]["target_speed_m_s"] == 2.1
        assert data["telemetry"]["controller"] == "pwm"


def test_server_compare_endpoint(server_instance):
    url = f"{server_instance}/api/compare"
    payload = json.dumps({"scenario": "baseline"}).encode("utf-8")
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as resp:
        assert resp.status == 200
        data = json.loads(resp.read().decode("utf-8"))
        assert "results" in data
        assert "pid" in data["results"]
        assert "pwm" in data["results"]
        assert "bang-bang" in data["results"]
        assert "interpretation" in data
