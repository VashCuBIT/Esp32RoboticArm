import json
import time
import asyncio
import httpx
from server.config import (
    ESP32_DEFAULT_IP, ESP32_PORT, TECHNICAL_PINS,
    DEFAULT_HOME_CONFIG, DEFAULT_SERVO_LIMITS, ARM_CONFIG_FILE
)

class ESP32Client:
    def __init__(self):
        self.ip = ESP32_DEFAULT_IP
        self.port = ESP32_PORT
        self.is_connected = False
        self.is_simulated = True
        
        # System status
        self.system_enabled = True
        
        self.speed = 60
        self.is_moving = False
        self._is_sending = False
        self._needs_send = False
        self.home_config = dict(DEFAULT_HOME_CONFIG)
        
        # Pin mapping (Servo ID -> GPIO pin)
        self.pins_mapping = {
            "garra_abertura": 13,
            "garra_rotacao": 14,
            "ombro_slave": 18,
            "punho": 19,
            "base_rotacao": 25,
            "cotovelo": 26,
            "ombro_master": 27
        }

        # 7 Motors Limit ranges (0..180° physical range, centered at 90° [-90°..+90°])
        self.servo_limits = {k: dict(v) for k, v in DEFAULT_SERVO_LIMITS.items()}

        # Load persisted home configuration, pins, and servo limits if existing
        self._load_persisted_config()

        # Exact angles (in transit) vs target angles (commanded goal)
        self.current_angles = {k: float(v) for k, v in self.home_config.items()}
        self.target_angles = dict(self.home_config)

        self.last_check_time = 0.0
        self._client: httpx.AsyncClient | None = None

    def get_effective_shoulder_limits(self) -> dict:
        """
        Calculates the safe, coupled physical motion limits for the shoulder joint.
        The shoulder has 2 motors working together:
          - ombro_master (MG996R)
          - ombro_slave (MG90S, inverted: slave = 180 - master)
        To ensure neither motor binds or is forced against mechanical limits:
          eff_min = max(master_min, 180 - slave_max)
          eff_max = min(master_max, 180 - slave_min)
        """
        master = self.servo_limits.get("ombro_master", {"min": 35, "max": 145})
        slave = self.servo_limits.get("ombro_slave", {"min": 35, "max": 145})
        eff_min = max(master["min"], 180 - slave["max"])
        eff_max = min(master["max"], 180 - slave["min"])
        if eff_min > eff_max:
            eff_min = master["min"]
            eff_max = master["max"]
        return {"min": eff_min, "max": eff_max}

    def _load_persisted_config(self):
        """Loads saved home_config, pins mapping, and servo limits from disk if available."""
        if ARM_CONFIG_FILE.exists():
            try:
                with open(ARM_CONFIG_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    if "esp32_ip" in data and isinstance(data["esp32_ip"], str) and data["esp32_ip"].strip():
                        self.set_ip(data["esp32_ip"], save=False)
                    if "home_config" in data and isinstance(data["home_config"], dict):
                        for k, v in data["home_config"].items():
                            if k in self.home_config:
                                self.home_config[k] = int(v)
                    if "pins" in data and isinstance(data["pins"], dict):
                        for k, v in data["pins"].items():
                            if k in self.pins_mapping:
                                self.pins_mapping[k] = int(v)
                    if "limits" in data and isinstance(data["limits"], dict):
                        for k, v in data["limits"].items():
                            if k in self.servo_limits and isinstance(v, dict):
                                min_v = max(0, min(180, int(v.get("min", self.servo_limits[k]["min"]))))
                                max_v = max(0, min(180, int(v.get("max", self.servo_limits[k]["max"]))))
                                if min_v > max_v:
                                    min_v, max_v = max_v, min_v
                                self.servo_limits[k]["min"] = min_v
                                self.servo_limits[k]["max"] = max_v
            except Exception as e:
                print(f"[ESP32Client] Erro ao carregar {ARM_CONFIG_FILE.name}: {e}")

    def _save_persisted_config(self):
        """Saves current home_config, pins mapping, servo limits, and esp32_ip to disk."""
        try:
            persisted_limits = {
                k: {"min": v["min"], "max": v["max"]}
                for k, v in self.servo_limits.items()
            }
            with open(ARM_CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump({
                    "esp32_ip": self.ip,
                    "home_config": self.home_config,
                    "pins": self.pins_mapping,
                    "limits": persisted_limits
                }, f, indent=2)
        except Exception as e:
            print(f"[ESP32Client] Erro ao salvar {ARM_CONFIG_FILE.name}: {e}")

    def _get_client(self) -> httpx.AsyncClient:
        if self._client is None or self._client.is_closed:
            self._client = httpx.AsyncClient(
                timeout=httpx.Timeout(1.0, connect=0.5),
                limits=httpx.Limits(max_keepalive_connections=5, max_connections=10)
            )
        return self._client

    def set_ip(self, raw_ip: str, save: bool = True) -> str:
        """Sanitizes raw IP/URL input, extracts port if specified, and persists."""
        ip = raw_ip.strip()
        if ip.startswith("http://"):
            ip = ip[7:]
        elif ip.startswith("https://"):
            ip = ip[8:]
        if ":" in ip:
            ip_part, port_part = ip.split(":", 1)
            ip = ip_part
            try:
                self.port = int(port_part.split("/")[0])
            except ValueError:
                pass
        if "/" in ip:
            ip = ip.split("/", 1)[0]
        self.ip = ip
        if save:
            self._save_persisted_config()
        return self.ip

    @property
    def base_url(self):
        return f"http://{self.ip}:{self.port}"

    async def check_connection(self) -> bool:
        """Pings ESP32 status endpoint to verify if hardware is online."""
        try:
            client = self._get_client()
            resp = await client.get(f"{self.base_url}/status", timeout=1.2)
            if resp.status_code == 200:
                data = resp.json()
                was_disconnected = not self.is_connected
                self.is_connected = True
                self.is_simulated = False
                self.system_enabled = data.get("habilitado", True)
                # Sync angles if available
                for m in data.get("motores", []):
                    nome = m.get("nome")
                    atual = m.get("atual")
                    alvo = m.get("alvo")
                    # Map ombro_master to ombro for frontend compatibility
                    key = "ombro" if nome == "ombro_master" else nome
                    if key in self.current_angles and atual is not None:
                        self.current_angles[key] = float(atual)
                    if key in self.target_angles and alvo is not None:
                        self.target_angles[key] = int(alvo)

                # Se acabou de conectar/reconectar, sincroniza imediatamente as configurações salvas (Home, Pinos e Limites)
                if was_disconnected:
                    asyncio.create_task(self.sync_all_persisted_to_esp32())
                return True
        except Exception:
            pass

        self.is_connected = False
        self.is_simulated = True
        return False

    async def sync_all_persisted_to_esp32(self):
        """Pushes persisted home_config, pins_mapping, and servo_limits to the ESP32 upon connection."""
        if not self.is_connected:
            return
        try:
            client = self._get_client()
            # 1. Sincroniza Posição Home salva
            await client.post(
                f"{self.base_url}/home/config",
                data={k: str(v) for k, v in self.home_config.items()},
                params=self.home_config,
                timeout=2.0
            )
            # 2. Sincroniza Mapeamento de Pinos
            await client.post(
                f"{self.base_url}/pins",
                json=self.pins_mapping,
                timeout=2.0
            )
            # 3. Sincroniza Limites dos 7 Motores
            limits_payload = {}
            for m_id, lim in self.servo_limits.items():
                limits_payload[f"{m_id}_min"] = lim["min"]
                limits_payload[f"{m_id}_max"] = lim["max"]
            await client.post(
                f"{self.base_url}/limits",
                params=limits_payload,
                json=limits_payload,
                timeout=2.0
            )
            print("[ESP32Client] Configurações persistidas (Home, Pinos e Limites) sincronizadas com o ESP32!")
        except Exception as e:
            print(f"[ESP32Client] Aviso: Falha ao sincronizar configurações com ESP32: {e}")

    async def poll_hardware_telemetry(self) -> bool:
        """Queries the ultra-fast /telemetry endpoint on ESP32 in real time."""
        if not self.is_connected:
            return False
        try:
            client = self._get_client()
            resp = await client.get(f"{self.base_url}/telemetry", timeout=0.35)
            if resp.status_code == 200:
                data = resp.json()
                # Fast mapping from ESP32 compact keys
                if "b" in data: self.current_angles["base_rotacao"] = float(data["b"])
                if "o" in data: self.current_angles["ombro"] = float(data["o"])
                if "c" in data: self.current_angles["cotovelo"] = float(data["c"])
                if "p" in data: self.current_angles["punho"] = float(data["p"])
                if "gr" in data: self.current_angles["garra_rotacao"] = float(data["gr"])
                if "ga" in data: self.current_angles["garra_abertura"] = float(data["ga"])
                self.is_moving = bool(data.get("m", 0))
                return True
        except Exception:
            pass
        return False

    def step_simulation(self, dt: float = 0.033) -> bool:
        """
        Simulates step-by-step physical servo motor kinematics in real time.
        Wrist and claw micro-servos (MG90S) move 2.5x faster for high responsiveness.
        """
        changed = False
        any_moving = False
        base_speed = max(5.0, float(self.speed))

        for joint, target in self.target_angles.items():
            curr = self.current_angles.get(joint, float(target))
            diff = float(target) - curr
            if abs(diff) > 0.01:
                any_moving = True
                # Punho e garras (MG90S) se movem 2.5x mais rápido
                joint_speed = base_speed * 2.5 if joint in ("punho", "garra_rotacao", "garra_abertura") else base_speed
                step = joint_speed * dt
                if abs(diff) <= step:
                    self.current_angles[joint] = float(target)
                else:
                    self.current_angles[joint] = curr + (step if diff > 0 else -step)
                changed = True

        self.is_moving = any_moving
        return changed

    async def send_move(self, angles: dict, speed: int = 60) -> dict:
        """Sends joint movement command to ESP32 or registers movement in simulator."""
        self.speed = max(1, min(180, int(speed)))
        eff_shoulder = self.get_effective_shoulder_limits()
        
        # Clamp and store target angles
        for k, v in angles.items():
            if k in self.target_angles:
                if k == "ombro":
                    # Coordinated dual-motor shoulder clamping
                    val = max(eff_shoulder["min"], min(eff_shoulder["max"], int(v)))
                else:
                    lim = self.servo_limits.get(k, {"min": 0, "max": 180})
                    min_val = lim.get("min", 0)
                    max_val = lim.get("max", 180)
                    val = max(min_val, min(max_val, int(v)))
                self.target_angles[k] = val
                if not self.is_connected and self.speed >= 120:
                    self.current_angles[k] = float(val)

        self.is_moving = any(
            abs(self.current_angles[k] - self.target_angles[k]) > 0.5
            for k in self.target_angles
        )

        if self.is_connected:
            self._needs_send = True
            if not self._is_sending:
                self._is_sending = True
                asyncio.create_task(self._do_send())
            return {"success": True, "simulated": False, "target_angles": self.target_angles}

        return {
            "success": True,
            "simulated": True,
            "target_angles": self.target_angles,
            "current_angles": {k: int(round(v)) for k, v in self.current_angles.items()}
        }

    async def _do_send(self):
        """Dedicated background task to send the freshest angles to ESP32 without queuing old ones."""
        while self._needs_send:
            self._needs_send = False
            try:
                client = self._get_client()
                payload = {k: int(v) for k, v in self.target_angles.items()}
                payload["speed"] = self.speed
                resp = await client.post(
                    f"{self.base_url}/move",
                    params=payload,
                    json=payload,
                    timeout=0.6 # Short timeout to fail fast and recover
                )
            except Exception as e:
                print(f"[ESP32Client] Send move error: {e}")
                self.is_connected = False
                self.is_simulated = True
                break
        self._is_sending = False

    async def send_home(self) -> dict:
        """Moves arm to the configured home position."""
        return await self.send_move(self.home_config, speed=35)

    async def set_home_config(self, new_config: dict) -> dict:
        """Updates the default/custom home position for each joint, persists to disk and notifies ESP32."""
        for k, v in new_config.items():
            if k in self.home_config:
                self.home_config[k] = int(v)

        self._save_persisted_config()

        if self.is_connected:
            try:
                client = self._get_client()
                await client.post(
                    f"{self.base_url}/home/config",
                    data={k: str(v) for k, v in self.home_config.items()},
                    params=self.home_config,
                    timeout=2.0
                )
            except Exception as e:
                print(f"[ESP32Client] Erro ao sincronizar home/config com ESP32: {e}")

        return {"success": True, "home_config": self.home_config}

    async def set_pins(self, new_pins: dict) -> dict:
        """Updates dynamic GPIO pin assignments for servos and persists to disk."""
        for k, v in new_pins.items():
            if k in self.pins_mapping:
                self.pins_mapping[k] = int(v)

        self._save_persisted_config()

        if self.is_connected:
            try:
                client = self._get_client()
                await client.post(f"{self.base_url}/pins", json=self.pins_mapping, timeout=2.0)
            except Exception:
                pass

        return {"success": True, "pins": self.pins_mapping}

    async def enable_system(self) -> dict:
        self.system_enabled = True
        if self.is_connected:
            try:
                client = self._get_client()
                await client.get(f"{self.base_url}/habilitar", timeout=1.5)
            except Exception:
                pass
        return {"success": True, "system_enabled": True}

    async def disable_system(self) -> dict:
        self.system_enabled = False
        if self.is_connected:
            try:
                client = self._get_client()
                await client.get(f"{self.base_url}/desabilitar", timeout=1.5)
            except Exception:
                pass
        return {"success": True, "system_enabled": False}

    async def stop_motion(self) -> dict:
        # Halt in place: target becomes current position
        for k in self.target_angles:
            self.target_angles[k] = int(round(self.current_angles[k]))
        self.is_moving = False

        if self.is_connected:
            try:
                client = self._get_client()
                await client.get(f"{self.base_url}/stop", timeout=1.5)
            except Exception:
                pass
        return {"success": True, "status": "stopped"}

    async def stream_audio_chunk(self, pcm_chunk: bytes) -> bool:
        """Streams audio chunk to ESP32 I2S speaker endpoint."""
        if not self.is_connected:
            return False
        try:
            client = self._get_client()
            await client.post(
                f"{self.base_url}/audio/stream",
                content=pcm_chunk,
                headers={"Content-Type": "application/octet-stream"},
                timeout=0.6
            )
            return True
        except Exception:
            return False

    async def capture_mic_audio(self) -> bytes:
        """Captures raw stereo audio buffer from ESP32 I2S INMP441 microphones."""
        if not self.is_connected:
            return b""
        try:
            client = self._get_client()
            resp = await client.get(f"{self.base_url}/mic/capture", timeout=3.0)
            if resp.status_code == 200:
                return resp.content
        except Exception as e:
            print(f"[ESP32Client] Mic capture error: {e}")
        return b""

    async def set_limits(self, new_limits: dict) -> dict:
        """
        Updates motion range limits (min and max, within 0..180 degrees) for each of the 7 motors.
        Persists to arm_config.json and synchronizes with ESP32 if online.
        """
        for motor_id, lims in new_limits.items():
            if motor_id in self.servo_limits and isinstance(lims, dict):
                min_v = int(lims.get("min", self.servo_limits[motor_id]["min"]))
                max_v = int(lims.get("max", self.servo_limits[motor_id]["max"]))
                # Clamp within physical limits [0, 180]
                min_v = max(0, min(180, min_v))
                max_v = max(0, min(180, max_v))
                if min_v > max_v:
                    min_v, max_v = max_v, min_v
                self.servo_limits[motor_id]["min"] = min_v
                self.servo_limits[motor_id]["max"] = max_v

        self._save_persisted_config()

        if self.is_connected:
            try:
                client = self._get_client()
                # Prepare payload for ESP32 /limits
                payload = {}
                for m_id, lim in self.servo_limits.items():
                    payload[f"{m_id}_min"] = lim["min"]
                    payload[f"{m_id}_max"] = lim["max"]
                await client.post(
                    f"{self.base_url}/limits",
                    params=payload,
                    json=payload,
                    timeout=2.0
                )
            except Exception as e:
                print(f"[ESP32Client] Erro ao sincronizar limits com ESP32: {e}")

        return {
            "success": True,
            "limits": self.servo_limits,
            "effective_shoulder": self.get_effective_shoulder_limits()
        }

    def get_status(self) -> dict:
        return {
            "success": True,
            "connected": self.is_connected,
            "simulated": self.is_simulated,
            "ip": self.ip,
            "port": self.port,
            "system_enabled": self.system_enabled,
            "is_moving": self.is_moving,
            "angles": {k: int(round(v)) for k, v in self.current_angles.items()},
            "target_angles": dict(self.target_angles),
            "home_config": self.home_config,
            "pins": self.pins_mapping,
            "technical_pins": TECHNICAL_PINS,
            "limits": self.servo_limits,
            "effective_shoulder": self.get_effective_shoulder_limits()
        }

esp32_client = ESP32Client()
