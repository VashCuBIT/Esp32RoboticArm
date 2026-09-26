import os
import json
import time
import asyncio
from pathlib import Path
from fastapi import FastAPI, UploadFile, File, Form, WebSocket, WebSocketDisconnect, BackgroundTasks, Request
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from server.config import (
    STATIC_DIR, MODELS_3D_DIR, UPLOADS_DIR,
    SERVER_HOST, SERVER_PORT
)
from server.esp32_client import esp32_client
from server.music_service import music_service
from server.ai_service import ai_service

app = FastAPI(title="ESP32 Robotic Arm Control Hub")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.middleware("http")
async def add_no_cache_headers(request: Request, call_next):
    response = await call_next(request)
    if request.url.path.startswith("/web") or request.url.path == "/":
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response

# Connected WebSocket clients for telemetry
connected_clients = set()
dance_running = False
dance_start_time = 0.0

# Mount static directories
app.mount("/static/3d", StaticFiles(directory=str(MODELS_3D_DIR)), name="3d_models")
app.mount("/media", StaticFiles(directory=str(UPLOADS_DIR)), name="media")
app.mount("/web", StaticFiles(directory=str(STATIC_DIR)), name="web_assets")

@app.get("/")
async def root():
    return FileResponse(STATIC_DIR / "index.html")

@app.get("/favicon.ico")
async def favicon():
    return FileResponse(STATIC_DIR / "favicon.svg", media_type="image/svg+xml")

# ==================== ESP32 & TELEMETRY APIS ====================

@app.get("/api/status")
async def get_status():
    await esp32_client.check_connection()
    status = esp32_client.get_status()
    status["music_playing"] = music_service.is_playing
    return status

class IPUpdate(BaseModel):
    ip: str

@app.post("/api/ip")
async def update_ip(data: IPUpdate):
    clean_ip = esp32_client.set_ip(data.ip)
    connected = await esp32_client.check_connection()
    await broadcast_telemetry()
    return {
        "success": True,
        "ip": esp32_client.ip,
        "port": esp32_client.port,
        "connected": connected,
        "simulated": esp32_client.is_simulated,
        "message": (
            f"Conexão com ESP32 estabelecida com sucesso em http://{esp32_client.ip}:{esp32_client.port}!"
            if connected else
            f"Não foi possível responder em http://{esp32_client.ip}:{esp32_client.port}. Modo Simulado mantido."
        )
    }

@app.get("/api/esp32/test")
async def test_esp32_connection():
    connected = await esp32_client.check_connection()
    await broadcast_telemetry()
    return {
        "success": True,
        "ip": esp32_client.ip,
        "port": esp32_client.port,
        "connected": connected,
        "simulated": esp32_client.is_simulated,
        "message": (
            f"ESP32 Online em http://{esp32_client.ip}:{esp32_client.port}!"
            if connected else
            f"ESP32 Inacessível em http://{esp32_client.ip}:{esp32_client.port}."
        )
    }

@app.post("/api/enable")
async def enable_system():
    return await esp32_client.enable_system()

@app.post("/api/disable")
async def disable_system():
    global dance_running
    dance_running = False
    music_service.is_playing = False
    return await esp32_client.disable_system()

@app.post("/api/stop")
async def stop_system():
    global dance_running
    dance_running = False
    music_service.is_playing = False
    return await esp32_client.stop_motion()

class MoveCommand(BaseModel):
    angles: dict
    speed: int = 50

@app.post("/api/move")
async def move_arm(cmd: MoveCommand):
    result = await esp32_client.send_move(cmd.angles, cmd.speed)
    await broadcast_telemetry()
    return result

class HomeCommand(BaseModel):
    angles: dict | None = None

@app.post("/api/home")
async def bring_home(cmd: HomeCommand | None = None):
    if cmd and cmd.angles:
        await esp32_client.set_home_config(cmd.angles)
    result = await esp32_client.send_home()
    await broadcast_telemetry()
    return result

class HomeConfigUpdate(BaseModel):
    config: dict

@app.get("/api/home/config")
async def get_home_config():
    return {
        "success": True,
        "home_config": esp32_client.home_config
    }

@app.post("/api/home/config")
async def update_home_config(data: HomeConfigUpdate):
    result = await esp32_client.set_home_config(data.config)
    await broadcast_telemetry()
    return result

class PinConfigUpdate(BaseModel):
    pins: dict

@app.get("/api/pins")
async def get_pins():
    return {
        "success": True,
        "pins": esp32_client.pins_mapping
    }

@app.post("/api/pins")
async def update_pins(data: PinConfigUpdate):
    result = await esp32_client.set_pins(data.pins)
    await broadcast_telemetry()
    return result

class LimitsUpdate(BaseModel):
    limits: dict

@app.get("/api/limits")
async def get_limits():
    return {
        "success": True,
        "limits": esp32_client.servo_limits,
        "effective_shoulder": esp32_client.get_effective_shoulder_limits()
    }

@app.post("/api/limits")
async def update_limits(data: LimitsUpdate):
    result = await esp32_client.set_limits(data.limits)
    await broadcast_telemetry()
    return result

# ==================== MUSIC & DANCING APIS ====================

@app.post("/api/music/upload")
async def upload_music(file: UploadFile = File(...)):
    filename = file.filename
    save_path = UPLOADS_DIR / filename
    with open(save_path, "wb") as f:
        content = await file.read()
        f.write(content)

    res = music_service.process_local_file(save_path)
    return res

class YouTubeRequest(BaseModel):
    url: str

@app.post("/api/music/youtube")
async def process_youtube(req: YouTubeRequest):
    res = music_service.download_youtube_audio(req.url.strip())
    return res

class PlayMusicRequest(BaseModel):
    filename: str
    attenuation_mode: str = "half_time"
    intensity: float = 0.75

@app.post("/api/music/play")
async def play_music(req: PlayMusicRequest, background_tasks: BackgroundTasks):
    global dance_running, dance_start_time
    music_service.set_attenuation(req.attenuation_mode, req.intensity)
    music_service.is_playing = True
    dance_running = True
    dance_start_time = time.time()
    background_tasks.add_task(run_dance_loop)
    return {
        "success": True,
        "status": "playing",
        "duration": music_service.current_song_duration,
        "bpm": music_service.detected_bpm,
        "attenuation_mode": music_service.attenuation_mode,
        "intensity": music_service.intensity
    }

class AttenuationRequest(BaseModel):
    mode: str = "half_time"
    intensity: float = 0.75

@app.post("/api/music/attenuation")
async def set_music_attenuation(req: AttenuationRequest):
    music_service.set_attenuation(req.mode, req.intensity)
    return {
        "success": True,
        "attenuation_mode": music_service.attenuation_mode,
        "intensity": music_service.intensity
    }

@app.post("/api/music/stop")
async def stop_music():
    global dance_running
    dance_running = False
    music_service.is_playing = False
    await esp32_client.send_home()
    await broadcast_telemetry()
    return {"success": True, "status": "stopped"}

async def run_dance_loop():
    """Background loop sending beat-synchronized dancing movements to servos."""
    global dance_running, dance_start_time
    while dance_running and music_service.is_playing:
        current_sec = time.time() - dance_start_time
        if current_sec > music_service.current_song_duration + 1.0:
            dance_running = False
            music_service.is_playing = False
            break

        pose = music_service.get_pose_at_time(current_sec)
        await esp32_client.send_move(pose, speed=pose.get("speed", 65))
        await broadcast_telemetry()
        await asyncio.sleep(0.045) # ~22 fps smooth rate

# ==================== IA TALK APIS ====================

@app.post("/api/ai-talk/turn")
async def ai_talk_turn(
    file: UploadFile = File(...),
    mic_source: str = Form("browser_mic"),
    tts_voice: str = Form("pt-BR-AntonioNeural")
):
    """
    Receives voice audio, performs DOA sound localization if using arm mics,
    transcribes with Groq Whisper, queries Groq chat, and synthesizes native Brazilian Portuguese speech.
    """
    audio_bytes = await file.read()
    is_arm_mic = (mic_source == "arm_mic")
    res = await ai_service.process_turn(audio_bytes, is_arm_mic=is_arm_mic, tts_voice=tts_voice)

    # If sound direction detected from Arm mics, rotate base towards the user!
    doa = res.get("doa", {})
    if is_arm_mic and doa.get("confidence", 0) > 0.35 and abs(doa.get("angle_degrees", 0)) > 8:
        current_base = esp32_client.current_angles.get("base_rotacao", 90)
        # Offset base angle in direction of sound arrival
        target_base = int(max(15, min(165, current_base + doa["angle_degrees"])))
        print(f"[DOA] Turning base from {current_base}° to {target_base}° (sound from {doa['side']} at {doa['angle_degrees']}°)")
        await esp32_client.send_move({"base_rotacao": target_base}, speed=60)
        await broadcast_telemetry()

    # If successful, stream PCM to ESP32 speaker if hardware is connected
    if res.get("success") and res.get("pcm_bytes"):
        pcm_data = res["pcm_bytes"]
        chunk_size = 4096
        for i in range(0, len(pcm_data), chunk_size):
            chunk = pcm_data[i:i+chunk_size]
            await esp32_client.stream_audio_chunk(chunk)
            await asyncio.sleep(0.03)

    return {
        "success": res.get("success", False),
        "user_text": res.get("user_text", ""),
        "ai_text": res.get("ai_text", ""),
        "audio_base64": res.get("audio_base64", ""),
        "doa": doa,
        "error": res.get("error", "")
    }

@app.post("/api/ai-talk/reset")
async def ai_talk_reset():
    ai_service.reset_conversation()
    return {"success": True, "message": "Histórico de conversa reiniciado."}

# ==================== WEBSOCKET TELEMETRY ====================

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    try:
        # Send initial status immediately
        status = esp32_client.get_status()
        status["music_playing"] = bool(dance_running)
        await websocket.send_text(json.dumps(status))
        while True:
            # Handle commands coming from client via WebSocket for ultra-low latency
            data = await websocket.receive_text()
            msg = json.loads(data)
            action = msg.get("action")
            if action == "move":
                angles = msg.get("angles", {})
                speed = msg.get("speed", 60)
                await esp32_client.send_move(angles, speed)
                await broadcast_telemetry()
            elif action == "home":
                await esp32_client.send_home()
                await broadcast_telemetry()
    except WebSocketDisconnect:
        connected_clients.discard(websocket)
    except Exception as e:
        print(f"[WS Exception]: {type(e).__name__}: {e}")
        connected_clients.discard(websocket)

async def broadcast_telemetry():
    if not connected_clients:
        return
    status = esp32_client.get_status()
    status["music_playing"] = bool(dance_running)
    msg = json.dumps(status)
    dead_clients = []
    for ws in list(connected_clients):
        try:
            await ws.send_text(msg)
        except Exception as e:
            print(f"[Broadcast Error]: {type(e).__name__}: {e}")
            dead_clients.append(ws)
    for ws in dead_clients:
        connected_clients.discard(ws)

# ==================== REAL-TIME TELEMETRY ENGINE ====================

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(telemetry_engine_loop())
    asyncio.create_task(connection_monitor_loop())

async def connection_monitor_loop():
    """Monitors ESP32 connection status periodically."""
    while True:
        try:
            prev_connected = esp32_client.is_connected
            await esp32_client.check_connection()
            if prev_connected != esp32_client.is_connected:
                await broadcast_telemetry()
        except Exception:
            pass
        await asyncio.sleep(2.5)

async def telemetry_engine_loop():
    """
    High-frequency (30 Hz / 33ms) real-time streaming telemetry loop.
    - If hardware is connected: polls /telemetry live from ESP32.
    - If in simulator: steps physical servo kinematics smoothly.
    - Broadcasts live telemetry to all connected 3D visualizers.
    """
    last_broadcast_time = 0.0
    while True:
        try:
            loop_start = time.time()
            dt = 0.033

            if esp32_client.is_connected:
                await esp32_client.poll_hardware_telemetry()
            else:
                esp32_client.step_simulation(dt=dt)

            now = time.time()
            # Broadcast at 30 FPS if moving or clients connected, or at least every 0.8s heartbeat
            should_broadcast = (
                connected_clients and (
                    esp32_client.is_moving or
                    (now - last_broadcast_time >= 0.8)
                )
            )
            if should_broadcast:
                await broadcast_telemetry()
                last_broadcast_time = now

            elapsed = time.time() - loop_start
            sleep_time = max(0.005, 0.033 - elapsed)
            await asyncio.sleep(sleep_time)
        except Exception:
            await asyncio.sleep(0.05)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("server.app:app", host=SERVER_HOST, port=SERVER_PORT, reload=False)
