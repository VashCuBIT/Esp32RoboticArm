import os
import sys
import re
import math
import time
import asyncio
import subprocess
from pathlib import Path
import numpy as np
from scipy.io import wavfile
from scipy.signal import find_peaks
from server.config import (
    UPLOADS_DIR, get_ffmpeg_executable, get_ffmpeg_dir, get_node_executable
)

class MusicService:
    def __init__(self):
        self.current_song = None
        self.is_playing = False
        self.dance_task = None
        self.subscribers = set()
        self.playback_start_time = 0.0
        self.current_choreography = []
        self.current_song_duration = 0.0
        self.detected_bpm = 120.0
        self.attenuation_mode = "half_time"  # "half_time", "auto", "quarter_time", "all_beats"
        self.intensity = 0.75  # 0.3 a 1.2
        self.last_pose = None

    def set_attenuation(self, mode: str, intensity: float | None = None):
        valid_modes = {"half_time", "auto", "quarter_time", "all_beats"}
        if mode in valid_modes:
            self.attenuation_mode = mode
        if intensity is not None:
            self.intensity = max(0.3, min(1.2, float(intensity)))
        print(f"[MusicService] Atenuação atualizada: modo={self.attenuation_mode}, intensidade={self.intensity:.2f}")

    def download_youtube_audio(self, youtube_url: str) -> dict:
        """Downloads YouTube audio using yt-dlp and converts to MP3 & 16kHz WAV."""
        try:
            output_template = str(UPLOADS_DIR / "yt_%(id)s.%(ext)s")
            ffmpeg_exe = get_ffmpeg_executable()
            ffmpeg_dir = get_ffmpeg_dir()
            node_exe = get_node_executable()

            # Always invoke yt-dlp via sys.executable to prevent Windows WinError 2
            cmd = [
                sys.executable,
                "-m",
                "yt_dlp",
                "-x",
                "--audio-format", "mp3",
                "--audio-quality", "0",
                "--no-playlist",
                "-o", output_template
            ]
            if ffmpeg_dir:
                cmd.extend(["--ffmpeg-location", ffmpeg_dir])
            if node_exe:
                cmd.extend(["--js-runtimes", f"node:{node_exe}"])
            cmd.append(youtube_url)

            subprocess.run(cmd, check=True, capture_output=True, timeout=120)

            # Find the downloaded file
            mp3_files = list(UPLOADS_DIR.glob("yt_*.mp3"))
            if not mp3_files:
                return {"success": False, "error": "Arquivo não encontrado após download."}

            latest_mp3 = max(mp3_files, key=os.path.getctime)
            wav_path = latest_mp3.with_suffix(".wav")
            pcm_path = latest_mp3.with_suffix(".pcm")

            # Convert to 16kHz WAV for analysis
            subprocess.run([
                ffmpeg_exe, "-y", "-i", str(latest_mp3),
                "-ar", "16000", "-ac", "1",
                str(wav_path)
            ], check=True, capture_output=True, timeout=30)

            # Convert to 16kHz raw PCM for ESP32 I2S
            subprocess.run([
                ffmpeg_exe, "-y", "-i", str(latest_mp3),
                "-ar", "16000", "-ac", "1", "-f", "s16le",
                str(pcm_path)
            ], check=True, capture_output=True, timeout=30)

            choreo, duration = self.analyze_beats_and_choreograph(str(wav_path))

            return {
                "success": True,
                "title": latest_mp3.stem,
                "mp3_filename": latest_mp3.name,
                "mp3_url": f"/media/{latest_mp3.name}",
                "duration": duration,
                "beats_count": len(choreo),
                "bpm": self.detected_bpm,
                "attenuation_mode": self.attenuation_mode,
                "intensity": self.intensity
            }
        except subprocess.CalledProcessError as e:
            err_msg = e.stderr.decode("utf-8", errors="ignore").strip() if e.stderr else str(e)
            err_lines = [l for l in err_msg.splitlines() if "ERROR:" in l or "error" in l.lower()]
            clean_err = err_lines[-1] if err_lines else (err_msg[:300] or str(e))
            print(f"[MusicService] YouTube process error: {clean_err}")
            return {"success": False, "error": clean_err}
        except Exception as e:
            print(f"[MusicService] YouTube download error: {e}")
            return {"success": False, "error": str(e)}

    def process_local_file(self, file_path: Path) -> dict:
        """Converts uploaded file to MP3, 16kHz WAV and raw PCM, then extracts choreography."""
        try:
            ffmpeg_exe = get_ffmpeg_executable()
            mp3_path = file_path.with_suffix(".mp3")
            wav_path = file_path.with_suffix(".wav")
            pcm_path = file_path.with_suffix(".pcm")

            # Ensure MP3 exists
            if file_path.suffix.lower() != ".mp3":
                subprocess.run([
                    ffmpeg_exe, "-y", "-i", str(file_path),
                    "-b:a", "192k",
                    str(mp3_path)
                ], check=True, capture_output=True, timeout=30)

            # Convert to 16kHz WAV
            subprocess.run([
                ffmpeg_exe, "-y", "-i", str(mp3_path),
                "-ar", "16000", "-ac", "1",
                str(wav_path)
            ], check=True, capture_output=True, timeout=30)

            # Convert to raw PCM
            subprocess.run([
                ffmpeg_exe, "-y", "-i", str(mp3_path),
                "-ar", "16000", "-ac", "1", "-f", "s16le",
                str(pcm_path)
            ], check=True, capture_output=True, timeout=30)

            choreo, duration = self.analyze_beats_and_choreograph(str(wav_path))

            return {
                "success": True,
                "title": mp3_path.stem,
                "mp3_filename": mp3_path.name,
                "mp3_url": f"/media/{mp3_path.name}",
                "duration": duration,
                "beats_count": len(choreo),
                "bpm": self.detected_bpm,
                "attenuation_mode": self.attenuation_mode,
                "intensity": self.intensity
            }
        except Exception as e:
            print(f"[MusicService] Process local file error: {e}")
            return {"success": False, "error": str(e)}

    def analyze_beats_and_choreograph(self, wav_path: str):
        """Analyzes audio energy and onsets to generate smooth beat-synchronized dance motions."""
        try:
            sr, data = wavfile.read(wav_path)
            if data.ndim > 1:
                data = data[:, 0]
            data = data.astype(np.float32)

            duration = len(data) / float(sr)
            self.current_song_duration = duration

            # Compute short-time energy envelope (hop size ~ 20ms)
            hop_size = int(sr * 0.02)
            window_size = int(sr * 0.04)
            num_frames = int(math.floor((len(data) - window_size) / hop_size))

            if num_frames <= 0:
                return [], duration

            energy = np.zeros(num_frames)
            for i in range(num_frames):
                segment = data[i * hop_size : i * hop_size + window_size]
                energy[i] = np.sqrt(np.mean(segment ** 2) + 1e-8)

            # High-pass filter of energy for onset novelty
            energy_diff = np.diff(energy, prepend=energy[0])
            energy_diff = np.maximum(0, energy_diff)

            # Peak detection for beats
            peaks, properties = find_peaks(
                energy_diff,
                height=np.mean(energy_diff) * 1.3,
                distance=int(0.25 / 0.02) # min 250ms between beats (~240 BPM max)
            )

            beat_times = peaks * 0.02

            # Estima o andamento (BPM) da faixa
            if len(beat_times) >= 2:
                intervals = np.diff(beat_times)
                valid_intervals = intervals[(intervals >= 0.2) & (intervals <= 2.0)]
                if len(valid_intervals) > 0:
                    med_interval = float(np.median(valid_intervals))
                    self.detected_bpm = round(60.0 / med_interval, 1)
                else:
                    self.detected_bpm = 120.0
            else:
                self.detected_bpm = 120.0

            print(f"[MusicService] Batidas detectadas: {len(beat_times)} | BPM estimado: {self.detected_bpm}")

            # Generate choreographic keyframes
            keyframes = []
            dance_modes = ["groove", "sway", "headbang", "claw_snap"]

            for idx, t in enumerate(beat_times):
                mode = dance_modes[idx % len(dance_modes)]
                phase = (idx % 8) / 8.0 * (2 * math.pi)

                # Rhythmic base sway
                base_angle = int(90 + 35 * math.sin(phase))
                
                # Shoulder & elbow rhythmic nodding
                if mode == "headbang" or idx % 2 == 0:
                    ombro_angle = int(90 - 25)
                    cotovelo_angle = int(90 + 30)
                else:
                    ombro_angle = int(90 + 15)
                    cotovelo_angle = int(90 - 15)

                # Wrist pitch & roll
                punho_angle = int(90 + 20 * math.cos(phase))
                garra_rot_angle = int(90 + 40 * math.sin(phase * 1.5))

                # Claw snapping to beats
                garra_abertura = 125 if (idx % 2 == 0) else 65

                keyframes.append({
                    "time": round(float(t), 2),
                    "base_rotacao": int(np.clip(base_angle, 25, 155)),
                    "ombro": int(np.clip(ombro_angle, 40, 140)),
                    "cotovelo": int(np.clip(cotovelo_angle, 40, 140)),
                    "punho": int(np.clip(punho_angle, 30, 150)),
                    "garra_rotacao": int(np.clip(garra_rot_angle, 20, 160)),
                    "garra_abertura": int(np.clip(garra_abertura, 50, 130)),
                    "speed": 85
                })

            self.current_choreography = keyframes
            self.last_pose = None
            return keyframes, duration
        except Exception as e:
            print(f"[MusicService] Beat detection error: {e}")
            return [], 0.0

    def get_pose_at_time(self, current_sec: float) -> dict:
        """
        Gera poses de dança orgânicas e fluidas sincronizadas com o ritmo musical.
        Aplica atenuação adaptativa na frequência das batidas (Half-time / 1 sim 1 não)
        para garantir que o braço nunca sofra solavancos violentos em músicas rápidas,
        mantendo a coerência rítmica absoluta com os tempos musicais.
        """
        beat_phase = 0.0
        if not self.current_choreography or len(self.current_choreography) < 2:
            beat_phase = current_sec * 2.0  # Fallback 120 BPM
        else:
            choreo = self.current_choreography
            if current_sec <= choreo[0]["time"]:
                beat_phase = 0.0
            elif current_sec >= choreo[-1]["time"]:
                beat_phase = float(len(choreo) - 1)
            else:
                for i in range(len(choreo) - 1):
                    if choreo[i]["time"] <= current_sec <= choreo[i + 1]["time"]:
                        dt = choreo[i + 1]["time"] - choreo[i]["time"]
                        alpha = (current_sec - choreo[i]["time"]) / dt if dt > 0 else 0.0
                        beat_phase = float(i) + alpha
                        break

        # Cadência / Atenuador de frequência das batidas:
        # - "half_time" (padrão): 1.0 -> Bate no tempo 0, recupera suave no tempo 1, bate no tempo 2 (1 sim, 1 não)
        # - "quarter_time": 0.5 -> Bate a cada 4 tempos (compasso quaternário suave)
        # - "all_beats": 2.0 -> Bate em todas as batidas detectadas
        # - "auto": Adaptativo inteligente pelo BPM detectado
        cadence_factor = 1.0
        if self.attenuation_mode == "all_beats":
            cadence_factor = 2.0
        elif self.attenuation_mode == "quarter_time":
            cadence_factor = 0.5
        elif self.attenuation_mode == "auto":
            if self.detected_bpm > 140.0:
                cadence_factor = 0.5
            elif self.detected_bpm > 95.0:
                cadence_factor = 1.0
            else:
                cadence_factor = 2.0
        else:  # "half_time" (padrão)
            cadence_factor = 1.0

        # Oscilação suave contínua com pico exato no instante da batida selecionada
        on_beat_bounce = (math.cos(beat_phase * cadence_factor * math.pi) + 1.0) / 2.0

        # Fator de intensidade e amplitude (padrão 0.75 para movimentos seguros e não-agressivos)
        intens = max(0.3, min(1.2, float(self.intensity)))

        # 1. Base (balanço horizontal rítmico elegante):
        base = 90.0 + (28.0 * intens) * math.sin(beat_phase * math.pi / 4.0)

        # 2. Ombro e Cotovelo (aceno vertical ritmado no tempo selecionado):
        # Em half-time ("1 sim, 1 não"), ombro e cotovelo descem suavemente na batida
        # e sobem no tempo seguinte, evitando oscilações frenéticas
        ombro = 90.0 - (20.0 * intens) * on_beat_bounce
        cotovelo = 90.0 + (28.0 * intens) * on_beat_bounce

        # 3. Punho (inclinação ondulada fluida):
        punho = 90.0 + (22.0 * intens) * math.sin(beat_phase * math.pi * 0.5)

        # 4. Garra Rotação (roll suave sincronizado):
        garra_rot = 90.0 + (35.0 * intens) * math.sin(beat_phase * math.pi * 0.25)

        # 5. Garra Abertura (abre no contratempo e fecha no tempo forte):
        garra_ab = 60.0 + (60.0 * intens) * (1.0 - on_beat_bounce)

        raw_pose = {
            "base_rotacao": int(np.clip(base, 25, 155)),
            "ombro": int(np.clip(ombro, 40, 140)),
            "cotovelo": int(np.clip(cotovelo, 40, 140)),
            "punho": int(np.clip(punho, 30, 150)),
            "garra_rotacao": int(np.clip(garra_rot, 20, 160)),
            "garra_abertura": int(np.clip(garra_ab, 45, 135)),
            "speed": 65 if cadence_factor <= 1.0 else 85
        }

        # Filtro de suavização exponencial para eliminar degraus bruscos entre quadros
        if self.last_pose is None:
            self.last_pose = raw_pose
            return raw_pose

        smooth_alpha = 0.85
        final_pose = {}
        for k in ["base_rotacao", "ombro", "cotovelo", "punho", "garra_rotacao", "garra_abertura"]:
            smoothed_val = smooth_alpha * raw_pose[k] + (1.0 - smooth_alpha) * self.last_pose[k]
            final_pose[k] = int(round(smoothed_val))
        final_pose["speed"] = raw_pose["speed"]

        self.last_pose = final_pose
        return final_pose

music_service = MusicService()
