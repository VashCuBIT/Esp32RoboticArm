/**
 * music_dance.js - Player de Música, Visualizador de Áudio e Dança Sincronizada com Batidas
 */

class MusicDanceController {
  constructor(armVisualizer) {
    this.arm = armVisualizer;
    this.currentTrack = null;
    this.isPlaying = false;
    this.audioElement = new Audio();
    this.audioContext = null;
    this.analyser = null;
    this.dataArray = null;

    this.initUI();
    this.initAudioVisualizer();
  }

  initUI() {
    this.fileInput = document.getElementById("music-file-input");
    this.dropzone = document.getElementById("music-dropzone");
    this.ytInput = document.getElementById("yt-url-input");
    this.btnYtDownload = document.getElementById("btn-yt-download");
    this.btnPlay = document.getElementById("btn-music-play");
    this.btnStop = document.getElementById("btn-music-stop");
    this.trackTitle = document.getElementById("track-title");
    this.canvas = document.getElementById("music-visualizer");
    this.statusText = document.getElementById("music-status-text");

    if (this.dropzone && this.fileInput) {
      this.dropzone.addEventListener("click", () => this.fileInput.click());
      this.fileInput.addEventListener("change", (e) => {
        if (e.target.files.length > 0) {
          this.uploadFile(e.target.files[0]);
        }
      });
    }

    if (this.btnYtDownload) {
      this.btnYtDownload.addEventListener("click", () => this.downloadYouTube());
    }

    if (this.btnPlay) {
      this.btnPlay.addEventListener("click", () => this.togglePlay());
    }

    if (this.btnStop) {
      this.btnStop.addEventListener("click", () => this.stop());
    }

    this.audioElement.addEventListener("ended", () => this.stop());

    this.cadenceSelect = document.getElementById("music-cadence-select");
    this.cadenceBadge = document.getElementById("music-cadence-badge");
    this.intensitySlider = document.getElementById("music-intensity-slider");
    this.intensityVal = document.getElementById("music-intensity-val");

    // Restaurar preferências salvas no navegador
    const savedCadence = localStorage.getItem("arm_dance_cadence") || "half_time";
    const savedIntensity = localStorage.getItem("arm_dance_intensity") || "75";
    if (this.cadenceSelect) this.cadenceSelect.value = savedCadence;
    if (this.intensitySlider) this.intensitySlider.value = savedIntensity;
    if (this.intensityVal) this.intensityVal.textContent = `${savedIntensity}%`;
    this.updateCadenceBadge(savedCadence);

    if (this.cadenceSelect) {
      this.cadenceSelect.addEventListener("change", (e) => {
        const mode = e.target.value;
        localStorage.setItem("arm_dance_cadence", mode);
        this.updateCadenceBadge(mode);
        this.syncAttenuation();
      });
    }

    if (this.intensitySlider) {
      this.intensitySlider.addEventListener("input", (e) => {
        const val = e.target.value;
        if (this.intensityVal) this.intensityVal.textContent = `${val}%`;
        localStorage.setItem("arm_dance_intensity", val);
      });
      this.intensitySlider.addEventListener("change", () => {
        this.syncAttenuation();
      });
    }
  }

  updateCadenceBadge(mode) {
    if (!this.cadenceBadge) return;
    const labels = {
      half_time: "1 Sim, 1 Não",
      auto: "Automático",
      quarter_time: "1 a cada 4",
      all_beats: "Todas (1:1)"
    };
    this.cadenceBadge.textContent = labels[mode] || mode;
  }

  async syncAttenuation() {
    const mode = this.cadenceSelect ? this.cadenceSelect.value : "half_time";
    const intensity = this.intensitySlider ? parseInt(this.intensitySlider.value, 10) / 100.0 : 0.75;
    try {
      await fetch("/api/music/attenuation", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode, intensity })
      });
    } catch (e) {
      console.warn("Erro ao sincronizar atenuação com o servidor:", e);
    }
  }

  initAudioVisualizer() {
    if (!this.canvas) return;
    this.ctx = this.canvas.getContext("2d");
  }

  setupWebAudio() {
    if (this.audioContext) return;
    try {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      this.audioContext = new AudioCtx();
      this.analyser = this.audioContext.createAnalyser();
      this.analyser.fftSize = 64;

      const source = this.audioContext.createMediaElementSource(this.audioElement);
      source.connect(this.analyser);
      this.analyser.connect(this.audioContext.destination);

      const bufferLength = this.analyser.frequencyBinCount;
      this.dataArray = new Uint8Array(bufferLength);
      this.drawVisualizer();
    } catch (e) {
      console.warn("Web Audio API not supported or blocked:", e);
    }
  }

  drawVisualizer() {
    requestAnimationFrame(() => this.drawVisualizer());
    if (!this.ctx || !this.analyser || !this.dataArray) return;

    this.analyser.getByteFrequencyData(this.dataArray);
    const width = this.canvas.width;
    const height = this.canvas.height;

    this.ctx.clearRect(0, 0, width, height);

    const barWidth = (width / this.dataArray.length) * 1.5;
    let x = 0;

    for (let i = 0; i < this.dataArray.length; i++) {
      const barHeight = (this.dataArray[i] / 255) * height;
      const gradient = this.ctx.createLinearGradient(0, height, 0, 0);
      gradient.addColorStop(0, "#38bdf8");
      gradient.addColorStop(1, "#818cf8");

      this.ctx.fillStyle = gradient;
      this.ctx.fillRect(x, height - barHeight, barWidth - 2, barHeight);
      x += barWidth;
    }
  }

  async uploadFile(file) {
    this.setStatus("Enviando e analisando batidas...");
    const formData = new FormData();
    formData.append("file", file);

    try {
      const res = await fetch("/api/music/upload", {
        method: "POST",
        body: formData
      });
      const data = await res.json();
      if (data.success) {
        this.currentTrack = data;
        this.trackTitle.textContent = data.title;
        this.audioElement.src = data.mp3_url;
        const bpmInfo = data.bpm ? ` (~${Math.round(data.bpm)} BPM)` : "";
        this.setStatus(`Pronto! ${data.beats_count} batidas${bpmInfo} detectadas.`);
        if (window.showToast) window.showToast(`Música carregada e analisada!${bpmInfo}`);
      } else {
        this.setStatus(`Erro: ${data.error}`);
      }
    } catch (e) {
      this.setStatus("Erro na conexão durante upload.");
    }
  }

  async downloadYouTube() {
    const url = this.ytInput.value.trim();
    if (!url) return;

    this.setStatus("Baixando do YouTube e gerando coreografia...");
    this.btnYtDownload.disabled = true;

    try {
      const res = await fetch("/api/music/youtube", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ url })
      });
      const data = await res.json();
      if (data.success) {
        this.currentTrack = data;
        this.trackTitle.textContent = data.title;
        this.audioElement.src = data.mp3_url;
        const bpmInfo = data.bpm ? ` (~${Math.round(data.bpm)} BPM)` : "";
        this.setStatus(`Pronto! ${data.beats_count} batidas${bpmInfo} detectadas.`);
        if (window.showToast) window.showToast(`Música do YouTube pronta!${bpmInfo}`);
      } else {
        this.setStatus(`Erro: ${data.error}`);
      }
    } catch (e) {
      this.setStatus("Erro ao baixar do YouTube.");
    } finally {
      this.btnYtDownload.disabled = false;
    }
  }

  async togglePlay() {
    if (!this.currentTrack) {
      if (window.showToast) window.showToast("Carregue uma música primeiro!");
      return;
    }

    if (this.isPlaying) {
      this.pause();
    } else {
      this.play();
    }
  }

  async play() {
    this.setupWebAudio();
    if (this.audioContext && this.audioContext.state === "suspended") {
      await this.audioContext.resume();
    }

    try {
      await this.audioElement.play();
      this.isPlaying = true;
      this.btnPlay.textContent = "Pausar";
      this.setStatus("Dançando no ritmo da música...");

      const mode = this.cadenceSelect ? this.cadenceSelect.value : "half_time";
      const intensity = this.intensitySlider ? parseInt(this.intensitySlider.value, 10) / 100.0 : 0.75;

      await fetch("/api/music/play", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          filename: this.currentTrack.mp3_filename,
          attenuation_mode: mode,
          intensity: intensity
        })
      });
    } catch (e) {
      console.error("Erro ao reproduzir:", e);
    }
  }

  pause() {
    this.audioElement.pause();
    this.isPlaying = false;
    if (this.btnPlay) this.btnPlay.textContent = "Reproduzir & Dançar";
    this.setStatus("Pausado.");
    fetch("/api/music/stop", { method: "POST" });
  }

  stop() {
    this.audioElement.pause();
    this.audioElement.currentTime = 0;
    this.isPlaying = false;
    if (this.btnPlay) this.btnPlay.textContent = "Reproduzir & Dançar";
    this.setStatus("Parado.");
    fetch("/api/music/stop", { method: "POST" });
  }

  setStatus(msg) {
    if (this.statusText) {
      this.statusText.textContent = msg;
    }
  }
}

window.MusicDanceController = MusicDanceController;
