// ── Socket Connection ───────────────────────────────────────────────────
const socket = io();

// ── DOM Elements ────────────────────────────────────────────────────────
const mapCanvas = document.getElementById('mapCanvas');
const mapStatusBadge = document.getElementById('map-status');
const areaVal = document.getElementById('area-val');
const exploreStatusVal = document.getElementById('explore-status-val');
const diagJson = document.getElementById('diag-json');
const logContainer = document.getElementById('log-container');

// Buttons
const btnSetup = document.getElementById('btn-setup');
const btnWait = document.getElementById('btn-wait');
const btnStop = document.getElementById('btn-stop');
const btnClearLog = document.getElementById('btn-clear-log');
const minAreaInput = document.getElementById('min-area-input');

// Progress bars
const setupProgressSection = document.getElementById('setup-progress-section');
const setupProgressBar = document.getElementById('setup-progress-bar');
const setupStageText = document.getElementById('setup-stage-text');
const setupPctText = document.getElementById('setup-pct-text');

const mapProgressSection = document.getElementById('map-progress-section');
const mapProgressBar = document.getElementById('map-progress-bar');
const mapStageText = document.getElementById('map-stage-text');
const mapPctText = document.getElementById('map-pct-text');

// ── State ───────────────────────────────────────────────────────────────
let robotPose = { x: 0, y: 0, yaw: 0 };
let currentMap = null;
let isSettingUp = false;
let isWaiting = false;
let cachedMapImage = null;

// ── Dual-Canvas Setup (Bug 6 fix) ──────────────────────────────────────
// Layer 1: mapCanvas — redrawn only on map_update events
// Layer 2: poseCanvas — overlay, redrawn only on robot_pose events
const mapCtx = mapCanvas.getContext('2d');

const poseCanvas = document.createElement('canvas');
poseCanvas.id = 'poseCanvas';
poseCanvas.width = mapCanvas.width;
poseCanvas.height = mapCanvas.height;
poseCanvas.style.position = 'absolute';
poseCanvas.style.top = '0';
poseCanvas.style.left = '0';
poseCanvas.style.pointerEvents = 'none';
mapCanvas.parentElement.style.position = 'relative';
mapCanvas.parentElement.appendChild(poseCanvas);
const poseCtx = poseCanvas.getContext('2d');

// ── Utility ─────────────────────────────────────────────────────────────

function addLog(message, type = 'info') {
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    const ts = new Date().toLocaleTimeString();
    entry.textContent = `[${ts}] ${message}`;
    logContainer.appendChild(entry);
    logContainer.scrollTop = logContainer.scrollHeight;
    // Keep last 100 entries
    while (logContainer.children.length > 100) {
        logContainer.removeChild(logContainer.firstChild);
    }
}

function updateButtons(exploreActive) {
    if (isSettingUp) {
        btnSetup.disabled = true;
        btnWait.disabled = true;
        btnStop.disabled = true;
    } else if (isWaiting) {
        btnSetup.disabled = true;
        btnWait.disabled = true;
        btnStop.disabled = false;
    } else if (exploreActive) {
        btnSetup.disabled = true;
        btnSetup.innerHTML = '<span class="icon">🚀</span> Start Autonomous Mapping';
        btnWait.disabled = false;
        btnStop.disabled = false;
    } else {
        btnSetup.disabled = false;
        btnSetup.innerHTML = '<span class="icon">🚀</span> Start Autonomous Mapping';
        btnWait.disabled = true;
        btnStop.disabled = true;
    }
}

function resizeCanvases() {
    poseCanvas.width = mapCanvas.width;
    poseCanvas.height = mapCanvas.height;
}

// ── SocketIO: Exploration Status ────────────────────────────────────────

socket.on('connect', () => {
    addLog('Connected to backend', 'success');
});

// Bug 8 fix: reset flags and progress sections on disconnect
socket.on('disconnect', () => {
    addLog('Disconnected from backend', 'error');
    isSettingUp = false;
    isWaiting = false;
    setupProgressSection.style.display = 'none';
    mapProgressSection.style.display = 'none';
    updateButtons(false);
});

socket.on('explore_status', (diag) => {
    diagJson.textContent = JSON.stringify(diag, null, 2);
    // Bug 7 fix: nullish coalescing guard for undefined latest_area_m2
    areaVal.textContent = `${(diag.latest_area_m2 ?? 0).toFixed(2)} m²`;

    if (diag.frontiers_exhausted) {
        exploreStatusVal.textContent = 'Complete ✓';
        exploreStatusVal.style.color = '#34d399';
    } else if (diag.explore_active) {
        exploreStatusVal.textContent = 'Exploring...';
        exploreStatusVal.style.color = '#fbbf24';
    } else {
        exploreStatusVal.textContent = 'Idle';
        exploreStatusVal.style.color = '';
    }

    // Only update buttons from status if no operation is in flight
    if (!isSettingUp && !isWaiting) {
        updateButtons(diag.explore_active);
    }
});

// ── SocketIO: Setup Progress ────────────────────────────────────────────

socket.on('setup_progress', (data) => {
    setupProgressSection.style.display = 'block';
    setupProgressBar.style.width = `${data.percent}%`;
    setupStageText.textContent = data.message;
    setupPctText.textContent = `${data.percent}%`;
    addLog(data.message, 'progress');
});

socket.on('setup_result', (data) => {
    isSettingUp = false;
    if (data.success) {
        setupProgressBar.style.width = '100%';
        setupPctText.textContent = '100%';
        setupStageText.textContent = 'Setup complete!';
        addLog(data.message, 'success');
        updateButtons(true);
    } else {
        addLog(`Setup failed: ${data.message}`, 'error');
        updateButtons(false);
    }
    // Hide progress bar after a delay
    setTimeout(() => {
        setupProgressSection.style.display = 'none';
        setupProgressBar.style.width = '0%';
    }, 3000);
});

// ── SocketIO: Map Wait Progress ─────────────────────────────────────────

socket.on('map_progress', (data) => {
    mapProgressSection.style.display = 'block';
    mapProgressBar.style.width = `${data.percent}%`;
    mapStageText.textContent = `${data.current_area} / ${data.target_area} m²  —  ${data.elapsed}s`;
    mapPctText.textContent = `${data.percent}%`;
});

socket.on('map_result', (data) => {
    isWaiting = false;
    if (data.success) {
        mapProgressBar.style.width = '100%';
        mapPctText.textContent = '100%';
        mapStageText.textContent = 'Map complete!';
        addLog(data.message, 'success');
    } else {
        addLog(`Map wait ended: ${data.message}`, 'error');
    }
    updateButtons(false);
    setTimeout(() => {
        mapProgressSection.style.display = 'none';
        mapProgressBar.style.width = '0%';
    }, 5000);
});

// ── SocketIO: Map & Pose ────────────────────────────────────────────────

// Bug 6 fix: map_update only redraws the map layer, never the pose
socket.on('map_update', (msg) => {
    mapStatusBadge.textContent = 'Live';
    mapStatusBadge.className = 'badge success';
    currentMap = msg;
    drawMapLayer();
});

// Bug 6 fix: robot_pose only redraws the lightweight pose overlay
socket.on('robot_pose', (msg) => {
    robotPose = msg;
    if (currentMap) drawPoseLayer();
});

// ── Button Handlers ─────────────────────────────────────────────────────

btnSetup.addEventListener('click', async () => {
    isSettingUp = true;
    btnSetup.disabled = true;
    btnSetup.innerHTML = '<span class="icon">⏳</span> Setting up...';
    setupProgressSection.style.display = 'block';
    setupProgressBar.style.width = '0%';
    setupStageText.textContent = 'Initializing...';
    setupPctText.textContent = '0%';
    addLog('Starting autonomous exploration setup...', 'info');

    try {
        const res = await fetch('/api/explore/setup', { method: 'POST' });
        const data = await res.json();
        if (!data.success && res.status !== 202) {
            addLog(`Setup request failed: ${data.message}`, 'error');
            isSettingUp = false;
            updateButtons(false);
        }
    } catch (e) {
        addLog(`Request error: ${e}`, 'error');
        isSettingUp = false;
        updateButtons(false);
    }
});

btnWait.addEventListener('click', async () => {
    const minArea = parseFloat(minAreaInput.value) || 10;
    isWaiting = true;
    updateButtons(true);
    mapProgressSection.style.display = 'block';
    mapProgressBar.style.width = '0%';
    mapStageText.textContent = `Waiting for ${minArea} m²...`;
    mapPctText.textContent = '0%';
    addLog(`Waiting for map to reach ${minArea} m²...`, 'info');

    try {
        const res = await fetch('/api/explore/wait', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ min_area: minArea, timeout: 300 }),
        });
        const data = await res.json();
        if (!data.success && res.status !== 202) {
            addLog(`Wait request failed: ${data.message}`, 'error');
            isWaiting = false;
            updateButtons(true);
        }
    } catch (e) {
        addLog(`Request error: ${e}`, 'error');
        isWaiting = false;
        updateButtons(true);
    }
});

btnStop.addEventListener('click', async () => {
    addLog('Stopping exploration...', 'info');
    try {
        const res = await fetch('/api/explore/stop', { method: 'POST' });
        const data = await res.json();
        addLog(data.message, data.success ? 'success' : 'error');
    } catch (e) {
        addLog(`Stop error: ${e}`, 'error');
    }
    isSettingUp = false;
    isWaiting = false;
    setupProgressSection.style.display = 'none';
    mapProgressSection.style.display = 'none';
    updateButtons(false);
});

btnClearLog.addEventListener('click', () => {
    logContainer.innerHTML = '';
    addLog('Log cleared.', 'info');
});

// ── Map Rendering (Bug 6 fix: split into two layers) ────────────────────

/**
 * drawMapLayer — renders the occupancy grid onto mapCanvas.
 * Called ONLY from socket.on('map_update').
 */
function drawMapLayer() {
    if (!currentMap) return;

    const { width, height, data } = currentMap;

    if (mapCanvas.width !== width || mapCanvas.height !== height) {
        mapCanvas.width = width;
        mapCanvas.height = height;
        resizeCanvases();
    }

    const imgData = mapCtx.createImageData(width, height);
    for (let i = 0; i < data.length; i++) {
        let val = data[i];
        let color;
        // Bug 9 fix: val > 0 captures all occupied probabilities (1–100)
        if (val === 0) {
            color = [255, 255, 255, 255];        // free — white
        } else if (val > 0) {
            color = [15, 23, 42, 255];            // occupied — dark
        } else {
            color = [203, 213, 225, 255];         // unknown (val === -1) — grey
        }

        // Map origin is bottom-left, canvas is top-left → flip Y
        let x = i % width;
        let y = height - 1 - Math.floor(i / width);
        let pxIdx = (y * width + x) * 4;

        imgData.data[pxIdx]     = color[0];
        imgData.data[pxIdx + 1] = color[1];
        imgData.data[pxIdx + 2] = color[2];
        imgData.data[pxIdx + 3] = color[3];
    }

    mapCtx.putImageData(imgData, 0, 0);
    cachedMapImage = imgData;

    // Redraw pose on fresh map
    drawPoseLayer();
}

/**
 * drawPoseLayer — renders the robot triangle onto poseCanvas (overlay).
 * Called ONLY from socket.on('robot_pose') and after drawMapLayer().
 */
function drawPoseLayer() {
    if (!currentMap) return;

    const { width, height, resolution, origin } = currentMap;

    poseCtx.clearRect(0, 0, poseCanvas.width, poseCanvas.height);

    // Draw robot triangle
    if (robotPose && origin && origin.position) {
        let rx = (robotPose.x - origin.position.x) / resolution;
        let ry = (robotPose.y - origin.position.y) / resolution;
        ry = height - ry;  // flip Y

        poseCtx.save();
        poseCtx.translate(rx, ry);
        poseCtx.rotate(-robotPose.yaw);

        poseCtx.beginPath();
        poseCtx.moveTo(8, 0);
        poseCtx.lineTo(-6, -5);
        poseCtx.lineTo(-6, 5);
        poseCtx.closePath();
        poseCtx.fillStyle = '#ef4444';
        poseCtx.fill();
        poseCtx.restore();
    }
}

// ── Resize Handler ──────────────────────────────────────────────────────

window.addEventListener('resize', () => {
    resizeCanvases();
    if (cachedMapImage) mapCtx.putImageData(cachedMapImage, 0, 0);
    drawPoseLayer();
});
